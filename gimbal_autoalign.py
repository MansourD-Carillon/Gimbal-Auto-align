import sys
import atexit
import ctypes
import time

try:
    from simple_pid import PID as _PID
    _HAVE_SIMPLE_PID = True
except ImportError:
    _HAVE_SIMPLE_PID = False

sys.path.insert(0, r"C:\Users\uconn\Downloads\Millibox Gimbal Files (1)\Millibox Gimbal Files\SWMilliBox\MBX\python")
import mbx_functions as mbx
import mbx_instrument as equip

_BAUD = 1000000
_ANGLE_MIN = -62.0
_ANGLE_MAX = 62.0

# lower values mean the motor eases into its stop more gradually. [0< _ACCEL_SCALE <= 1 ] 
_ACCEL_SCALE_H = 0.1
_ACCEL_SCALE_V = 0.005

# Convenience aliases for the motor / gimbal-type constants (these never change at runtime)
H = mbx.H
V = mbx.V
P = mbx.P
TH = mbx.TH
PH = mbx.PH
HV = mbx.HV
SPHERICAL = mbx.SPHERICAL

# ################################################################################################a######################
# #  CLOSED-LOOP DIRECT-MOTION BEAM ALIGNMENT
# #  (reuses mbx.move_angle for motion and mbx.get_power / an injected measure_fn for VNA feedback)
# ######################################################################################################################

def measure_power_db(inst, freq_idx=None):
    """Single VNA/SA measurement -> scalar power in dB at one frequency.

    Wraps the existing mbx.get_power(), which returns (val, freq) lists across the VNA frequency list.
    By MBX coy nvention the midpoint index is used as the representative point, exactly like the existing
    beam_align routines (freqIdx = int(len(val)/2)).

    Returns (power_db, freq_hz) or (None, None) if no data came back.
    """
    val, freq = mbx.get_power(inst)
    if not val:
        return None, None
    if freq_idx is None:
        freq_idx = int(len(val) / 2)
    freq_idx = max(0, min(freq_idx, len(val) - 1))
    return float(val[freq_idx]), float(freq[freq_idx])


# GIM_MOTION is keyed by PHYSICAL motor id. Map the virtual axis ids used here onto those keys:
#   H->1, V->2, P->4 (HV gimbal);  theta TH->1 (= H motor),  phi PH->5 (= T motor) (spherical gimbal)
_LIMIT_KEY = {H: 1, V: 2, P: 4, TH: 1, PH: 5}


def _angle_limits(motor):
    """Return [min, max] software angle limits for a motor from the live GIM_MOTION structure."""
    gm = mbx.get_gim_motion()
    key = _LIMIT_KEY.get(motor, motor)
    try:
        return gm[key]["anglelim"]
    except (KeyError, TypeError):
        return [-90.0, 90.0]


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _clamp_angle(motor, ang):
    """Clamp a target angle to the motor's software angle limits (so move_angle never cancels the move)."""
    lo, hi = _angle_limits(motor)
    return _clamp(ang, lo, hi)


def _read_angle(motor):
    """Read the current settled pointing angle of a motor, in degrees."""
    return mbx.convertpostoangle(motor, mbx.current_pos(motor, 1))


class _PIDAxis:
    """Per-axis gradient PID for beam alignment and tracking.

    Uses simple_pid.PID when available (installed as simple-pid), otherwise falls back
    to an equivalent inline implementation. Setpoint is always 0 — the gradient is zero
    at the beam peak. Input is gradient in dB/degree; output is an angular correction in
    degrees, clamped to ±output_limit to prevent large jumps that would cause jerky motion.
    """

    def __init__(self, kp=0.6, ki=0.04, kd=0.08, output_limit=5.0):
        if _HAVE_SIMPLE_PID:
            self._pid = _PID(Kp=kp, Ki=ki, Kd=kd,
                             setpoint=0.0,
                             output_limits=(-output_limit, output_limit),
                             sample_time=None)
            self._use_lib = True
        else:
            self._kp, self._ki, self._kd = kp, ki, kd
            self._limit = output_limit
            self._integral = 0.0
            self._prev_grad = None
            self._use_lib = False

    def step(self, gradient):
        """Return angular correction (deg) for gradient signal (dB/deg).

        Positive gradient → peak is in the + direction → positive correction.
        """
        if self._use_lib:
            # Pass -gradient so simple_pid error = setpoint - (-gradient) = gradient,
            # giving output = Kp*gradient (move toward the peak, not away from it).
            return self._pid(-gradient)
        self._integral += gradient
        d = (gradient - self._prev_grad) if self._prev_grad is not None else 0.0
        self._prev_grad = gradient
        output = self._kp * gradient + self._ki * self._integral + self._kd * d
        if abs(output) >= self._limit:
            self._integral -= gradient   # anti-windup: undo accumulation when saturated
            output = _clamp(output, -self._limit, self._limit)
        return output

    def reset(self):
        if self._use_lib:
            self._pid.reset()
        else:
            self._integral = 0.0
            self._prev_grad = None


def _parabola_vertex(x1, y1, x2, y2, x3, y3):
    """Fit y = a*x^2 + b*x + c through three (x, y) samples and return the vertex x.

    Returns (x_vertex, is_concave). `is_concave` is True only when a < 0, i.e. the parabola has a true
    MAXIMUM. If the three points are degenerate/colinear/convex the vertex is meaningless and the caller
    should fall back to a gradient step; in that case is_concave is False and x_vertex is None.

    Points need NOT be symmetric or evenly spaced, so this stays valid even after the probe angles have
    been clamped to the software limits.
    """
    denom = (x1 - x2) * (x1 - x3) * (x2 - x3)
    if denom == 0:
        return None, False
    a = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / denom
    b = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / denom
    if a >= 0 or abs(a) < 1e-12:                                # not concave -> no usable maximum
        return None, False
    return (-b / (2.0 * a)), True


# Per-axis direct-motion refinement

def _refine_axis(meas, move_to, center_angle, center_power, probe, max_move, motor):
    """Refine one axis: probe center +/- probe, fit a parabola, and pick a new target.

    If the parabola is concave the analytic vertex is used; otherwise take a gradient step
    toward the higher neighbour. Results are clamped to +/- max_move and motor limits.

    meas() -> power (dB) or None. move_to(angle) performs the 1-axis direct move.
    Returns (target_angle, best_power_seen, n_probe_moves).
    """
    lo, hi = _angle_limits(motor)

    # --- probe the two neighbours with direct moves ---
    a_minus = _clamp(center_angle - probe, lo, hi)
    a_plus = _clamp(center_angle + probe, lo, hi)

    move_to(a_minus)
    p_minus = meas()

    move_to(a_plus)
    p_plus = meas()

    # track the best of the three samples so we never report a target worse than what we have already seen
    samples = [(center_angle, center_power), (a_minus, p_minus), (a_plus, p_plus)]
    samples = [s for s in samples if s[1] is not None]
    if not samples:
        # instrument returned no data for any probe -> do not move this axis
        return center_angle, None, 2
    best_angle, best_power = max(samples, key=lambda s: s[1])

    # --- estimate the new target ---
    target = None
    if p_minus is not None and p_plus is not None and center_power is not None:
        vertex, concave = _parabola_vertex(a_minus, p_minus, center_angle, center_power, a_plus, p_plus)
        if concave and vertex is not None:
            target = vertex                                     # analytic peak -> overshoot-free

    if target is None:                                          # gradient-direction fallback
        if p_plus is not None and p_minus is not None and p_plus != p_minus:
            direction = 1.0 if p_plus > p_minus else -1.0
            target = center_angle + direction * probe
        else:
            target = best_angle                                 # flat: stay at the best sampled point

    # constrain the jump: bounded step + software limits
    target = _clamp(target, center_angle - max_move, center_angle + max_move)
    target = _clamp(target, lo, hi)
    return target, best_power, 2


def _refine_axis_pid(meas, move_to, center_angle, center_power, probe, max_move, motor, pid):
    """PID-corrected axis refinement — same two probe moves as _refine_axis, but the
    gradient feeds a _PIDAxis instance that accumulates integral and derivative state
    across successive passes.

    I-term corrects sustained pointing offset (cable drag, mechanical bias).
    D-term damps oscillation when the gradient flips sign between passes.
    The output is bounded by pid.output_limit so no single pass makes a large jump.

    Falls back to the best raw sample and resets PID state if the VNA returns no data.
    Returns (target_angle, best_power_seen, n_probe_moves).
    """
    lo, hi = _angle_limits(motor)
    a_minus = _clamp(center_angle - probe, lo, hi)
    a_plus  = _clamp(center_angle + probe, lo, hi)

    move_to(a_minus); p_minus = meas()
    move_to(a_plus);  p_plus  = meas()

    samples = [(center_angle, center_power), (a_minus, p_minus), (a_plus, p_plus)]
    samples = [s for s in samples if s[1] is not None]
    if not samples:
        pid.reset()
        return center_angle, None, 2
    best_angle, best_power = max(samples, key=lambda s: s[1])

    if p_minus is not None and p_plus is not None:
        span = a_plus - a_minus
        if span > 0:
            grad       = (p_plus - p_minus) / span          # dB/deg — zero at peak
            correction = pid.step(grad)                      # PID output: deg toward peak
            target     = _clamp(center_angle + correction,
                                center_angle - max_move, center_angle + max_move)
            target     = _clamp(target, lo, hi)
            return target, best_power, 2

    pid.reset()
    return best_angle, best_power, 2


# ======================================================================================================================
#  CONTINUOUS-SWEEP AXIS REFINEMENT
# ======================================================================================================================

def _sweep_axis_continuous(meas, move_to, motor, center_angle, probe, max_move):
    """Non-stopping version of _refine_axis.

    Moves to the sweep start (blocking via move_to), then fires the non-blocking
    move_pos() to the sweep end. Samples (angle, power) pairs the whole time the
    motor is in motion. Fits a parabola through the three samples around the peak to
    sub-sample the vertex; falls back to the best raw sample if the fit is concave.

    Returns (target_angle, best_power, n_samples).
    """
    lo, hi = _angle_limits(motor)
    a_start = _clamp(center_angle - probe, lo, hi)
    a_end   = _clamp(center_angle + probe, lo, hi)

    move_to(a_start)                                        # settle at sweep start (blocking)

    pos_end = mbx.convertangletopos(motor, a_end)
    mbx.write_accel()                                       # re-enforce _ACCEL_SCALE ramps before sweep
    mbx.move_pos(motor, pos_end)                            # kick off sweep non-blocking

    samples  = []
    _t_ang   = []    # (time, angle) for velocity estimation — not returned
    _WIN     = 4     # samples to average over per velocity window
    _v_peak  = 0.0
    _cruise_printed = False
    _decel_printed  = False

    def _wv(end):
        """Mean angular speed (deg/s) over the _WIN samples ending at index `end`."""
        start = max(0, end - _WIN + 1)
        if end <= start:
            return 0.0
        t0, a0 = _t_ang[start]
        t1, a1 = _t_ang[end]
        dt = t1 - t0
        return abs(a1 - a0) / dt if dt > 1e-4 else 0.0

    while mbx.check_is_moving():
        try:
            angle = mbx.convertpostoangle(motor, mbx.current_pos(motor, 1))
            _t_ang.append((time.time(), angle))
            p = meas()
            if p is not None:
                samples.append((angle, p))
        except Exception:
            pass

        n = len(_t_ang) - 1
        if n >= _WIN * 2:
            v_now  = _wv(n)
            v_prev = _wv(n - _WIN)
            if v_now > _v_peak:
                _v_peak = v_now

            if (not _cruise_printed and _v_peak > 0.2
                    and abs(v_now - v_prev) < 0.08 * _v_peak):
                angle_now = _t_ang[n][1]
                print(f" P1: {v_now:.2f} °/s  at {angle_now:.2f}°")
                GimbalController._print_position()
                _cruise_printed = True

            if (not _decel_printed and _cruise_printed
                    and v_now < _v_peak * 0.90):
                angle_now = _t_ang[n][1]
                print(f" P2:  {v_now:.2f} °/s  at {angle_now:.2f}°")
                GimbalController._print_position()
                _decel_printed = True

    try:                                                    # one final sample at rest
        angle = mbx.convertpostoangle(motor, mbx.current_pos(motor, 1))
        p = meas()
        if p is not None:
            samples.append((angle, p))
    except Exception:
        pass

    if not samples:
        return center_angle, None, 0

    best_angle, best_power = max(samples, key=lambda s: s[1])
    target = best_angle

    if len(samples) >= 3:
        peak_i = max(range(len(samples)), key=lambda i: samples[i][1])
        i0 = max(0, peak_i - 1)
        i2 = min(len(samples) - 1, peak_i + 1)
        if i0 != i2:
            x1, y1 = samples[i0]
            x2, y2 = samples[peak_i]
            x3, y3 = samples[i2]
            vertex, concave = _parabola_vertex(x1, y1, x2, y2, x3, y3)
            if concave and vertex is not None:
                target = vertex

    target = _clamp(target, center_angle - max_move, center_angle + max_move)
    target = _clamp(target, lo, hi)
    return target, best_power, len(samples)


# ======================================================================================================================
#  SHARED PASS LOOP
# ======================================================================================================================

def _beam_align_pass_loop(meas, move1, move2, motor1, motor2,
                           start1, start2, p_start,
                           max_passes, init_probe, min_probe, probe_decay,
                           max_move, tol_db, tol_ang,
                           continuous1, continuous2,
                           axis1_name, axis2_name, gimbal_label,
                           inst, verbose,
                           pid1=None, pid2=None):
    """Iterative probe-and-refine loop shared by HV and spherical alignment.

    Returns (best1, best2, best_p, passes_used, total_moves) on success, or
    (None, None, None, 0, 0) on abort (cont_trigger already fired).
    total_moves starts at 2 to account for the two initial axis moves the caller performed.
    """
    a1, a2      = start1, start2
    p_cur       = p_start
    best1, best2, best_p = a1, a2, (p_cur if p_cur is not None else -1e9)
    probe       = float(init_probe)
    total_moves = 2

    if verbose:
        print("\n==== CLOSED-LOOP DIRECT-MOTION ALIGNMENT (%s) ====" % gimbal_label)
        print("start (%s,%s) = (%0.3f, %0.3f)   P = %s dB"
              % (axis1_name, axis2_name, a1, a2, _fmt(p_cur)))

    k = -1
    for k in range(max_passes):
        if _aborted():
            _safe_cont_trigger(inst)
            print("*** alignment aborted ***")
            return None, None, None, 0, 0

        if verbose:
            print("\n-- pass %d/%d  (probe radius = %0.2f deg) --" % (k + 1, max_passes, probe))

        if continuous1:
            t1, _, n1 = _sweep_axis_continuous(meas, move1, motor1, a1, probe, max_move)
        elif pid1 is not None:
            t1, _, n1 = _refine_axis_pid(meas, move1, a1, p_cur, probe, max_move, motor1, pid1)
        else:
            t1, _, n1 = _refine_axis(meas, move1, a1, p_cur, probe, max_move, motor1)
        move1(t1)
        p_after1 = meas()
        total_moves += n1 + 1

        if continuous2:
            t2, _, n2 = _sweep_axis_continuous(meas, move2, motor2, a2, probe, max_move)
        elif pid2 is not None:
            t2, _, n2 = _refine_axis_pid(meas, move2, a2, p_after1, probe, max_move, motor2, pid2)
        else:
            t2, _, n2 = _refine_axis(meas, move2, a2, p_after1, probe, max_move, motor2)
        move2(t2)
        p_after2 = meas()
        total_moves += n2 + 1

        d1    = t1 - a1
        d2    = t2 - a2
        new_p = p_after2 if p_after2 is not None else best_p

        if verbose:
            print("   -> target (%s,%s) = (%0.3f, %0.3f)   d%s=%+0.3f d%s=%+0.3f   P = %s dB"
                  % (axis1_name, axis2_name, t1, t2,
                     axis1_name, d1, axis2_name, d2, _fmt(new_p)))

        if new_p > best_p:
            best1, best2, best_p = t1, t2, new_p

        improvement = new_p - p_cur if (p_cur is not None and new_p is not None) else 0.0
        a1, a2, p_cur = t1, t2, new_p

        if max(abs(d1), abs(d2)) < tol_ang and abs(improvement) < tol_db:
            if verbose:
                print("   converged (update < %0.2f deg, dP < %0.2f dB)" % (tol_ang, tol_db))
            break

        probe = max(min_probe, probe * probe_decay)

    return best1, best2, best_p, k + 1, total_moves


# ======================================================================================================================
#  MAIN ROUTINE - HV GIMBAL (azimuth / elevation)
# ======================================================================================================================

def beam_align_hv_directmotion(inst, pangle=0.0, accuracy="VERY HIGH",
                               max_passes=5, init_probe=8.0, min_probe=0.5,
                               probe_decay=0.5, max_move=60.0,
                               tol_db=0.05, tol_ang=0.10,
                               start_h=None, start_v=None, verbose=True,
                               measure_fn=None, continuous=False,
                               use_pid=False, pid_kp=0.6, pid_ki=0.04, pid_kd=0.08):
    """Closed-loop HV gimbal alignment using direct-motion probes and parabola fits.

    continuous=True: each axis is swept without stopping while VNA samples are
    collected in flight; a parabola is fit to the captured samples to find the vertex.
    continuous=False (default): classic discrete probe-stop-measure behaviour.

    use_pid=True: replaces the single-pass parabola fit with a _PIDAxis controller
    that accumulates integral and derivative state across passes. Automatically
    forces continuous=False (PID needs per-step discrete measurements). The I-term
    corrects sustained pointing offset; the D-term damps pass-to-pass oscillation.

    Returns (H_off, V_off) or (None, None) on error.
    """
    if mbx.gim_type != HV:
        print("*** ERROR: gimbal is not HV - use beam_align_sph_directmotion() instead")
        return None, None

    t0 = time.time()

    if mbx.num_motors >= 4:
        mbx.move_angle(pang=pangle, accuracy="HIGH")

    h0 = _read_angle(H) if start_h is None else float(start_h)
    v0 = _read_angle(V) if start_v is None else float(start_v)

    try:
        inst.fix_status()
    except Exception:
        pass

    def meas():
        if measure_fn is not None:
            return measure_fn()
        p, _ = measure_power_db(inst)
        return p

    def move_h(ang): mbx.move_angle(hang=ang, accuracy="HIGH")
    def move_v(ang): mbx.move_angle(vang=ang, accuracy="HIGH")

    move_h(_clamp_angle(H, h0))
    move_v(_clamp_angle(V, v0))
    p_cur = meas()

    if use_pid:
        continuous = False   # PID needs discrete per-step measurements
        pid1 = _PIDAxis(pid_kp, pid_ki, pid_kd)
        pid2 = _PIDAxis(pid_kp, pid_ki, pid_kd)
    else:
        pid1 = pid2 = None

    best_h, best_v, best_p, passes_used, total_moves = _beam_align_pass_loop(
        meas, move_h, move_v, H, V, h0, v0, p_cur,
        max_passes, init_probe, min_probe, probe_decay, max_move,
        tol_db, tol_ang,
        continuous1=continuous, continuous2=continuous,
        axis1_name="H", axis2_name="V", gimbal_label="HV",
        inst=inst, verbose=verbose,
        pid1=pid1, pid2=pid2,
    )

    if best_h is None:
        return None, None

    if best_p <= -1e8:
        print("*** ERROR: no valid VNA measurement obtained - alignment not performed")
        _safe_cont_trigger(inst)
        return None, None

    h_final = _clamp_angle(H, best_h)
    v_final = _clamp_angle(V, best_v)
    mbx.move_angle(hang=h_final, accuracy=accuracy)
    mbx.move_angle(vang=v_final, accuracy=accuracy)
    _safe_cont_trigger(inst)

    t1 = time.time()
    if verbose:
        print("\n******************************************")
        print("**** DIRECT-MOTION BEAM ALIGNMENT RESULT ****")
        print("******************************************")
        print("         (H,V) = (%0.3f, %0.3f)   P = %s dB" % (h_final, v_final, _fmt(best_p)))
        print("   passes used = %d   direct moves = %d   elapsed = %0.1f s"
              % (passes_used, total_moves + 2, (t1 - t0)))
        print("")

    return h_final, v_final


# ======================================================================================================================
#  MAIN ROUTINE - SPHERICAL GIMBAL (theta / phi)
# ======================================================================================================================

def beam_align_sph_directmotion(inst, accuracy="VERY HIGH",
                                max_passes=5, init_probe=8.0, min_probe=0.5,
                                probe_decay=0.5, max_move=60.0,
                                tol_db=0.05, tol_ang=0.10,
                                start_th=None, start_ph=None, verbose=True,
                                measure_fn=None, continuous=False,
                                use_pid=False, pid_kp=0.6, pid_ki=0.04, pid_kd=0.08):
    """Closed-loop direct-motion alignment for a SPHERICAL gimbal (theta = TH, phi = PH).

    continuous=True: TH axis swept without stopping (same as HV). PH axis stays
    discrete because it drives two motors (T+Z) whose combined position is a list,
    not a scalar angle, which the sweep sampler cannot handle.

    use_pid=True: same PID gradient control as the HV variant (see beam_align_hv_directmotion).
    Forces continuous=False on both axes.
    """
    if mbx.gim_type != SPHERICAL:
        print("*** ERROR: gimbal is not SPHERICAL - use beam_align_hv_directmotion() instead")
        return None, None

    t0 = time.time()

    th0     = _read_angle(TH) if start_th is None else float(start_th)
    ph_pair = mbx.convertpostoangle(PH, mbx.current_pos(PH, 1))
    phi0    = ph_pair[0] if start_ph is None else float(start_ph)
    dphi    = ph_pair[1]

    try:
        inst.fix_status()
    except Exception:
        pass

    def meas():
        if measure_fn is not None:
            return measure_fn()
        p, _ = measure_power_db(inst)
        return p

    def move_th(ang): mbx.move_angle(thang=ang, accuracy="HIGH")
    def move_ph(ang): mbx.move_angle(phang=[ang, dphi], accuracy="HIGH")

    move_th(_clamp_angle(TH, th0))
    move_ph(phi0)
    p_cur = meas()

    if use_pid:
        continuous = False
        pid1 = _PIDAxis(pid_kp, pid_ki, pid_kd)
        pid2 = _PIDAxis(pid_kp, pid_ki, pid_kd)
    else:
        pid1 = pid2 = None

    best_th, best_ph, best_p, passes_used, total_moves = _beam_align_pass_loop(
        meas, move_th, move_ph, TH, PH, th0, phi0, p_cur,
        max_passes, init_probe, min_probe, probe_decay, max_move,
        tol_db, tol_ang,
        continuous1=continuous, continuous2=False,
        axis1_name="TH", axis2_name="PHI", gimbal_label="SPHERICAL",
        inst=inst, verbose=verbose,
        pid1=pid1, pid2=pid2,
    )

    if best_th is None:
        return None, None

    if best_p <= -1e8:
        print("*** ERROR: no valid VNA measurement obtained - alignment not performed")
        _safe_cont_trigger(inst)
        return None, None

    th_final = _clamp_angle(TH, best_th)
    mbx.move_angle(thang=th_final, accuracy=accuracy)
    mbx.move_angle(phang=[best_ph, dphi], accuracy=accuracy)
    _safe_cont_trigger(inst)

    t1 = time.time()
    if verbose:
        print("\n******************************************")
        print("**** DIRECT-MOTION BEAM ALIGNMENT RESULT ****")
        print("******************************************")
        print("         (TH,PHI) = (%0.3f, %0.3f)   P = %s dB" % (th_final, best_ph, _fmt(best_p)))
        print("   passes used = %d   direct moves = %d   elapsed = %0.1f s"
              % (passes_used, total_moves + 2, (t1 - t0)))
        print("")

    return th_final, best_ph


# ======================================================================================================================
#  small internal utilities
# ======================================================================================================================

def _fmt(x):
    return "n/a" if (x is None or x <= -1e8) else ("%0.2f" % x)


def _aborted():
    """Non-blocking ESC-to-abort check, mirroring the existing sweep routines."""
    try:
        if mbx.kbhit():
            return mbx.check_abort()
    except Exception:
        pass
    return False


def _safe_cont_trigger(inst):
    try:
        inst.cont_trigger()
    except Exception:
        pass


# ======================================================================================================================
#  generic entry point
# ======================================================================================================================

def beam_align_directmotion(inst, **kwargs):
    """Dispatch to the HV or SPHERICAL routine based on the live gimbal type."""
    if mbx.gim_type == HV:
        return beam_align_hv_directmotion(inst, **kwargs)
    elif mbx.gim_type == SPHERICAL:
        return beam_align_sph_directmotion(inst, **kwargs)
    else:
        print("*** ERROR: unknown gimbal type")
        return None, None


# ######################################################################################################################
# #  SWEEPING BEAM SIMULATION
# #  Extends VNA_Generic (the built-in MBX simulation stub) with a Gaussian beam pattern that responds
# #  to the live gimbal position, so the alignment algorithm has a real peak to converge on.
# ######################################################################################################################

class _SweepingBeamVNA(equip.VNA_Generic):
    """VNA_Generic with position-driven Gaussian beam.

    Each call to get_s_dbphase() reads the live H/V angles and computes power on a
    Gaussian pattern centred at (_PEAK_H, _PEAK_V) with a 20-degree half-power beamwidth.
    Higher frequencies get a slightly narrower beam to reflect realistic scaling.
    """
    _PEAK_H   = 15.0    # beam peak azimuth (deg)
    _PEAK_V   = -8.0    # beam peak elevation (deg)
    _HPBW     = 20.0    # full half-power beamwidth (deg)
    _PEAK_DB  = -20.0   # on-axis signal level (dBm)
    _FLOOR_DB = -80.0   # noise floor (dBm)

    def get_s_dbphase(self):
        try:
            h = mbx.convertpostoangle(H, mbx.current_pos(H, 1))
            v = mbx.convertpostoangle(V, mbx.current_pos(V, 1))
        except Exception:
            h, v = 0.0, 0.0
        half_bw = self._HPBW / 2.0
        off_sq = ((h - self._PEAK_H) / half_bw) ** 2 + ((v - self._PEAK_V) / half_bw) ** 2
        freqs = self.get_freq_list()
        db, phase = [], []
        for i in range(len(freqs)):
            bw_scale = 1.0 - 0.03 * i          # 3 % narrower beam per GHz step upward
            p = self._PEAK_DB - 3.0 * off_sq / (bw_scale ** 2)
            db.append(max(p, self._FLOOR_DB))
            phase.append(0.0)
        return db, phase


# ######################################################################################################################
# #  GIMBAL CONTROLLER
# ######################################################################################################################

class GimbalController:

    def __init__(self, port="COM7"):
        self._port = port
        self._connected = False
        self._vna = None
        if not mbx.connect(self._port, _BAUD):
            raise ConnectionError(
                f"Failed to connect to GIM04 on {self._port} at {_BAUD} bps"
            )
        self._connected = True
        mbx.set_gim_motion_default()
        # Cap both axes to 10 % of hardware maximum (raw register 102 / 1023 for XH540).
        # Keep V at 1% of max and H at 20%
        gm = mbx.get_gim_motion()
        if 1 in gm:
            gm[1]["vel"]  = 12
            #round(0.05 * mbx.max_H_velocity * mbx.base_vel_unit / mbx.base_ratio, 4)
            gm[1]["accel"] = max(1, round(gm[1]["accel"] * _ACCEL_SCALE_H))
        if 2 in gm:
            gm[2]["vel"]  = 3
            #round(0.05 * mbx.XH540_MAX_PROFILE_VELOCITY * mbx.XH540_VELOCITY_UNIT / mbx.XH540_V_RATIO, 4)
            gm[2]["accel"] = max(1, round(gm[2]["accel"] * _ACCEL_SCALE_V))
        mbx.set_gim_motion(gm)
        atexit.register(self._on_exit)
        self._print_position()

    def _require_connection(self):
        if not self._connected:
            raise RuntimeError("Positioner is not connected")

    def _validate_angle(self, axis, angle):
        if not (_ANGLE_MIN <= angle <= _ANGLE_MAX):
            raise ValueError(
                f"{axis} angle {angle:.4f}° is outside [{_ANGLE_MIN}, {_ANGLE_MAX}]"
            )

    def move(self, h=None, v=None):
        self._require_connection()
        if h is None and v is None:
            raise ValueError("At least one of h or v must be provided")
        if h is not None:
            self._validate_angle("H", h)
        if v is not None:
            self._validate_angle("V", v)
        ok = mbx.move_angle(hang=h, vang=v, accuracy="HIGH")
        self._print_position()
        try:
            if not ok:
                raise RuntimeError(f"move_angle failed — H={h}, V={v}")
        finally:
            mbx.gotoZERO(accuracy="HIGH")
            self._print_position()

    def move_h(self, angle):
        self.move(h=angle)

    def move_v(self, angle):
        self.move(v=angle)

    # ------------------------------------------------------------------
    #  Persistent direct positioning (does NOT auto-home, unlike move())
    # ------------------------------------------------------------------
    def goto(self, h=None, v=None, accuracy="HIGH"):
        """Move directly to (h, v) and STAY there. Used by the alignment workflow.

        Unlike move(), this does not return to zero afterwards. It is the COM-level
        positioning primitive the alignment relies on (via mbx.move_angle)."""
        self._require_connection()
        if h is None and v is None:
            raise ValueError("At least one of h or v must be provided")
        if h is not None:
            self._validate_angle("H", h)
        if v is not None:
            self._validate_angle("V", v)
        ok = mbx.move_angle(hang=h, vang=v, accuracy=accuracy)
        self._print_position()
        if not ok:
            raise RuntimeError(f"move_angle failed — H={h}, V={v}")
        return ok

    def connect_vna(self, visa_addr):
        self._vna = equip.inst_setup_single("VNA", visa_addr)
        if not self._vna.port_open:
            self._vna = None
            raise ConnectionError(f"VNA not reachable at {visa_addr}")
        self._vna.init_meas()

    def connect_simulated_vna(self):
        """Beam-sweeping simulation built on the VNA_Generic stub — no physical instrument needed."""
        vna = _SweepingBeamVNA()
        vna.port_open = 1
        self._vna = vna
        self._vna.init_meas()

    def measure(self):
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")
        self._vna.single_trigger()
        freqs = self._vna.get_freq_list()
        db, phase = self._vna.get_s_dbphase()
        self._vna.cont_trigger()
        return {"freqs": freqs, "db": db, "phase": phase}

    def _print_measurement(self, data):
        pos = self.position()
        print(f"--- Measurement  H: {pos['H']:.2f}°  V: {pos['V']:.2f}° ---")
        for f, d, p in zip(data["freqs"], data["db"], data["phase"]):
            print(f"  {f/1e9:.4f} GHz  {d:.2f} dB  {p:.2f}°")

    def home(self):
        self._require_connection()
        gm = mbx.get_gim_motion()
        saved = {}
        for k in (1, 2):
            if k in gm:
                saved[k] = gm[k]["vel"]
                gm[k]["vel"] = max(1, gm[k]["vel"] // 16)
        mbx.set_gim_motion(gm)
        try:
            mbx.gotoZERO(accuracy="HIGH")
        finally:
            for k, v in saved.items():
                gm[k]["vel"] = v
            mbx.set_gim_motion(gm)
        self._print_position()

    # ==================================================================
    #  CLOSED-LOOP DIRECT-MOTION ALIGNMENT  (replaces the grid scan)
    # ==================================================================

    def _align_measure(self, freq_mode="mid"):
        """Scalar feedback for the alignment loop, taken from our own VNA pipeline.

        Returns a single power value in dB (or None if the VNA returned nothing).
        freq_mode:
            "mid"  -> the midpoint frequency point (MBX convention, smooth single-tone
                      objective -> best for gradient/parabola convergence).
            "peak" -> the strongest point across the band (matches run_scan's metric).
        """
        data = self.measure()
        db = data.get("db") or []
        if not db:
            return None
        if freq_mode == "peak":
            return float(max(db))
        return float(db[len(db) // 2])

    def run_direct_align(self, max_passes=5, init_probe=8.0, min_probe=0.5,
                         max_move=60.0, accuracy="VERY HIGH", pangle=0.0,
                         freq_mode="mid", enter_keyboard=True,
                         use_pid=False, enter_track=False):
        """Point the gimbal at the direction of maximum VNA signal using closed-loop
        direct motion (no full grid sweep, <= max_passes correction passes).

        All motion goes through mbx.move_angle over the COM link; feedback comes from
        this controller's measure() pipeline via the injected measure_fn. The gimbal is
        left parked at the located peak.

        use_pid=True  — use PID gradient control instead of single-pass parabola fit.
                        Automatically switches to discrete probe mode. Recommended when
                        the beam has a mechanical pointing bias that accumulates over passes.
        enter_track=True — after alignment, enter run_track() to hold the peak
                           continuously. Implies enter_keyboard=False.

        Returns (H, V) of the peak, or None if it could not complete.
        """
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        print("\nStarting closed-loop DIRECT-MOTION alignment "
              f"(max {max_passes} passes, initial probe {init_probe}°)...")

        # wrap measure_fn to record (H, V, db_array) at every VNA sample
        _log = []
        _freqs = []
        def _recording_measure():
            data = self.measure()
            db = data.get("db") or []
            if not _freqs:
                _freqs.extend(data.get("freqs") or [])
            if not db:
                _log.append((None, None, []))
                return None
            p = float(max(db)) if freq_mode == "peak" else float(db[len(db) // 2])
            try:
                pos = self.position()
                _log.append((pos["H"], pos["V"], list(db)))
            except Exception:
                _log.append((None, None, list(db)))
            return p

        result = beam_align_directmotion(
            self._vna,                                  # used for fix_status / cont_trigger only
            measure_fn=_recording_measure,              # feedback via our VNA pipeline + logging
            accuracy=accuracy,                          # final lock-in accuracy
            max_passes=max_passes,
            init_probe=init_probe,
            min_probe=min_probe,
            max_move=max_move,
            pangle=pangle,                              # HV only; ignored for spherical
            verbose=True,
            continuous=not use_pid,                     # PID needs discrete probe mode
            use_pid=use_pid,
        )

        # beam_align_directmotion returns a 2-tuple for both HV (H,V) and spherical (TH,PHI)
        a, b = result if result is not None else (None, None)
        if a is None or b is None:
            print("\nAlignment did not complete (aborted, wrong gimbal type, or no VNA data).")
            self.home()
            return None

        # the alignment already performed the final direct move to the peak; report + verify
        print(f"\nPeak located and parked:  axis1={a:.3f}°  axis2={b:.3f}°")
        self._print_position()
        if self._vna is not None:
            self._print_measurement(self.measure())

        # plot power measurements collected during alignment
        if _log:
            self._plot_alignment(_log, _freqs)

        if enter_track:
            print("\nAlignment complete. Entering PID tracker — Q/ESC/X to stop and hold position.")
            self.run_track()
        elif enter_keyboard:
            print("\nParked at peak. Entering keyboard control -- Q/ESC/X: quit & hold position  |  H: go home")
            self.run_keyboard_control(start_h=a, start_v=b)
        return a, b

    def _plot_alignment(self, log, freqs=None):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed — skipping alignment plot")
            return

        # log entries: (H, V, db_list)
        valid = [(h, v, db) for h, v, db in log if db]
        if not valid:
            print("No valid measurements to plot.")
            return

        hs     = [h  for h, _, _  in valid]
        vs     = [v  for _, v, _  in valid]
        dbs    = [db for _, _, db in valid]
        powers = [db[len(db) // 2] for db in dbs]
        idxs   = list(range(len(valid)))
        peak_i = powers.index(max(powers))

        # shared frequency axis for spectrum plots
        n_freqs = len(dbs[peak_i])
        if freqs and len(freqs) == n_freqs:
            freq_x     = [f / 1e9 for f in freqs]
            freq_label = "Frequency (GHz)"
        else:
            freq_x     = list(range(n_freqs))
            freq_label = "Frequency index"

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("Direct-Motion Alignment — VNA Power Log", fontweight="bold")
        ax1, ax2, ax3, ax4 = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

        # ---- power convergence trace ----
        ax1.plot(idxs, powers, "b-o", markersize=4, linewidth=1.2, label="Mid-band power")
        ax1.axvline(peak_i, color="red", linestyle="--",
                    label=f"Peak  {max(powers):.2f} dB  (meas #{peak_i})")
        ax1.set_xlabel("Measurement index")
        ax1.set_ylabel("Power (dB)")
        ax1.set_title("Power vs Measurement")
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.4)

        # ---- gimbal path coloured by power ----
        has_pos = any(h is not None for h in hs)
        if has_pos:
            sc = ax2.scatter(hs, vs, c=powers, cmap="hot", s=60, zorder=3,
                             vmin=min(powers), vmax=max(powers))
            plt.colorbar(sc, ax=ax2, label="Power (dB)")
            ax2.plot(hs[0],      vs[0],      "gs", markersize=10, zorder=4, label="Start")
            ax2.plot(hs[-1],     vs[-1],     "b^", markersize=10, zorder=4, label="Parked")
            ax2.plot(hs[peak_i], vs[peak_i], "r*", markersize=14, zorder=5,
                     label=f"Peak  {max(powers):.2f} dB")
            ax2.set_xlabel("H angle (deg)")
            ax2.set_ylabel("V angle (deg)")
            ax2.set_title("Gimbal Path (colour = power)")
            ax2.legend(fontsize=8)
            ax2.grid(True, alpha=0.4)
        else:
            ax2.plot(idxs, powers, "r-o", markersize=4)
            ax2.set_xlabel("Measurement index")
            ax2.set_ylabel("Power (dB)")
            ax2.set_title("Power (no position data)")
            ax2.grid(True, alpha=0.4)

        # ---- full power spectrum at peak position ----
        peak_db  = dbs[peak_i]
        peak_pos = (f"H={hs[peak_i]:.2f}°  V={vs[peak_i]:.2f}°"
                    if has_pos and hs[peak_i] is not None else "peak position")
        ax3.plot(freq_x, peak_db, "g-o", markersize=5, linewidth=1.5)
        ax3.set_xlabel(freq_label)
        ax3.set_ylabel("Power (dB)")
        ax3.set_title(f"Spectrum at Peak  ({peak_pos})")
        ax3.grid(True, alpha=0.4)

        # ---- per-frequency best H and V angle ----
        pos_valid = [(h, v, db) for h, v, db in valid if h is not None and v is not None]
        if has_pos and pos_valid and n_freqs > 0:
            best_h_per_freq = []
            best_v_per_freq = []
            for fi in range(n_freqs):
                best = max(pos_valid, key=lambda x, fi=fi: x[2][fi] if fi < len(x[2]) else -1e9)
                best_h_per_freq.append(best[0])
                best_v_per_freq.append(best[1])
            ax4.plot(freq_x, best_h_per_freq, "b-o", markersize=4, linewidth=1.2, label="H")
            ax4.plot(freq_x, best_v_per_freq, "r-o", markersize=4, linewidth=1.2, label="V")
            ax4.set_xlabel(freq_label)
            ax4.set_ylabel("Angle (deg)")
            ax4.set_title("Per-Frequency Best Pointing Angle")
            ax4.legend(fontsize=8)
            ax4.grid(True, alpha=0.4)
        else:
            ax4.set_visible(False)

        plt.tight_layout()
        plt.show()

    def _grid_scan(self, h_lo, h_hi, h_step, v_lo, v_hi, v_step, label):
        h_angles = []
        h = h_lo
        while h <= h_hi + h_step * 0.01:
            h_angles.append(round(h, 4))
            h += h_step

        v_angles = []
        v = v_lo
        while v <= v_hi + v_step * 0.01:
            v_angles.append(round(v, 4))
            v += v_step

        total = len(h_angles) * len(v_angles)
        print(f"\n{label} scan: {len(h_angles)} H x {len(v_angles)} V = {total} points")

        best_h, best_v, best_db = 0.0, 0.0, float("-inf")
        count = 0
        for h in h_angles:
            for v in v_angles:
                count += 1
                mbx.move_angle(hang=h, vang=v, accuracy="HIGH")
                data = self.measure()
                peak = max(data["db"])
                if peak > best_db:
                    best_db = peak
                    best_h = h
                    best_v = v
                print(
                    f"  [{count:4d}/{total}] H:{h:7.2f}  V:{v:7.2f}  {peak:7.2f} dB"
                    f"  (best H:{best_h:.2f}  V:{best_v:.2f}  {best_db:.2f} dB)"
                )

        return best_h, best_v, best_db

    def run_scan(self, coarse_step=45.0, fine_step=11.25):
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        print("Starting coarse scan (45 deg steps)...")
        best_h, best_v, best_db = self._grid_scan(
            -180.0, 180.0, coarse_step,
            -180.0, 180.0, coarse_step,
            "Coarse"
        )
        print(f"\nCoarse peak: H={best_h:.2f}  V={best_v:.2f}  {best_db:.2f} dB")

        h_lo = max(-180.0, best_h - coarse_step)
        h_hi = min(180.0, best_h + coarse_step)
        v_lo = max(-180.0, best_v - coarse_step)
        v_hi = min(180.0, best_v + coarse_step)

        print(f"\nStarting fine scan ({fine_step} deg steps around peak)...")
        best_h, best_v, best_db = self._grid_scan(
            h_lo, h_hi, fine_step,
            v_lo, v_hi, fine_step,
            "Fine"
        )

        print(f"\nPeak found: H={best_h:.2f}  V={best_v:.2f}  {best_db:.2f} dB")
        print("Moving to peak...")
        mbx.move_angle(hang=best_h, vang=best_v, accuracy="HIGH")
        self._print_position()
        print("\nParked at peak. Entering keyboard control -- Q/ESC/X: quit & hold position  |  H: go home")
        self.run_keyboard_control(start_h=best_h, start_v=best_v)

    def run_keyboard_control(self, start_h=0.0, start_v=0.0):
        self._require_connection()

        _k = ctypes.windll.user32.GetAsyncKeyState
        def held(vk): return bool(_k(vk) & 0x8000)

        VK = {
            "up": 0x26, "down": 0x28, "left": 0x25, "right": 0x27,
            "i":  0x49, "k":   0x4B, "j":   0x4A,  "l":   0x4C,
            "a":  0x41, "s":   0x53, "m":   0x4D,
            "q":  0x51, "esc": 0x1B,
            "x":  0x58, "h":   0x48,
        }

        step = 11.25
        h = start_h
        v = start_v
        prev_m = False
        prev_a = False
        prev_s = False
        prev_h = False
        print("Hold arrows/IJKL: move  |  A/S: step  |  M: measure  |  H: go home  |  Q/ESC/X: quit & hold position")
        print(f"Step: {step}")
        self._print_position()

        while True:
            if held(VK["q"]) or held(VK["esc"]) or held(VK["x"]):
                break

            h_now = held(VK["h"])
            if h_now and not prev_h:
                print("Going home...")
                self.home()
                h = 0.0
                v = 0.0
                self._print_position()
            prev_h = h_now

            m_now = held(VK["m"])
            if m_now and not prev_m:
                if self._vna is not None:
                    self._print_measurement(self.measure())
                else:
                    print("No VNA connected -- call connect_vna() first")
            prev_m = m_now

            a_now = held(VK["a"])
            if a_now and not prev_a:
                step = max(0.18, step / 2)
                print(f"Step: {step:.4f}")
            prev_a = a_now

            s_now = held(VK["s"])
            if s_now and not prev_s:
                step = min(180.0, step * 2)
                print(f"Step: {step:.4f}")
            prev_s = s_now

            dh = 0.0
            dv = 0.0
            if held(VK["left"])  or held(VK["j"]): dh -= step
            if held(VK["right"]) or held(VK["l"]): dh += step
            if held(VK["up"])    or held(VK["i"]): dv += step
            if held(VK["down"])  or held(VK["k"]): dv -= step

            if dh != 0.0 or dv != 0.0:
                h = max(-180.0, min(180.0, h + dh))
                v = max(-180.0, min(180.0, v + dv))
                mbx.move_angle(hang=h, vang=v, accuracy="HIGH")
                self._print_position()
            else:
                time.sleep(0.02)

        self._print_position()
        print("Holding position — motors remain engaged.")

    def run_track(self, dither=0.5, kp=0.6, ki=0.04, kd=0.08,
                  output_limit=2.0, interval=0.2):
        """Continuously track the beam peak after initial alignment.

        Each cycle probes ±dither degrees on each axis, estimates the gradient, and
        applies a PID correction to hold the gimbal at the power maximum. Corrections
        are bounded to ±output_limit degrees per cycle so the motion stays smooth with
        no sudden jumps. The existing motor velocity and acceleration caps (_ACCEL_SCALE)
        further ensure gentle ramps between probe points.

        dither:       half-amplitude of the probe step (degrees, default 0.5)
        output_limit: maximum single-cycle angular correction (degrees, default 2.0)
        interval:     seconds of sleep after each correction cycle (default 0.2)
        kp/ki/kd:     PID gains — output in degrees, error in dB/degree

        Press Q / ESC / X to stop tracking and hold the current position.
        """
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        _k = ctypes.windll.user32.GetAsyncKeyState
        def held(vk): return bool(_k(vk) & 0x8000)
        QUIT = [0x51, 0x1B, 0x58]  # Q, ESC, X

        pid_h = _PIDAxis(kp, ki, kd, output_limit)
        pid_v = _PIDAxis(kp, ki, kd, output_limit)

        try:
            pos = self.position()
            h, v = pos["H"], pos["V"]
        except Exception:
            h, v = 0.0, 0.0

        lo_h, hi_h = _angle_limits(H)
        lo_v, hi_v = _angle_limits(V)

        print(f"\nPID tracker active  dither={dither}°  max_correction={output_limit}°  interval={interval}s")
        print("Q / ESC / X : stop tracking and hold position")

        tick = 0
        while True:
            if any(held(vk) for vk in QUIT):
                break

            # H axis: probe at h±dither (V stays fixed), compute gradient, apply PID
            h_lo = _clamp(h - dither, lo_h, hi_h)
            h_hi = _clamp(h + dither, lo_h, hi_h)
            span_h = h_hi - h_lo

            mbx.move_angle(hang=h_lo, vang=v, accuracy="HIGH")
            p_hm = self._align_measure()
            mbx.move_angle(hang=h_hi, vang=v, accuracy="HIGH")
            p_hp = self._align_measure()

            if p_hm is not None and p_hp is not None and span_h > 0:
                h = _clamp(h + pid_h.step((p_hp - p_hm) / span_h), lo_h, hi_h)
            else:
                pid_h.reset()

            # V axis: probe at v±dither (H now at corrected value), apply PID
            v_lo = _clamp(v - dither, lo_v, hi_v)
            v_hi = _clamp(v + dither, lo_v, hi_v)
            span_v = v_hi - v_lo

            mbx.move_angle(hang=h, vang=v_lo, accuracy="HIGH")
            p_vm = self._align_measure()
            mbx.move_angle(hang=h, vang=v_hi, accuracy="HIGH")
            p_vp = self._align_measure()

            if p_vm is not None and p_vp is not None and span_v > 0:
                v = _clamp(v + pid_v.step((p_vp - p_vm) / span_v), lo_v, hi_v)
            else:
                pid_v.reset()

            # move to PID-corrected position
            mbx.move_angle(hang=h, vang=v, accuracy="HIGH")

            tick += 1
            try:
                pos = self.position()
                p = self._align_measure()
                p_str = f"{p:.2f} dB" if p is not None else "---"
                print(f"  [{tick:4d}]  H={pos['H']:+.3f}°  V={pos['V']:+.3f}°  P={p_str}")
            except Exception:
                pass

            time.sleep(interval)

        self._print_position()
        print("Tracker stopped — holding position.")

    def _print_position(self):
        try:
            h_steps = mbx.current_pos(mbx.H, 1)
            v_steps = mbx.current_pos(mbx.V, 1)
            h_angle = mbx.convertpostoangle(mbx.H, h_steps)
            v_angle = mbx.convertpostoangle(mbx.V, v_steps)
            print(f"Position  H: {h_angle:.2f}  V: {v_angle:.2f}")
        except Exception as e:
            print(f"Position read failed: {e}")

    def position(self):
        self._require_connection()
        h = mbx.convertpostoangle(mbx.H, mbx.current_pos(mbx.H, 1))
        v = mbx.convertpostoangle(mbx.V, mbx.current_pos(mbx.V, 1))
        return {"H": h, "V": v}

    def run_cross_sweep(self, probe=20.0, accuracy="VERY HIGH"):
        """Sweep H ±probe° then V ±probe° (40° total each) from the current pointing position.

        Each axis is swept continuously while the VNA samples in flight; the motor moves
        to the measured peak on each axis before the next sweep begins.
        Parks at the located peak, plots the power log, then enters keyboard control.

        Returns (h_peak, v_peak) or (None, None) on error.
        """
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        print(f"\nStarting cross-sweep  (H ±{probe:.1f}°  then  V ±{probe:.1f}°)...")

        _log = []
        _freqs = []

        def _rec_meas():
            data = self.measure()
            db = data.get("db") or []
            if not _freqs:
                _freqs.extend(data.get("freqs") or [])
            if not db:
                _log.append((None, None, []))
                return None
            p = float(db[len(db) // 2])
            try:
                pos = self.position()
                _log.append((pos["H"], pos["V"], list(db)))
            except Exception:
                _log.append((None, None, list(db)))
            return p

        h0 = _read_angle(H)
        v0 = _read_angle(V)

        def move_h(ang):
            mbx.move_angle(hang=ang, accuracy="HIGH")

        def move_v(ang):
            mbx.move_angle(vang=ang, accuracy="HIGH")

        # ---- H sweep ----
        print(f"  H sweep: {h0 - probe:.1f}° → {h0 + probe:.1f}°")
        h_target, _, _ = _sweep_axis_continuous(_rec_meas, move_h, H, h0, probe, probe * 2)
        move_h(h_target)
        p_h = _rec_meas()
        print(f"  H peak: {h_target:.3f}°  ({_fmt(p_h)} dB)")

        # ---- V sweep ----
        print(f"  V sweep: {v0 - probe:.1f}° → {v0 + probe:.1f}°")
        v_target, _, _ = _sweep_axis_continuous(_rec_meas, move_v, V, v0, probe, probe * 2)
        move_v(v_target)
        p_v = _rec_meas()
        print(f"  V peak: {v_target:.3f}°  ({_fmt(p_v)} dB)")

        if p_h is None and p_v is None:
            print("*** ERROR: no valid VNA data — cross-sweep aborted")
            self.home()
            return None, None

        # final lock-in at full accuracy
        mbx.move_angle(hang=h_target, accuracy=accuracy)
        mbx.move_angle(vang=v_target, accuracy=accuracy)
        print(f"\nParked at  H={h_target:.3f}°  V={v_target:.3f}°")
        self._print_position()
        if self._vna is not None:
            self._print_measurement(self.measure())

        if _log:
            self._plot_alignment(_log, _freqs)

        print("\nEntering keyboard control -- Q/ESC/X: quit & hold position  |  H: go home")
        self.run_keyboard_control(start_h=h_target, start_v=v_target)
        return h_target, v_target

    # ==================================================================
    #  RADIATION PATTERN SWEEP  (modes 8 / 9)
    # ==================================================================

    def run_radiation_sweep(self, probe=40.0, accuracy="VERY HIGH"):
        """Home, then sweep H ±probe° then V ±probe°, recording a full radiation pattern.

        Uses the same motor speeds as run_cross_sweep. Plots the standard alignment
        graphs (from _plot_alignment) plus additional radiation-pattern figures.

        Returns (h_peak, v_peak) or (None, None) on error.
        """
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        print(f"\nStarting radiation-pattern sweep  (H ±{probe:.0f}°  then  V ±{probe:.0f}°)...")

        _log   = []   # (H, V, db_list) → _plot_alignment
        _freqs = []
        _h_pat = []   # (H_angle, db_list) collected during H sweep
        _v_pat = []   # (V_angle, db_list) collected during V sweep

        def _make_rec(pat_list, axis_motor):
            def _rec():
                data = self.measure()
                db = data.get("db") or []
                if not _freqs:
                    _freqs.extend(data.get("freqs") or [])
                if not db:
                    _log.append((None, None, []))
                    return None
                p = float(db[len(db) // 2])
                # Read the sweep axis directly — does not depend on self.position() succeeding.
                try:
                    angle = mbx.convertpostoangle(axis_motor, mbx.current_pos(axis_motor, 1))
                    pat_list.append((angle, list(db)))
                except Exception:
                    pass
                try:
                    pos = self.position()
                    _log.append((pos["H"], pos["V"], list(db)))
                except Exception:
                    _log.append((None, None, list(db)))
                return p
            return _rec

        print("  Homing...")
        self.home()

        def move_h(ang): mbx.move_angle(hang=ang, accuracy="HIGH")
        def move_v(ang): mbx.move_angle(vang=ang, accuracy="HIGH")

        # ---- H sweep (V held at 0°) ----
        h_rec = _make_rec(_h_pat, H)
        print(f"  H sweep: {-probe:.0f}° → +{probe:.0f}°")
        h_target, _, n_h = _sweep_axis_continuous(h_rec, move_h, H, 0.0, probe, probe * 2)
        move_h(h_target)
        h_rec()
        print(f"  H peak: {h_target:.3f}°  ({n_h} samples)")

        # ---- V sweep (H held at h_target) ----
        v_rec = _make_rec(_v_pat, V)
        print(f"  V sweep: {-probe:.0f}° → +{probe:.0f}°")
        v_target, _, n_v = _sweep_axis_continuous(v_rec, move_v, V, 0.0, probe, probe * 2)
        move_v(v_target)
        v_rec()
        print(f"  V peak: {v_target:.3f}°  ({n_v} samples)")

        if not _log or (h_target is None and v_target is None):
            print("*** ERROR: no valid VNA data — radiation sweep aborted")
            self.home()
            return None, None

        mbx.move_angle(hang=h_target, accuracy=accuracy)
        mbx.move_angle(vang=v_target, accuracy=accuracy)
        print(f"\nParked at  H={h_target:.3f}°  V={v_target:.3f}°")
        self._print_position()
        if self._vna is not None:
            self._print_measurement(self.measure())

        self._plot_radiation_sweep_results(_log, _freqs, _h_pat, _v_pat, h_target, v_target)

        print("\nEntering keyboard control -- Q/ESC/X: quit & hold position  |  H: go home")
        self.run_keyboard_control(start_h=h_target, start_v=v_target)
        return h_target, v_target

    def _plot_radiation_sweep_results(self, log, freqs, h_pat, v_pat, h_peak, v_peak):
        """Combined single-window output for run_radiation_sweep.

        Top row: alignment data (power trace, gimbal path, peak spectrum, per-freq angles).
        Bottom row: radiation pattern (H 1-D, V 1-D, H heatmap, V heatmap).
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed — skipping plot")
            return
        try:
            import numpy as np
            _have_numpy = True
        except ImportError:
            _have_numpy = False

        valid = [(h, v, db) for h, v, db in log if db]
        if not valid and not h_pat and not v_pat:
            print("No measurement data to plot.")
            return

        fig, axes = plt.subplots(2, 4, figsize=(22, 9))
        fig.suptitle("Radiation-Pattern Sweep Results", fontweight="bold")
        ax_pc, ax_gp, ax_sp, ax_pf = axes[0]   # top row: alignment
        ax_h1, ax_v1, ax_hm, ax_vm = axes[1]   # bottom row: radiation pattern

        n_freqs = len(valid[0][2]) if valid else (len(h_pat[0][1]) if h_pat else 0)
        freq_x = ([f / 1e9 for f in freqs] if freqs and len(freqs) == n_freqs
                  else list(range(n_freqs)))
        freq_label = "Frequency (GHz)" if freqs and len(freqs) == n_freqs else "Freq index"
        mid_i = n_freqs // 2

        # ---- top row: alignment plots ----
        if valid:
            hs     = [h  for h, _, _  in valid]
            vs     = [v  for _, v, _  in valid]
            dbs    = [db for _, _, db in valid]
            powers = [db[mid_i] for db in dbs]
            idxs   = list(range(len(valid)))
            peak_i = powers.index(max(powers))

            ax_pc.plot(idxs, powers, "b-o", markersize=4, linewidth=1.2)
            ax_pc.axvline(peak_i, color="red", linestyle="--",
                          label=f"Peak  {max(powers):.2f} dB")
            ax_pc.set_xlabel("Measurement index")
            ax_pc.set_ylabel("Power (dB)")
            ax_pc.set_title("Power vs Measurement")
            ax_pc.legend(fontsize=8)
            ax_pc.grid(True, alpha=0.4)

            has_pos = any(h is not None for h in hs)
            if has_pos:
                sc = ax_gp.scatter(hs, vs, c=powers, cmap="hot", s=50, zorder=3,
                                   vmin=min(powers), vmax=max(powers))
                plt.colorbar(sc, ax=ax_gp, label="Power (dB)")
                ax_gp.plot(hs[0],      vs[0],      "gs", markersize=9, zorder=4, label="Start")
                ax_gp.plot(hs[-1],     vs[-1],     "b^", markersize=9, zorder=4, label="Parked")
                ax_gp.plot(hs[peak_i], vs[peak_i], "r*", markersize=12, zorder=5,
                           label=f"Peak  {max(powers):.2f} dB")
                ax_gp.set_xlabel("H angle (deg)")
                ax_gp.set_ylabel("V angle (deg)")
                ax_gp.set_title("Gimbal Path (colour = power)")
                ax_gp.legend(fontsize=8)
                ax_gp.grid(True, alpha=0.4)

            ax_sp.plot(freq_x, dbs[peak_i], "g-o", markersize=4, linewidth=1.4)
            ax_sp.set_xlabel(freq_label)
            ax_sp.set_ylabel("Power (dB)")
            peak_pos = (f"H={hs[peak_i]:.2f}°  V={vs[peak_i]:.2f}°"
                        if has_pos and hs[peak_i] is not None else "peak")
            ax_sp.set_title(f"Spectrum at Peak  ({peak_pos})")
            ax_sp.grid(True, alpha=0.4)

            pos_valid = [(h, v, db) for h, v, db in valid if h is not None and v is not None]
            if has_pos and pos_valid and n_freqs > 0:
                bh = [max(pos_valid, key=lambda x, fi=fi: x[2][fi] if fi < len(x[2]) else -1e9)[0]
                      for fi in range(n_freqs)]
                bv = [max(pos_valid, key=lambda x, fi=fi: x[2][fi] if fi < len(x[2]) else -1e9)[1]
                      for fi in range(n_freqs)]
                ax_pf.plot(freq_x, bh, "b-o", markersize=4, linewidth=1.2, label="H")
                ax_pf.plot(freq_x, bv, "r-o", markersize=4, linewidth=1.2, label="V")
                ax_pf.set_xlabel(freq_label)
                ax_pf.set_ylabel("Angle (deg)")
                ax_pf.set_title("Per-Frequency Best Angle")
                ax_pf.legend(fontsize=8)
                ax_pf.grid(True, alpha=0.4)

        # ---- bottom row: radiation pattern plots ----
        if h_pat:
            h_ang = [a for a, _ in h_pat]
            h_pw  = [db[mid_i] for _, db in h_pat]
            ax_h1.plot(h_ang, h_pw, "b-o", markersize=4, linewidth=1.2)
            ax_h1.axvline(h_peak, color="red", linestyle="--",
                          label=f"Peak  {h_peak:.2f}°  ({max(h_pw):.2f} dB)")
            ax_h1.set_xlabel("H angle (deg)")
            ax_h1.set_ylabel("Power (dB)")
            ax_h1.set_title("H Radiation Pattern (mid-band)")
            ax_h1.legend(fontsize=8)
            ax_h1.grid(True, alpha=0.4)

        if v_pat:
            v_ang = [a for a, _ in v_pat]
            v_pw  = [db[mid_i] for _, db in v_pat]
            ax_v1.plot(v_ang, v_pw, "r-o", markersize=4, linewidth=1.2)
            ax_v1.axvline(v_peak, color="red", linestyle="--",
                          label=f"Peak  {v_peak:.2f}°  ({max(v_pw):.2f} dB)")
            ax_v1.set_xlabel("V angle (deg)")
            ax_v1.set_ylabel("Power (dB)")
            ax_v1.set_title("V Radiation Pattern (mid-band)")
            ax_v1.legend(fontsize=8)
            ax_v1.grid(True, alpha=0.4)

        if _have_numpy and h_pat and n_freqs > 1:
            h_ang = [a for a, _ in h_pat]
            h_mat = np.array([db for _, db in h_pat])
            im = ax_hm.imshow(h_mat.T, aspect="auto", origin="lower",
                              extent=[min(h_ang), max(h_ang), freq_x[0], freq_x[-1]],
                              cmap="hot")
            plt.colorbar(im, ax=ax_hm, label="Power (dB)")
            ax_hm.set_xlabel("H angle (deg)")
            ax_hm.set_ylabel(freq_label)
            ax_hm.set_title("H Pattern vs Frequency")

        if _have_numpy and v_pat and n_freqs > 1:
            v_ang = [a for a, _ in v_pat]
            v_mat = np.array([db for _, db in v_pat])
            im = ax_vm.imshow(v_mat.T, aspect="auto", origin="lower",
                              extent=[min(v_ang), max(v_ang), freq_x[0], freq_x[-1]],
                              cmap="hot")
            plt.colorbar(im, ax=ax_vm, label="Power (dB)")
            ax_vm.set_xlabel("V angle (deg)")
            ax_vm.set_ylabel(freq_label)
            ax_vm.set_title("V Pattern vs Frequency")

        plt.tight_layout()
        plt.show()

    def _plot_radiation_pattern(self, h_pat, v_pat, freqs, h_peak, v_peak):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed — skipping radiation pattern plot")
            return
        try:
            import numpy as np
            _have_numpy = True
        except ImportError:
            _have_numpy = False

        n_freqs = (len(h_pat[0][1]) if h_pat else len(v_pat[0][1]) if v_pat else 0)
        if n_freqs == 0:
            return

        freq_x = ([f / 1e9 for f in freqs] if freqs and len(freqs) == n_freqs
                  else list(range(n_freqs)))
        freq_label = "Frequency (GHz)" if freqs and len(freqs) == n_freqs else "Frequency index"
        mid_i = n_freqs // 2

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("Radiation Pattern — ±60° H and V Sweep", fontweight="bold")
        ax1, ax2, ax3, ax4 = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

        # ---- H radiation pattern (mid-band power vs H angle) ----
        if h_pat:
            h_ang = [a for a, _ in h_pat]
            h_pw  = [db[mid_i] for _, db in h_pat]
            ax1.plot(h_ang, h_pw, "b-o", markersize=4, linewidth=1.2)
            ax1.axvline(h_peak, color="red", linestyle="--",
                        label=f"Peak  {h_peak:.2f}°  ({max(h_pw):.2f} dB)")
            ax1.set_xlabel("H angle (deg)")
            ax1.set_ylabel("Power (dB)")
            ax1.set_title("H-Axis Radiation Pattern (mid-band)")
            ax1.legend(fontsize=8)
            ax1.grid(True, alpha=0.4)

        # ---- V radiation pattern (mid-band power vs V angle) ----
        if v_pat:
            v_ang = [a for a, _ in v_pat]
            v_pw  = [db[mid_i] for _, db in v_pat]
            ax2.plot(v_ang, v_pw, "r-o", markersize=4, linewidth=1.2)
            ax2.axvline(v_peak, color="red", linestyle="--",
                        label=f"Peak  {v_peak:.2f}°  ({max(v_pw):.2f} dB)")
            ax2.set_xlabel("V angle (deg)")
            ax2.set_ylabel("Power (dB)")
            ax2.set_title("V-Axis Radiation Pattern (mid-band)")
            ax2.legend(fontsize=8)
            ax2.grid(True, alpha=0.4)

        # ---- H pattern heatmap: power vs (H angle × frequency) ----
        if _have_numpy and h_pat and n_freqs > 1:
            h_ang   = [a for a, _ in h_pat]
            h_mat   = np.array([db for _, db in h_pat])   # (n_samples, n_freqs)
            im = ax3.imshow(h_mat.T, aspect="auto", origin="lower",
                            extent=[min(h_ang), max(h_ang), freq_x[0], freq_x[-1]],
                            cmap="hot")
            plt.colorbar(im, ax=ax3, label="Power (dB)")
            ax3.set_xlabel("H angle (deg)")
            ax3.set_ylabel(freq_label)
            ax3.set_title("H Pattern vs Frequency")

        # ---- V pattern heatmap: power vs (V angle × frequency) ----
        if _have_numpy and v_pat and n_freqs > 1:
            v_ang   = [a for a, _ in v_pat]
            v_mat   = np.array([db for _, db in v_pat])
            im = ax4.imshow(v_mat.T, aspect="auto", origin="lower",
                            extent=[min(v_ang), max(v_ang), freq_x[0], freq_x[-1]],
                            cmap="hot")
            plt.colorbar(im, ax=ax4, label="Power (dB)")
            ax4.set_xlabel("V angle (deg)")
            ax4.set_ylabel(freq_label)
            ax4.set_title("V Pattern vs Frequency")

        plt.tight_layout()
        plt.show()

    def close(self):
        if self._connected:
            try:
                mbx.enable_torque(H)
                mbx.enable_torque(V)
            finally:
                mbx.close()
                self._connected = False
        if self._vna is not None:
            try:
                self._vna.close_instrument()
            except Exception:
                pass
            self._vna = None

    def _on_exit(self):
        if self._connected:
            try:
                mbx.enable_torque(H)
                mbx.enable_torque(V)
            except Exception:
                pass
            try:
                mbx.close()
            except Exception:
                pass
            self._connected = False
        if self._vna is not None:
            try:
                self._vna.close_instrument()
            except Exception:
                pass
            self._vna = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


if __name__ == "__main__":
    while True:
        print("Select mode:")
        print("  1 - Manual control")
        print("  2 - Manual control with VNA (Anritsu MS46322B)")
        print("  3 - Autonomous VNA grid scan (Anritsu MS46322B)")
        print("  4 - Autonomous closed-loop DIRECT-MOTION alignment (Anritsu MS46322B)")
        print("  5 - Autonomous closed-loop DIRECT-MOTION alignment (SIMULATED VNA)")
        print("  6 - 40° cross-sweep: H then V (Anritsu MS46322B)")
        print("  7 - 40° cross-sweep: H then V (SIMULATED VNA)")
        print("  8 - 80° radiation-pattern sweep: home → H ±40° → V ±40° (Anritsu MS46322B)")
        print("  9 - 80° radiation-pattern sweep: home → H ±40° → V ±40° (SIMULATED VNA)")
        print(" 10 - PID alignment + continuous PID tracker (Anritsu MS46322B)")
        print(" 11 - PID alignment + continuous PID tracker (SIMULATED VNA)")
        choice = input("Enter 1-11: ").strip()
        if choice in ("1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"):
            break
        print("Invalid input -- enter 1-11.")

    with GimbalController(port="COM7") as gim:
        if choice == "2":
            gim.connect_vna("TCPIP0::192.168.6.150::9001::SOCKET")
            gim.run_keyboard_control()
        elif choice == "3":
            gim.connect_vna("TCPIP0::192.168.6.150::9001::SOCKET")
            gim.run_scan()
        elif choice == "4":
            gim.connect_vna("TCPIP0::192.168.6.150::9001::SOCKET")
            gim.run_direct_align()
        elif choice == "5":
            gim.connect_simulated_vna()
            gim.run_direct_align()
        elif choice == "6":
            gim.connect_vna("TCPIP0::192.168.6.150::9001::SOCKET")
            gim.run_cross_sweep()
        elif choice == "7":
            gim.connect_simulated_vna()
            gim.run_cross_sweep()
        elif choice == "8":
            gim.connect_vna("TCPIP0::192.168.6.150::9001::SOCKET")
            gim.run_radiation_sweep()
        elif choice == "9":
            gim.connect_simulated_vna()
            gim.run_radiation_sweep()
        elif choice == "10":
            gim.connect_vna("TCPIP0::192.168.6.150::9001::SOCKET")
            gim.run_direct_align(use_pid=True, enter_track=True, enter_keyboard=False)
        elif choice == "11":
            gim.connect_simulated_vna()
            gim.run_direct_align(use_pid=True, enter_track=True, enter_keyboard=False)
        else:
            gim.run_keyboard_control()
