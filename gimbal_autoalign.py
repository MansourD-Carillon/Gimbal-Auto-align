import sys
import atexit
import ctypes
import time

sys.path.insert(0, r"C:\Millibox Gimbal Files\SWMilliBox\MBX\python")
import mbx_functions as mbx
import mbx_instrument as equip

_BAUD = 1000000
_ANGLE_MIN = -62.0
_ANGLE_MAX = 62.0

# Convenience aliases for the motor / gimbal-type constants (these never change at runtime)
H = mbx.H
V = mbx.V
P = mbx.P
TH = mbx.TH
PH = mbx.PH
HV = mbx.HV
SPHERICAL = mbx.SPHERICAL


# ######################################################################################################################
# #  CLOSED-LOOP DIRECT-MOTION BEAM ALIGNMENT
# #  (reuses mbx.move_angle for motion and mbx.get_power / an injected measure_fn for VNA feedback)
# ######################################################################################################################

def measure_power_db(inst, freq_idx=None):
    """Single VNA/SA measurement -> scalar power in dB at one frequency.

    Wraps the existing mbx.get_power(), which returns (val, freq) lists across the VNA frequency list.
    By MBX convention the midpoint index is used as the representative point, exactly like the existing
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


# ======================================================================================================================
#  MAIN ROUTINE - HV GIMBAL (azimuth / elevation)
# ======================================================================================================================

def beam_align_hv_directmotion(inst, pangle=0.0, accuracy="VERY HIGH",
                               max_passes=5, init_probe=8.0, min_probe=0.5,
                               probe_decay=0.5, max_move=60.0,
                               tol_db=0.05, tol_ang=0.10,
                               start_h=None, start_v=None, verbose=True,
                               measure_fn=None):
    """Closed-loop HV gimbal alignment using direct-motion probes and parabola fits.
    
    Each pass: measure center, probe ±probe on H and V axes, fit parabola to each axis, move to vertex.
    Probe radius decays each pass (coarse-to-fine). Final move uses requested accuracy.
    
    Returns (H_off, V_off) or (None, None) on error.
    """
    if mbx.gim_type != HV:
        print("*** ERROR: gimbal is not HV - use beam_align_sph_directmotion() instead")
        return None, None

    t0 = time.time()

    # set polarization once, up front
    if mbx.num_motors >= 4:
        mbx.move_angle(pang=pangle, accuracy="HIGH")

    # starting working point = current pointing (or caller override)
    h0 = _read_angle(H) if start_h is None else float(start_h)
    v0 = _read_angle(V) if start_v is None else float(start_v)

    try:
        inst.fix_status()                                       # calibrate instrument if needed
    except Exception:
        pass

    # measurement source: injected callable (e.g. a controller's own VNA pipeline) or the MBX default
    def meas():
        if measure_fn is not None:
            return measure_fn()
        p, _ = measure_power_db(inst)
        return p

    # 1-axis direct-move closures (so _refine_axis stays axis-agnostic)
    def move_h(ang):
        mbx.move_angle(hang=ang, accuracy="HIGH")

    def move_v(ang):
        mbx.move_angle(vang=ang, accuracy="HIGH")

    # initial measurement at the working point
    move_h(_clamp_angle(H, h0))
    move_v(_clamp_angle(V, v0))
    p_cur = meas()
    best_h, best_v, best_p = h0, v0, (p_cur if p_cur is not None else -1e9)

    probe = float(init_probe)
    total_moves = 2

    if verbose:
        print("\n==== CLOSED-LOOP DIRECT-MOTION ALIGNMENT (HV) ====")
        print("start (H,V) = (%0.3f, %0.3f)   P = %s dB" % (h0, v0, _fmt(p_cur)))

    for k in range(max_passes):
        if _aborted():
            _safe_cont_trigger(inst)
            print("*** alignment aborted ***")
            return None, None

        if verbose:
            print("\n-- pass %d/%d  (probe radius = %0.2f deg) --" % (k + 1, max_passes, probe))

        # ---- Azimuth (H) : estimate uphill direction & new target, then move directly ----
        h_target, p_axis, nh = _refine_axis(meas, move_h, h0, p_cur, probe, max_move, H)
        move_h(h_target)
        p_after_h = meas()
        total_moves += nh + 1

        # ---- Elevation (V) : re-probe around the just-updated H, then move directly ----
        v_target, p_axis, nv = _refine_axis(meas, move_v, v0, p_after_h, probe, max_move, V)
        move_v(v_target)
        p_after_v = meas()
        total_moves += nv + 1

        dh = h_target - h0
        dv = v_target - v0
        new_p = p_after_v if p_after_v is not None else best_p

        if verbose:
            print("   -> target (H,V) = (%0.3f, %0.3f)   dH=%+0.3f dV=%+0.3f   P = %s dB"
                  % (h_target, v_target, dh, dv, _fmt(new_p)))

        # keep the best point actually visited
        if new_p > best_p:
            best_h, best_v, best_p = h_target, v_target, new_p

        # advance the working point
        improvement = new_p - p_cur if (p_cur is not None and new_p is not None) else 0.0
        h0, v0, p_cur = h_target, v_target, new_p

        # ---- convergence test : small geometric update AND small power change ----
        if max(abs(dh), abs(dv)) < tol_ang and abs(improvement) < tol_db:
            if verbose:
                print("   converged (update < %0.2f deg, dP < %0.2f dB)" % (tol_ang, tol_db))
            break

        # shrink the probe radius for the next, finer pass (coarse -> fine)
        probe = max(min_probe, probe * probe_decay)

    # ---- final lock-in : move to the best point with the requested (high) accuracy ----
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
              % (k + 1, total_moves + 2, (t1 - t0)))
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
                                measure_fn=None):
    """Closed-loop direct-motion alignment for a SPHERICAL gimbal (theta = TH, phi = PH).

    Same algorithm as beam_align_hv_directmotion(), mapped onto the theta/phi axes. The delta-phi (DPH)
    component of the PH motor is held at its current value; only PHI is optimized.
    """
    if mbx.gim_type != SPHERICAL:
        print("*** ERROR: gimbal is not SPHERICAL - use beam_align_hv_directmotion() instead")
        return None, None

    t0 = time.time()

    th0 = _read_angle(TH) if start_th is None else float(start_th)
    ph_pair = mbx.convertpostoangle(PH, mbx.current_pos(PH, 1))      # [phi, dphi]
    phi0 = ph_pair[0] if start_ph is None else float(start_ph)
    dphi = ph_pair[1]                                               # hold delta-phi fixed

    try:
        inst.fix_status()
    except Exception:
        pass

    def meas():
        if measure_fn is not None:
            return measure_fn()
        p, _ = measure_power_db(inst)
        return p

    def move_th(ang):
        mbx.move_angle(thang=ang, accuracy="HIGH")

    def move_ph(ang):
        mbx.move_angle(phang=[ang, dphi], accuracy="HIGH")

    move_th(_clamp_angle(TH, th0))
    move_ph(phi0)
    p_cur = meas()
    best_th, best_ph, best_p = th0, phi0, (p_cur if p_cur is not None else -1e9)

    probe = float(init_probe)
    total_moves = 2

    if verbose:
        print("\n==== CLOSED-LOOP DIRECT-MOTION ALIGNMENT (SPHERICAL) ====")
        print("start (TH,PHI) = (%0.3f, %0.3f)   P = %s dB" % (th0, phi0, _fmt(p_cur)))

    for k in range(max_passes):
        if _aborted():
            _safe_cont_trigger(inst)
            print("*** alignment aborted ***")
            return None, None

        if verbose:
            print("\n-- pass %d/%d  (probe radius = %0.2f deg) --" % (k + 1, max_passes, probe))

        th_target, _, nt = _refine_axis(meas, move_th, th0, p_cur, probe, max_move, TH)
        move_th(th_target)
        p_after_th = meas()
        total_moves += nt + 1

        # phi probes are clamped against the phi (PH -> T motor) software limits
        ph_target, _, np_ = _refine_axis(meas, move_ph, phi0, p_after_th, probe, max_move, PH)
        move_ph(ph_target)
        p_after_ph = meas()
        total_moves += np_ + 1

        dth = th_target - th0
        dph = ph_target - phi0
        new_p = p_after_ph if p_after_ph is not None else best_p

        if verbose:
            print("   -> target (TH,PHI) = (%0.3f, %0.3f)   dTH=%+0.3f dPHI=%+0.3f   P = %s dB"
                  % (th_target, ph_target, dth, dph, _fmt(new_p)))

        if new_p > best_p:
            best_th, best_ph, best_p = th_target, ph_target, new_p

        improvement = new_p - p_cur if (p_cur is not None and new_p is not None) else 0.0
        th0, phi0, p_cur = th_target, ph_target, new_p

        if max(abs(dth), abs(dph)) < tol_ang and abs(improvement) < tol_db:
            if verbose:
                print("   converged (update < %0.2f deg, dP < %0.2f dB)" % (tol_ang, tol_db))
            break

        probe = max(min_probe, probe * probe_decay)

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
              % (k + 1, total_moves + 2, (t1 - t0)))
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
# #  GIMBAL CONTROLLER
# ######################################################################################################################

class GimbalController:

    def __init__(self, port="COM8"):
        self._port = port
        self._connected = False
        self._vna = None
        if not mbx.connect(self._port, _BAUD):
            raise ConnectionError(
                f"Failed to connect to GIM04 on {self._port} at {_BAUD} bps"
            )
        self._connected = True
        mbx.set_gim_motion_default()
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
        mbx.gotoZERO(accuracy="HIGH")
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
                         freq_mode="mid", enter_keyboard=True):
        """Point the gimbal at the direction of maximum VNA signal using closed-loop
        direct motion (no full grid sweep, <= max_passes correction passes).

        All motion goes through mbx.move_angle over the COM link; feedback comes from
        this controller's measure() pipeline via the injected measure_fn. The gimbal is
        left parked at the located peak.

        Returns (H, V) of the peak, or None if it could not complete.
        """
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        print("\nStarting closed-loop DIRECT-MOTION alignment "
              f"(max {max_passes} passes, initial probe {init_probe}°)...")

        # wrap measure_fn to record (H, V, power) at every VNA sample
        _log = []
        def _recording_measure():
            p = self._align_measure(freq_mode)
            try:
                pos = self.position()
                _log.append((pos["H"], pos["V"], p))
            except Exception:
                _log.append((None, None, p))
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
            self._plot_alignment(_log)

        if enter_keyboard:
            print("\nParked at peak. Entering keyboard control -- Q/ESC to quit & home.")
            self.run_keyboard_control(start_h=a, start_v=b)
        return a, b

    def _plot_alignment(self, log):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed — skipping alignment plot")
            return

        valid = [(h, v, p) for h, v, p in log if p is not None]
        if not valid:
            print("No valid measurements to plot.")
            return

        hs     = [h for h, v, p in valid]
        vs     = [v for h, v, p in valid]
        powers = [p for h, v, p in valid]
        idxs   = list(range(len(valid)))
        peak_i = powers.index(max(powers))

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
        fig.suptitle("Direct-Motion Alignment — VNA Power Log", fontweight="bold")

        # power convergence trace
        ax1.plot(idxs, powers, "b-o", markersize=4, linewidth=1.2, label="Power")
        ax1.axvline(peak_i, color="red", linestyle="--",
                    label=f"Peak  {max(powers):.2f} dB  (meas #{peak_i})")
        ax1.set_xlabel("Measurement index")
        ax1.set_ylabel("Power (dB)")
        ax1.set_title("Power vs Measurement")
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.4)

        # gimbal path coloured by power
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
        print("\nParked at peak. Entering keyboard control -- Q/ESC to quit & home.")
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
        }

        step = 11.25
        h = start_h
        v = start_v
        prev_m = False
        prev_a = False
        prev_s = False

        print("Hold arrows/IJKL: continuous move  |  A/S: step  |  M: measure  |  Q/ESC: quit & home")
        print(f"Step: {step}")
        self._print_position()

        while True:
            if held(VK["q"]) or held(VK["esc"]):
                break

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

        self.home()

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

    def close(self):
        if self._connected:
            try:
                mbx.gotoZERO(accuracy="HIGH")
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
                mbx.gotoZERO(accuracy="HIGH")
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
        choice = input("Enter 1, 2, 3, or 4: ").strip()
        if choice in ("1", "2", "3", "4"):
            break
        print("Invalid input -- enter 1, 2, 3, or 4.")

    with GimbalController(port="COM8") as gim:
        if choice == "2":
            gim.connect_vna("TCPIP0::192.168.6.150::9001::SOCKET")
            gim.run_keyboard_control()
        elif choice == "3":
            gim.connect_vna("TCPIP0::192.168.6.150::9001::SOCKET")
            gim.run_scan()
        elif choice == "4":
            gim.connect_vna("TCPIP0::192.168.6.150::9001::SOCKET")
            gim.run_direct_align()
        else:
            gim.run_keyboard_control()
