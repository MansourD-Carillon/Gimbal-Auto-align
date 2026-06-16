import sys
import atexit
import ctypes
import time
import os
import csv
import datetime

import numpy as np

sys.path.insert(0, r"C:\Users\uconn\Downloads\MBX_Release_024_1\SWMilliBox\MBX\python")
import mbx_functions as mbx
import mbx_instrument as equip

_BAUD = 1000000

# V-axis absolute travel limits (degrees from motor mechanical zero)
V_LIMIT_LOWER = -60.0           # absolute lower bound for V axis
V_LIMIT_UPPER =  60.0           # absolute upper bound for V axis
# Inward margin on every continuous V sweep endpoint — prevents overshoot past the hard
# limit from motor inertia (move_pos) and the SDK overshoot-then-return in move_angle HIGH.
V_SWEEP_MARGIN_DEG = 3.0

# Adaptive sweep speed settings — tuned for a 1 kg antenna load
SCAN_SPEED_NEAR_DPS  = 1.0      # slow speed near the target angle
SCAN_SPEED_MID_DPS   = 3.0      # medium speed away from target
SCAN_SPEED_FAR_DPS   = 8.0      # max sweep speed
SCAN_SPEED_SCALE     = 1.0      # global aggressiveness multiplier
SCAN_NEAR_RADIUS_DEG = 5.0      # use slow speed within this many degrees of target
SCAN_FAST_RADIUS_DEG = 10.0     # use medium speed up to this many degrees

# ======================================================================================================================
#  VNA CONFIGURATION  --  flip ONE switch to go between the built-in simulator and a real VNA
# ======================================================================================================================
#   SIMULATE_VNA = True   ->  no VNA hardware needed. mbx.get_power() synthesizes the S21 response from the
#                             gimbal's live (H,V) angles: a beam peaked at boresight (0,0). The COM gimbal is
#                             still real and really moves, so this is a full end-to-end test of the alignment.
#   SIMULATE_VNA = False  ->  connect to a real VNA over the ethernet (raw TCPIP SOCKET) at VNA_ADDRESS.
SIMULATE_VNA = True
VNA_ADDRESS  = "TCPIP0::ANRITSU-VNA::inst0::INSTR"     # real VNA, used only when SIMULATE_VNA = False

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

# ======================================================================================================================
#  LOW-LEVEL HELPERS  (feedback, clamping, peak estimation)
# ======================================================================================================================

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


# ======================================================================================================================
#  PER-AXIS DIRECT-MOTION REFINEMENT
# ======================================================================================================================

def _refine_axis(meas, move_to, center_angle, center_power, probe, max_move, motor):
    """Estimate the peak along ONE axis and return the new target angle for that axis.

    This is the core "estimate the direction of increasing signal and compute a new target" step.

    Strategy (no scanning):
      * Probe the two neighbours at center +/- probe with DIRECT moves and measure each.
      * Fit a parabola through the three samples (minus, center, plus).
          - If it is concave (a real beam peak), the new target is the analytic vertex.
            A vertex is inherently overshoot-free, which is what suppresses oscillation around the peak.
          - If it is flat/convex/degenerate (noise, or we are off the main lobe), fall back to a gradient
            step: move `probe` degrees in whichever direction measured higher power (the uphill slope).
      * The candidate is clamped to +/- max_move from the current center and to the motor's software limits.

    `meas()` is a zero-arg callable returning the scalar power in dB (or None) at the current position.
    `move_to(angle)` is a small closure supplied by the caller that performs the actual 1-axis direct move
    (so this routine does not need to know whether the axis is H, V, theta or phi).

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
    """Closed-loop direct-motion alignment of the beam peak for an HV gimbal.

    Replaces the step-scan beam_align_hv(). Uses VNA feedback, direct moves only, no full sweep, and at
    most `max_passes` (<=5) correction passes.

    Each pass:
        1. Measure the current VNA response at the working point.
        2. Estimate the uphill direction on each axis with two direct-move probes (+/- probe) and a
           parabola fit  ->  this gives the local "direction of increasing signal strength".
        3. Compute the new target azimuth (H) and elevation (V) from the parabola vertex.
        4. Execute direct moves to those targets.
        5. Re-measure; the refined point becomes the start of the next pass.

    The probe radius shrinks geometrically each pass (coarse capture -> fine lock), which is what makes the
    motion converge quickly and zoom smoothly toward the peak instead of stepping through a grid.

    Optimization behaviour:
        * Final accuracy   : the last committed target is the analytic parabola vertex, and the final move
                             uses the requested `accuracy` (default "VERY HIGH", which overshoots+settles).
        * Fast convergence : a few large direct moves, early-exit once the update is below tol_ang / tol_db.
        * Minimal travel   : 2 probes per axis, the center sample is reused, probe radius decays each pass.
        * No oscillation   : targets are analytic maxima (overshoot-free) and the probe radius only shrinks.

    Returns (H_off, V_off) - the final azimuth/elevation - or (None, None) on error/abort.
    Note: probe moves use "HIGH" accuracy for speed; only the final lock-in uses `accuracy`.
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

    def __init__(self, port="COM7"):
        self._port = port
        self._connected = False
        self._vna = None
        self._simulated_vna = False
        if not mbx.connect(self._port, _BAUD):
            raise ConnectionError(
                f"Failed to connect to GIM04 on {self._port} at {_BAUD} bps"
            )
        self._connected = True
        mbx.set_gim_motion_default()

        # Fixed absolute V limits — independent of power-on position
        self._v_abs_min = V_LIMIT_LOWER
        self._v_abs_max = V_LIMIT_UPPER
        # Sweep endpoints are inset by the margin to absorb motor overshoot
        self._v_sweep_min = V_LIMIT_LOWER + V_SWEEP_MARGIN_DEG
        self._v_sweep_max = V_LIMIT_UPPER - V_SWEEP_MARGIN_DEG
        # Propagate into the MBX motion table so mbx.move_angle() also enforces the limits
        mbx.get_gim_motion()[2]["anglelim"] = [self._v_abs_min, self._v_abs_max]

        # H=16 keeps azimuth snappy; V=4 gives a long ramp to reduce jerk under 1 kg load
        try:
            mbx.set_accel(24, 1.5, 10)
        except Exception:
            pass

        atexit.register(self._on_exit)
        print(f"V-axis limits (absolute): [{self._v_abs_min:.1f}°, {self._v_abs_max:.1f}°]")

        # Return to mechanical zero on startup
        print("Homing to (H=0°, V=0°, P=0°) on startup...")
        try:
            mbx.gotoZERO(accuracy="HIGH")
        except Exception as e:
            print(f"Startup home failed: {e}")
        self._print_position()

    def _require_connection(self):
        if not self._connected:
            raise RuntimeError("Positioner is not connected")

    def _guard_v(self, angle):
        """Proactive V-limit guard: clamp target to the absolute safe window and warn.

        This is the primary enforcement layer — call before every V move so the limit
        is never physically reached during normal operation.
        """
        if angle < self._v_abs_min:
            print(f"*** V-limit: {angle:.3f}° is below floor {self._v_abs_min:.1f}° — clamped")
            return self._v_abs_min
        if angle > self._v_abs_max:
            print(f"*** V-limit: {angle:.3f}° exceeds ceiling {self._v_abs_max:.1f}° — clamped")
            return self._v_abs_max
        return angle

    def _assert_v_in_bounds(self):
        """Reactive V-limit fallback: stop V motion and raise if the axis is outside limits.

        Call after any move command as a safety net in case the proactive guard was bypassed.
        """
        v_now = mbx.convertpostoangle(mbx.V, mbx.current_pos(mbx.V, 1))
        if v_now < self._v_abs_min or v_now > self._v_abs_max:
            try:
                # Cancel any ongoing V move by commanding current position
                mbx.move_pos(mbx.V, mbx.current_pos(mbx.V, 1))
            except Exception:
                pass
            which = "lower" if v_now < self._v_abs_min else "upper"
            limit = self._v_abs_min if v_now < self._v_abs_min else self._v_abs_max
            raise RuntimeError(
                f"V-axis {which} limit breached: current={v_now:.3f}°, limit={limit:.1f}°"
            )

    def _validate_angle(self, axis, angle):
        pass  # superseded by _guard_v() for V; H is unrestricted

    def move(self, h=None, v=None):
        self._require_connection()
        if h is None and v is None:
            raise ValueError("At least one of h or v must be provided")
        if v is not None:
            v = self._guard_v(v)                       # proactive clamp
        ok = mbx.move_angle(hang=h, vang=v, accuracy="HIGH")
        self._print_position()
        self._assert_v_in_bounds()                     # reactive fallback
        if not ok:
            raise RuntimeError(f"move_angle failed — H={h}, V={v}")

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
        if v is not None:
            v = self._guard_v(v)                       # proactive clamp
        ok = mbx.move_angle(hang=h, vang=v, accuracy=accuracy)
        self._print_position()
        self._assert_v_in_bounds()                     # reactive fallback
        if not ok:
            raise RuntimeError(f"move_angle failed — H={h}, V={v}")
        return ok

    def connect_vna(self, visa_addr=None, simulate=False):
        """Connect a VNA, or use the built-in simulator.

        simulate=True (or visa_addr=None): create an unconnected VNA driver. With its port closed,
        mbx.get_power() synthesizes the S21 magnitude from the live gimbal (H,V) angles -> a beam
        peaked at boresight (0,0). No VNA hardware is required.

        Otherwise: open the real VNA at visa_addr (e.g. a raw TCPIP SOCKET over ethernet).
        """
        if simulate or visa_addr is None:
            self._vna = equip.VNA_Generic()        # a VNA-typed driver, port stays closed (port_open = 0)
            self._vna.inst_type = "VNA"
            self._simulated_vna = True
            print("VNA: using BUILT-IN SIMULATION (no hardware). "
                  "Synthetic beam peaks at boresight H=0, V=0.")
            return

        self._vna = equip.inst_setup_single("VNA", visa_addr)
        if not self._vna.port_open:
            self._vna = None
            raise ConnectionError(f"VNA not reachable at {visa_addr}")
        self._vna.init_meas()
        self._simulated_vna = False
        print(f"VNA: connected (real) at {visa_addr}")

    def measure(self):
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")
        if getattr(self._vna, "port_open", 0):
            # real instrument
            self._vna.single_trigger()
            freqs = self._vna.get_freq_list()
            db, phase = self._vna.get_s_dbphase()
            self._vna.cont_trigger()
            return {"freqs": freqs, "db": db, "phase": phase}
        # simulated VNA: synthesize magnitude vs gimbal angle via mbx.get_power()
        val, freq = mbx.get_power(self._vna)
        return {"freqs": freq, "db": val, "phase": [0.0] * len(val)}

    def _print_measurement(self, data):
        pos = self.position()
        print(f"--- Measurement  H: {pos['H']:.2f}°  V: {pos['V']:.2f}° ---")
        for f, d, p in zip(data["freqs"], data["db"], data["phase"]):
            print(f"  {f/1e9:.4f} GHz  {d:.2f} dB  {p:.2f}°")

    def home(self):
        self._require_connection()
        if not (self._v_abs_min <= 0.0 <= self._v_abs_max):
            raise RuntimeError(
                f"home() blocked: absolute 0° is outside V limits "
                f"[{self._v_abs_min:.1f}°, {self._v_abs_max:.1f}°]. "
                f"Homing would breach the V limit — move manually instead."
            )
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
                         freq_mode="mid", enter_keyboard=True,
                         start_h=None, start_v=None):
        """Point the gimbal at the direction of maximum VNA signal using closed-loop
        direct motion (no full grid sweep, <= max_passes correction passes).

        All motion goes through mbx.move_angle over the COM link; feedback comes from
        this controller's measure() pipeline (real VNA or built-in simulator) via the
        injected measure_fn. The gimbal is left parked at the located peak.

        start_h/start_v: if given, the gimbal is first driven there (a convenient way to
        start off-boresight when testing against the simulator, whose beam peaks at 0,0).

        Returns (H, V) of the peak, or None if it could not complete.
        """
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        if start_h is not None or start_v is not None:
            print(f"Pre-positioning to H={start_h}, V={start_v} before alignment...")
            self.goto(h=start_h, v=start_v, accuracy="HIGH")

        if self._simulated_vna:
            print("[SIMULATED VNA] beam peaks at boresight (H=0, V=0); "
                  "start off-center to watch it converge.")

        print("\nStarting closed-loop DIRECT-MOTION alignment "
              f"(max {max_passes} passes, initial probe {init_probe}°)...")

        result = beam_align_directmotion(
            self._vna,                                  # used for fix_status / cont_trigger only
            measure_fn=lambda: self._align_measure(freq_mode),   # feedback via our VNA pipeline (real or sim)
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
            print("Gimbal parked at current position — no automatic movement.")
            self._print_position()
            return None

        # the alignment already performed the final direct move to the peak; report + verify
        print(f"\nPeak located and parked:  axis1={a:.3f}°  axis2={b:.3f}°")
        self._print_position()
        if self._vna is not None:
            self._print_measurement(self.measure())

        if enter_keyboard:
            print("\nParked at peak. Entering keyboard control -- Q/ESC to quit & home.")
            self.run_keyboard_control(start_h=a, start_v=b)
        return a, b

    def _grid_scan(self, h_lo, h_hi, h_step, v_lo, v_hi, v_step, label, target_h=None):
        # Clip V range to session limits
        v_lo = max(v_lo, self._v_sweep_min)
        v_hi = min(v_hi, self._v_sweep_max)
        if v_lo >= v_hi:
            print(f"*** {label}: V range is entirely outside limits "
                  f"[{self._v_abs_min:.1f}°, {self._v_abs_max:.1f}°] — scan skipped")
            return 0.0, 0.0, float("-inf")

        # Build H column list
        h_angles = []
        h = h_lo
        while h <= h_hi + h_step * 0.01:
            h_angles.append(round(h, 4))
            h += h_step

        total_cols = len(h_angles)
        print(f"\n{label} scan: {total_cols} H cols × V [{v_lo:.1f}°→{v_hi:.1f}°]  "
              f"(~{v_step:.1f}° grid)  continuous fly-by zigzag")

        # One encoder step as the V-done tolerance
        done_tol_v = abs(mbx.convertangletopos(mbx.V, 1.0) - mbx.convertangletopos(mbx.V, 0.0))
        if done_tol_v < 1:
            done_tol_v = 1

        best_h, best_v, best_db = 0.0, 0.0, float("-inf")
        sweep_up = True         # zigzag: alternate V sweep direction each column

        for col_idx, h_col in enumerate(h_angles):
            v_start = v_lo if sweep_up else v_hi
            v_end   = v_hi if sweep_up else v_lo

            # V sweep velocity — linearly interpolated from NEAR at boresight to FAR at FAST_RADIUS
            if target_h is not None:
                t = min(abs(h_col - target_h) / SCAN_FAST_RADIUS_DEG, 1.0)
                v_vel = SCAN_SPEED_NEAR_DPS + t * (SCAN_SPEED_FAR_DPS - SCAN_SPEED_NEAR_DPS)
            else:
                v_vel = SCAN_SPEED_FAR_DPS

            print(f"\n{label} col {col_idx + 1}/{total_cols}  "
                  f"H={h_col:.2f}°  V: {v_start:.1f}°→{v_end:.1f}°  {v_vel:.1f} dps")

            # Blocking position: step H to column.
            # With zigzag V is already at v_start after the previous column's sweep,
            # so only H needs to move (except on the very first column).
            mbx.set_velocity(SCAN_SPEED_FAR_DPS, SCAN_SPEED_FAR_DPS, 0)
            if col_idx == 0:
                mbx.move_angle(hang=h_col, vang=self._guard_v(v_start), accuracy="HIGH")
            else:
                mbx.move_angle(hang=h_col, accuracy="HIGH")   # V already at v_start
            self._assert_v_in_bounds()

            # Non-blocking V fly-by — motor moves while VNA is sampled
            v_goal_pos = mbx.convertangletopos(mbx.V, v_end)
            mbx.set_velocity(0, v_vel, 0)   # H fixed, V at scan speed
            mbx.move_pos(mbx.V, v_goal_pos)

            while True:
                cur_v_pos = mbx.current_pos(mbx.V, 1)
                cur_v_ang = mbx.convertpostoangle(mbx.V, cur_v_pos)
                self._assert_v_in_bounds()              # reactive V limit fallback

                try:
                    data = self.measure()
                    db_vals = data.get("db", [])
                    peak = max(db_vals) if db_vals else float("-inf")
                except Exception:
                    peak = float("-inf")

                if peak > best_db:
                    best_db = peak
                    best_h = h_col
                    best_v = cur_v_ang                  # live encoder position at peak

                print(f"  H:{h_col:7.2f}°  V:{cur_v_ang:7.2f}°  {peak:7.2f} dB"
                      f"  (best H:{best_h:.2f}°  V:{best_v:.2f}°  {best_db:.2f} dB)")

                if abs(v_goal_pos - cur_v_pos) <= done_tol_v:
                    break

            mbx.set_velocity(0, 0, 0)
            sweep_up = not sweep_up     # reverse V direction for next column

        return best_h, best_v, best_db

    def run_scan(self, coarse_step=45.0, fine_step=11.25):
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        print("Starting coarse scan (45 deg steps)...")
        best_h, best_v, best_db = self._grid_scan(
            -180.0, 180.0, coarse_step,
            self._v_abs_min, self._v_abs_max, coarse_step,
            "Coarse"
        )
        print(f"\nCoarse peak: H={best_h:.2f}  V={best_v:.2f}  {best_db:.2f} dB")

        h_lo = max(-180.0, best_h - coarse_step)
        h_hi = min(180.0, best_h + coarse_step)
        v_lo = max(self._v_abs_min, best_v - coarse_step)
        v_hi = min(self._v_abs_max, best_v + coarse_step)

        print(f"\nStarting fine scan ({fine_step} deg steps around peak)...")
        best_h, best_v, best_db = self._grid_scan(
            h_lo, h_hi, fine_step,
            v_lo, v_hi, fine_step,
            "Fine"
        )

        print(f"\nPeak found from sweep data: H={best_h:.2f}  V={best_v:.2f}  {best_db:.2f} dB")
        if self._simulated_vna:
            err_h = best_h - 0.0
            err_v = best_v - 0.0
            print("Simulated beam center expected at H=0.00, V=0.00 deg")
            print("Position error vs simulated center: dH=%+.2f deg, dV=%+.2f deg" % (err_h, err_v))
        print("Moving the gimbal to that VNA peak direction...")
        mbx.move_angle(hang=best_h, vang=self._guard_v(best_v), accuracy="HIGH")
        self._assert_v_in_bounds()
        self._print_position()
        try:
            data = self.measure()
            peak_db = max(data["db"]) if data.get("db") else None
            print(f"Confirmed VNA peak power at this location: {peak_db:.2f} dB")
        except Exception as exc:
            print(f"Peak confirmation read failed: {exc}")
        print("\nParked at the strongest measured VNA direction. Entering keyboard control -- Q/ESC to quit.")
        self.run_keyboard_control(start_h=best_h, start_v=best_v)

    def run_adaptive_scan(self):
        """Continuous adaptive-speed H then V scan — speed reduces within ±10° of boresight (0°, 0°)."""
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        t0 = time.time()

        # Boresight is always (0°, 0°) — no user input needed
        target_h = 0.0
        target_v = 0.0

        def _speed_for_angle(h_angle):
            t = min(abs(h_angle - target_h) / SCAN_FAST_RADIUS_DEG, 1.0)
            return (SCAN_SPEED_NEAR_DPS + t * (SCAN_SPEED_FAR_DPS - SCAN_SPEED_NEAR_DPS)) * SCAN_SPEED_SCALE

        def _speed_for_v(v_angle):
            t = min(abs(v_angle - target_v) / SCAN_FAST_RADIUS_DEG, 1.0)
            return (SCAN_SPEED_NEAR_DPS + t * (SCAN_SPEED_FAR_DPS - SCAN_SPEED_NEAR_DPS)) * SCAN_SPEED_SCALE

        h_lo = self._v_abs_min   # -60°
        h_hi = self._v_abs_max   # +60°

        print(f"\nStarting continuous adaptive scan  H [{h_lo:.1f}°, {h_hi:.1f}°]  "
              f"V [{self._v_abs_min:.1f}°, {self._v_abs_max:.1f}°]")
        print("  within %.1f deg of boresight -> %.1f dps" % (SCAN_NEAR_RADIUS_DEG, SCAN_SPEED_NEAR_DPS))
        print("  within %.1f deg of boresight -> %.1f dps" % (SCAN_FAST_RADIUS_DEG, SCAN_SPEED_MID_DPS))
        print("  beyond %.1f deg of boresight -> %.1f dps" % (SCAN_FAST_RADIUS_DEG, SCAN_SPEED_FAR_DPS))

        try:
            self._vna.fix_status()
        except Exception:
            pass

        mbx.move_angle(vang=self._guard_v(0.0), hang=h_lo, accuracy="HIGH")
        time.sleep(0.1)

        best_h = h_lo
        best_v = 0.0
        best_db = float("-inf")
        last_vel = None

        # Continuous H sweep across the full range.
        h_goal_pos = mbx.convertangletopos(mbx.H, h_hi)
        mbx.set_velocity(_speed_for_angle(h_lo), 0, 0)
        mbx.move_pos(mbx.H, h_goal_pos)

        done_tol_pos = abs(mbx.convertangletopos(mbx.H, 1.0) - mbx.convertangletopos(mbx.H, 0.0))
        if done_tol_pos < 1:
            done_tol_pos = 1

        while True:
            cur_pos = mbx.current_pos(mbx.H, 1)
            cur_ang = mbx.convertpostoangle(mbx.H, cur_pos)
            try:
                data = self.measure()
                db_vals = data.get("db", [])
                peak = max(db_vals) if db_vals else float("-inf")
            except Exception:
                peak = float("-inf")

            if peak > best_db:
                best_db = peak
                best_h = cur_ang
                best_v = mbx.convertpostoangle(mbx.V, mbx.current_pos(mbx.V, 1))

            vel = _speed_for_angle(cur_ang)
            if last_vel != vel:
                mbx.set_velocity(vel, 0, 0)
                last_vel = vel

            if abs(h_goal_pos - cur_pos) <= done_tol_pos:
                break

            time.sleep(0.05)

        mbx.set_velocity(0, 0, 0)

        # Return to origin, then V sweep at H=0.
        print("\nReturning to (H=0°, V=0°) before V sweep...")
        mbx.move_angle(hang=0.0, vang=self._guard_v(0.0), accuracy="HIGH")
        time.sleep(0.1)

        print(f"\nPhase 2: V sweep [{self._v_sweep_min:.1f}°, {self._v_sweep_max:.1f}°] at H=0°")
        mbx.move_angle(hang=0.0, vang=self._guard_v(self._v_sweep_min), accuracy="HIGH")
        time.sleep(0.1)

        v_goal_pos = mbx.convertangletopos(mbx.V, self._v_sweep_max)
        mbx.set_velocity(0, _speed_for_v(self._v_sweep_min), 0)
        mbx.move_pos(mbx.V, v_goal_pos)

        done_tol_pos_v = abs(mbx.convertangletopos(mbx.V, 1.0) - mbx.convertangletopos(mbx.V, 0.0))
        if done_tol_pos_v < 1:
            done_tol_pos_v = 1

        last_vel_v = None
        while True:
            cur_v_pos = mbx.current_pos(mbx.V, 1)
            cur_v_ang = mbx.convertpostoangle(mbx.V, cur_v_pos)
            self._assert_v_in_bounds()                 # reactive fallback
            try:
                data = self.measure()
                db_vals = data.get("db", [])
                peak = max(db_vals) if db_vals else float("-inf")
            except Exception:
                peak = float("-inf")

            if peak > best_db:
                best_db = peak
                best_h = mbx.convertpostoangle(mbx.H, mbx.current_pos(mbx.H, 1))
                best_v = cur_v_ang

            vel_v = _speed_for_v(cur_v_ang)
            if last_vel_v != vel_v:
                mbx.set_velocity(0, vel_v, 0)
                last_vel_v = vel_v

            if abs(v_goal_pos - cur_v_pos) <= done_tol_pos_v:
                break

            time.sleep(0.05)

        mbx.set_velocity(0, 0, 0)
        mbx.move_angle(hang=best_h, vang=self._guard_v(best_v), accuracy="HIGH")
        self._assert_v_in_bounds()
        self._print_position()
        elapsed = time.time() - t0
        print(f"\nPeak found from continuous adaptive sweep: H={best_h:.2f}  V={best_v:.2f}  {best_db:.2f} dB")
        if self._simulated_vna:
            err_h = best_h - 0.0
            err_v = best_v - 0.0
            print("Simulated beam center expected at H=0.00, V=0.00 deg")
            print("Position error vs simulated center: dH=%+.2f deg, dV=%+.2f deg" % (err_h, err_v))
        print(f"Motion complete in {elapsed:.2f} seconds")
        print("Parked at the strongest measured VNA direction.")
        self.run_keyboard_control(start_h=best_h, start_v=best_v)

    def run_velocity_scan(self, kv=0.8, v_min=0.5, v_max=8.0,
                          poll_interval=0.05, tolerance=0.1, plot=True):
        """Proportional-velocity VNA scan + precision park at the measured peak.

        Three phases:
          1. H sweep  (−60° → +60°) with velocity = clamp(Kv × |H|, v_min, v_max).
             Slower near boresight (0°), faster far away.
          2. V sweep  (−60° → +60°) at the best H found, same proportional profile vs 0°.
          3. Precision park at the VNA peak using velocity-proportional approach then
             VERY HIGH accuracy fine-settle (move_with_pid).
        """
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        t0 = time.time()

        # Boresight is always (0°, 0°) — no user input needed
        target_h = 0.0
        target_v = 0.0

        h_lo = self._v_abs_min   # -60°
        h_hi = self._v_abs_max   # +60°

        def _vel(angle, hint):
            return _clamp(kv * abs(angle - hint), v_min, v_max)

        def _scan_aborted():
            k = ctypes.windll.user32.GetAsyncKeyState
            return bool(k(0x51) & 0x8000) or bool(k(0x1B) & 0x8000)  # Q or ESC

        print(f"\nPhase 1: H sweep [{h_lo:.1f}°, {h_hi:.1f}°]  "
              f"vel = clamp({kv}×|H|, {v_min}, {v_max}) dps — slower near boresight (0°)")

        try:
            self._vna.fix_status()
        except Exception:
            pass

        mbx.move_angle(vang=self._guard_v(0.0), hang=h_lo, accuracy="HIGH")
        time.sleep(0.1)

        best_h = h_lo
        best_v = 0.0
        best_db = float("-inf")
        h_scan_ang = []
        h_scan_db = []

        done_tol_h = abs(mbx.convertangletopos(mbx.H, 1.0) - mbx.convertangletopos(mbx.H, 0.0))
        if done_tol_h < 1:
            done_tol_h = 1

        h_goal_pos = mbx.convertangletopos(mbx.H, h_hi)
        mbx.set_velocity(_vel(h_lo, target_h), 0, 0)
        mbx.move_pos(mbx.H, h_goal_pos)

        while True:
            if _scan_aborted():
                print("\n*** Scan aborted (Q/ESC) ***")
                mbx.set_velocity(0, 0, 0)
                print("Gimbal parked at current position — no automatic movement.")
                self._print_position()
                return
            cur_h_pos = mbx.current_pos(mbx.H, 1)
            cur_h_ang = mbx.convertpostoangle(mbx.H, cur_h_pos)
            try:
                db_vals = self.measure().get("db", [])
                peak = max(db_vals) if db_vals else float("-inf")
            except Exception:
                peak = float("-inf")

            if peak > best_db:
                best_db = peak
                best_h = cur_h_ang
                best_v = mbx.convertpostoangle(mbx.V, mbx.current_pos(mbx.V, 1))

            h_scan_ang.append(cur_h_ang)
            h_scan_db.append(peak)

            # Read encoder position every tick and re-issue move_pos so the Dynamixel
            # applies the updated velocity profile immediately rather than keeping the
            # profile set at the start of the sweep.
            vel_h = _vel(cur_h_ang, target_h)
            mbx.set_velocity(vel_h, 0, 0)
            mbx.move_pos(mbx.H, h_goal_pos)

            print(f"  H={cur_h_ang:+8.3f}°  vel={vel_h:5.2f} dps  P={peak:.2f} dB"
                  f"  (best H={best_h:.2f}° @ {best_db:.2f} dB)")

            if abs(h_goal_pos - cur_h_pos) <= done_tol_h:
                break
            time.sleep(poll_interval)

        mbx.set_velocity(0, 0, 0)

        # ---- Return to origin between sweeps ----
        print("\nReturning to (H=0°, V=0°) before V sweep...")
        mbx.move_angle(hang=0.0, vang=self._guard_v(0.0), accuracy="HIGH")
        time.sleep(0.1)

        # ---- Phase 2: V sweep at H=0 ----
        print(f"\nPhase 2: V sweep [{self._v_sweep_min:.1f}°, {self._v_sweep_max:.1f}°] at H=0°  "
              f"vel = clamp({kv}×|V|, {v_min}, {v_max}) dps — slower near boresight (0°)")

        mbx.move_angle(hang=0.0, vang=self._guard_v(self._v_sweep_min), accuracy="HIGH")
        time.sleep(0.1)

        v_scan_ang = []
        v_scan_db = []

        done_tol_v = abs(mbx.convertangletopos(mbx.V, 1.0) - mbx.convertangletopos(mbx.V, 0.0))
        if done_tol_v < 1:
            done_tol_v = 1

        v_goal_pos = mbx.convertangletopos(mbx.V, self._v_sweep_max)
        mbx.set_velocity(0, _vel(self._v_sweep_min, target_v), 0)
        mbx.move_pos(mbx.V, v_goal_pos)

        while True:
            if _scan_aborted():
                print("\n*** Scan aborted (Q/ESC) ***")
                mbx.set_velocity(0, 0, 0)
                return
            cur_v_pos = mbx.current_pos(mbx.V, 1)
            cur_v_ang = mbx.convertpostoangle(mbx.V, cur_v_pos)
            self._assert_v_in_bounds()                 # reactive fallback
            try:
                db_vals = self.measure().get("db", [])
                peak = max(db_vals) if db_vals else float("-inf")
            except Exception:
                peak = float("-inf")

            if peak > best_db:
                best_db = peak
                best_h = mbx.convertpostoangle(mbx.H, mbx.current_pos(mbx.H, 1))
                best_v = cur_v_ang

            v_scan_ang.append(cur_v_ang)
            v_scan_db.append(peak)

            vel_v = _vel(cur_v_ang, target_v)
            mbx.set_velocity(0, vel_v, 0)
            mbx.move_pos(mbx.V, v_goal_pos)

            print(f"  V={cur_v_ang:+8.3f}°  vel={vel_v:5.2f} dps  P={peak:.2f} dB"
                  f"  (best V={best_v:.2f}° @ {best_db:.2f} dB)")

            if abs(v_goal_pos - cur_v_pos) <= done_tol_v:
                break
            time.sleep(poll_interval)

        mbx.set_velocity(0, 0, 0)

        print(f"\nScan complete. VNA peak: H={best_h:.3f}°  V={best_v:.3f}°  P={best_db:.2f} dB")
        if self._simulated_vna:
            print(f"Simulated beam center H=0.00° V=0.00°  →  "
                  f"dH={best_h:+.3f}°  dV={best_v:+.3f}°")

        # ---- Phase 3: precision park ----
        print(f"\nPhase 3: velocity-controlled park at VNA peak + VERY HIGH accuracy fine-settle...")
        park_log = self.move_with_pid(h=best_h, v=best_v, kv=kv,
                                       v_min=v_min, v_max=v_max,
                                       tolerance=tolerance, plot=False)

        print(f"\nTotal elapsed: {time.time() - t0:.2f}s")

        if plot:
            self._plot_scan_results(h_scan_ang, h_scan_db, v_scan_ang, v_scan_db,
                                     best_h, best_v, park_log)

        print("\nParked at VNA peak. Entering keyboard control — Q/ESC to quit.")
        self.run_keyboard_control(start_h=best_h, start_v=best_v)

    def run_continuous_sweep(self, minh=-180.0, maxh=180.0, step=1.0,
                             vert=0.0, h_speed_dps=5.0, settle=0.2,
                             plot_freq=None, tag="continuous", validonly=True,
                             plot=1):
        """Continuous H-sweep capture that writes raw VNA power vs angle to CSV.

        This integrates the provided continuous-sweep logic into the existing controller.
        It uses the current VNA source (real or simulated), analyzes the captured data
        to find the strongest point, and parks the gimbal there at the end of the sweep.
        """
        self._require_connection()
        if self._vna is None:
            raise RuntimeError("VNA not connected — call connect_vna() first")

        t0 = time.time()
        time_str = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())
        outdir = os.path.join("..", "..", "MilliBox_plot_data")
        os.makedirs(outdir, exist_ok=True)
        basename = os.path.join(outdir, f"mbx_capture_{time_str}_1d_Hcont_{tag}")
        filename = f"{basename}.csv"
        print(f"Continuous-sweep raw data saved in: {filename}")

        csvplot = open(filename, "w", buffering=1, newline="")
        capture = csv.writer(csvplot, lineterminator="\n")

        try:
            if validonly:
                try:
                    Hlim = mbx.get_gim_motion()[1]["anglelim"]
                    minh = max(minh, Hlim[0])
                    maxh = min(maxh, Hlim[1])
                except Exception:
                    pass
            if maxh <= minh:
                print("*** Empty H range after limit clipping; aborting ***")
                return None, None, filename

            nout = int(np.floor((maxh - minh) / step))
            Hgrid = np.linspace(minh, minh + nout * step, nout + 1)

            val, freq = mbx.get_power(self._vna)
            freq = np.asarray(freq, dtype=float)
            if plot_freq is None:
                plot_freq = freq[int(len(freq) / 2)]
            freq_idx = int(np.abs(freq - plot_freq).argmin())
            print(f"\n**** Plotting frequency = {freq[freq_idx] / 1.0e9:.3f} GHz ****\n")
            capture.writerow(["t", "actual_H", "actual_V"] + list(freq))

            mbx.move_angle(vang=self._guard_v(vert), hang=minh, accuracy="HIGH")
            try:
                self._vna.fix_status()
            except Exception:
                pass

            mbx.set_velocity(h_speed_dps, 0, 0)
            h_goal_pos = mbx.convertangletopos(mbx.H, maxh)
            mbx.move_pos(mbx.H, h_goal_pos)
            time.sleep(settle)

            samples_ang = []
            samples_val = []
            aborted = False

            done_tol_pos = abs(mbx.convertangletopos(mbx.H, step / 2.0) - mbx.convertangletopos(mbx.H, 0))
            if done_tol_pos < 1:
                done_tol_pos = 1

            while True:
                if mbx.kbhit() and mbx.check_abort():
                    aborted = True
                    break

                cur_pos = mbx.current_pos(mbx.H, 1)
                cur_ang = mbx.convertpostoangle(mbx.H, cur_pos)
                val, _ = mbx.get_power(self._vna)
                actual_v = mbx.convertpostoangle(mbx.V, mbx.current_pos(mbx.V, 1))
                tnow = time.time() - t0

                samples_ang.append(cur_ang)
                samples_val.append(val[freq_idx])
                capture.writerow([tnow, cur_ang, actual_v] + list(val))

                if plot == 1:
                    print("t=%6.2f  H=%+8.3f  V=%+8.3f  VAL=%0.2f" %
                          (tnow, cur_ang, actual_v, val[freq_idx]))

                if abs(h_goal_pos - cur_pos) <= done_tol_pos:
                    break

                time.sleep(0.05)

            try:
                self._vna.cont_trigger()
            except Exception:
                pass
            mbx.set_velocity(0, 0, 0)

            if aborted:
                print("*** Sweep aborted by user ***")
                return Hgrid, None, filename

            samples_ang = np.asarray(samples_ang)
            samples_val = np.asarray(samples_val)
            order = np.argsort(samples_ang)
            samples_ang = samples_ang[order]
            samples_val = samples_val[order]
            uniq = np.concatenate(([True], np.diff(samples_ang) > 0))
            samples_ang = samples_ang[uniq]
            samples_val = samples_val[uniq]

            if len(samples_ang) < 2:
                print("*** Too few samples captured; lower h_speed_dps ***")
                return Hgrid, None, filename

            heat_grid = np.interp(Hgrid, samples_ang, samples_val, left=np.nan, right=np.nan)
            valid = np.isfinite(heat_grid)
            if not np.any(valid):
                print("*** No valid sweep data found; nothing to align to ***")
                return Hgrid, heat_grid, filename

            peak_idx = int(np.nanargmax(heat_grid))
            best_h = float(Hgrid[peak_idx])
            best_db = float(heat_grid[peak_idx])
            span = samples_ang[-1] - samples_ang[0]
            print(f"\nCaptured {len(samples_ang)} points over {span:.1f} deg  ->  {len(samples_ang) / max(span, 1e-9):.2f} samples/deg")
            print(f"Peak from sweep: H={best_h:.3f} deg, V={vert:.3f} deg, P={best_db:.2f} dB")
            print("Moving gimbal to the sweep peak...")
            mbx.move_angle(hang=best_h, vang=self._guard_v(vert), accuracy="HIGH")
            self._print_position()
            print("Parked at the strongest point found from the captured data.")
            print(f"*** Elapsed = {datetime.timedelta(seconds=int(time.time() - t0))} ***")
            print(f"## RAW DATA SAVED: {filename} ##")
            return Hgrid, heat_grid, filename
        finally:
            csvplot.close()

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
        prev_m = False
        prev_a = False
        prev_s = False

        # Initialise from live encoder — not from the caller-supplied estimate,
        # which is a commanded position and may differ from settled reality.
        try:
            _pos = self.position()
            h = _pos["H"]
            v = _pos["V"]
        except Exception:
            h = float(start_h)
            v = float(start_v)

        print("Hold arrows/IJKL: continuous move  |  A/S: step  |  M: measure  |  Q/ESC: quit")
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
                # Read live encoder for each move so the delta is applied to the
                # actual settled position, not a software-tracked estimate.
                try:
                    _pos = self.position()
                    h = _pos["H"]
                    v = _pos["V"]
                except Exception:
                    pass                               # keep last known value on read error
                h = h + dh
                v = self._guard_v(v + dv)             # proactive clamp against session limits
                mbx.move_angle(hang=h, vang=v, accuracy="HIGH")
                self._assert_v_in_bounds()             # reactive fallback
                self._print_position()
            else:
                time.sleep(0.02)

        print("Stopped. Gimbal parked at current position.")
        self._print_position()

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

    # ------------------------------------------------------------------------------------------------------------------
    #  PID-CONTROLLED MOVE
    # ------------------------------------------------------------------------------------------------------------------

    def move_with_pid(self, h=None, v=None,
                      kv=1.6, v_min=1.0, v_max=8.0,
                      h_speed=15.0,
                      tolerance=0.1, max_time=30.0,
                      poll_interval=0.05, plot=True, verbose=True):
        """Move to (h, v), then fine-settle.

        H moves at a constant h_speed (dps) throughout — no proportional scaling.
        V uses proportional velocity: clamp(Kv × |error|, v_min, v_max) dps,
        slowing to a creep as it approaches the target.
        Once both axes are within *tolerance* degrees for 3 consecutive ticks the
        loop exits and mbx.move_angle(accuracy="VERY HIGH") eliminates residual
        backlash via the SDK's built-in overshoot-then-approach correction.

        verbose=False suppresses per-tick table output.
        Returns the log dict.
        """
        self._require_connection()
        if h is None and v is None:
            raise ValueError("At least one of h or v must be provided")
        if v is not None:
            v = self._guard_v(v)                       # proactive clamp

        pos = self.position()
        h_target = float(h) if h is not None else pos["H"]
        v_target = float(v) if v is not None else pos["V"]

        h_steps = mbx.convertangletopos(mbx.H, h_target)
        v_steps = mbx.convertangletopos(mbx.V, v_target)

        log = {
            "t": [], "h_actual": [], "v_actual": [],
            "h_error": [], "v_error": [],
            "h_target": h_target, "v_target": v_target,
        }

        SEP = "─" * 76
        if verbose:
            print(f"\n{SEP}")
            print(f"  VELOCITY-CONTROLLED MOVE  →  H={h_target:.3f}°  V={v_target:.3f}°")
            print(f"  H: {h_speed} dps constant  |  V: clamp({kv}×|err|, {v_min}, {v_max}) dps  tol=±{tolerance}°")
            print(SEP)
            print(f"{'t(s)':>7}  {'H actual':>10}  {'H error':>9}  {'vel_H':>7}"
                  f"  {'V actual':>10}  {'V error':>9}  {'vel_V':>7}")
            print(SEP)

        t0 = time.time()
        settled = 0

        while True:
            elapsed = time.time() - t0
            h_act = mbx.convertpostoangle(mbx.H, mbx.current_pos(mbx.H, 1))
            v_act = mbx.convertpostoangle(mbx.V, mbx.current_pos(mbx.V, 1))
            h_err = h_target - h_act
            v_err = v_target - v_act

            vel_h = h_speed
            vel_v = _clamp(kv * abs(v_err), v_min, v_max)

            log["t"].append(elapsed)
            log["h_actual"].append(h_act)
            log["v_actual"].append(v_act)
            log["h_error"].append(h_err)
            log["v_error"].append(v_err)

            if verbose:
                print(
                    f"{elapsed:7.2f}  {h_act:+10.3f}°  {h_err:+9.3f}°  {vel_h:7.2f}"
                    f"  {v_act:+10.3f}°  {v_err:+9.3f}°  {vel_v:7.2f}"
                )

            self._assert_v_in_bounds()                 # reactive fallback each tick

            if abs(h_err) <= tolerance and abs(v_err) <= tolerance:
                settled += 1
                if settled >= 3:
                    break
            else:
                settled = 0
                # Velocity proportional to error; re-issue position target at new speed
                mbx.set_velocity(vel_h, vel_v, 0)
                mbx.move_pos(mbx.H, h_steps)
                mbx.move_pos(mbx.V, v_steps)

            if elapsed >= max_time:
                if verbose:
                    print(f"\n  Timeout after {max_time}s. "
                          f"Residual: H={h_err:+.3f}°  V={v_err:+.3f}°")
                break

            time.sleep(poll_interval)

        # Fine-settle: SDK overshoot-then-approach backlash correction
        if verbose:
            print(f"\n  Fine-settle (VERY HIGH accuracy — overshoot + backlash correction)...")
        mbx.set_velocity(0, 0, 0)  # 0 → max velocity for the settle move
        mbx.move_angle(hang=h_target, vang=v_target, accuracy="VERY HIGH")
        self._assert_v_in_bounds()                     # reactive fallback after settle

        elapsed = time.time() - t0
        h_act = mbx.convertpostoangle(mbx.H, mbx.current_pos(mbx.H, 1))
        v_act = mbx.convertpostoangle(mbx.V, mbx.current_pos(mbx.V, 1))
        h_err = h_target - h_act
        v_err = v_target - v_act
        log["t"].append(elapsed)
        log["h_actual"].append(h_act)
        log["v_actual"].append(v_act)
        log["h_error"].append(h_err)
        log["v_error"].append(v_err)

        if verbose:
            print(SEP)
            print(f"  Final: H_err={h_err:+.4f}°  V_err={v_err:+.4f}°  t={elapsed:.2f}s")
            self._print_position()
            print(f"{SEP}\n")
        else:
            print(f"    settle: H_err={h_err:+.4f}°  V_err={v_err:+.4f}°  t={elapsed:.2f}s")

        if plot:
            self._plot_pid_results(log)

        return log

    @staticmethod
    def _plot_pid_results(log):
        import matplotlib.pyplot as plt

        t = log["t"]
        h_tgt = log["h_target"]
        v_tgt = log["v_target"]

        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(
            f"PID Move  →  H={h_tgt:.3f}°  V={v_tgt:.3f}°",
            fontsize=13, fontweight="bold",
        )

        # --- H position ---
        ax = axes[0, 0]
        ax.plot(t, log["h_actual"], "b-", linewidth=1.5, label="Actual")
        ax.axhline(h_tgt, color="r", linestyle="--", linewidth=1, label="Target")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (°)")
        ax.set_title("Azimuth (H) — Position")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.4)

        # --- V position ---
        ax = axes[0, 1]
        ax.plot(t, log["v_actual"], "g-", linewidth=1.5, label="Actual")
        ax.axhline(v_tgt, color="r", linestyle="--", linewidth=1, label="Target")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (°)")
        ax.set_title("Elevation (V) — Position")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.4)

        # --- H error ---
        ax = axes[1, 0]
        ax.plot(t, log["h_error"], "b-", linewidth=1.5)
        ax.fill_between(t, log["h_error"], alpha=0.15, color="b")
        ax.axhline(0, color="k", linewidth=0.5)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (°)")
        ax.set_title("Azimuth (H) — Error")
        ax.grid(True, alpha=0.4)

        # --- V error ---
        ax = axes[1, 1]
        ax.plot(t, log["v_error"], "g-", linewidth=1.5)
        ax.fill_between(t, log["v_error"], alpha=0.15, color="g")
        ax.axhline(0, color="k", linewidth=0.5)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (°)")
        ax.set_title("Elevation (V) — Error")
        ax.grid(True, alpha=0.4)

        plt.tight_layout()
        plt.show()

    @staticmethod
    def _plot_scan_results(h_ang, h_db, v_ang, v_db, best_h, best_v, park_log):
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(13, 9))
        fig.suptitle(
            f"Velocity Scan + Park  →  VNA Peak: H={best_h:.3f}°  V={best_v:.3f}°",
            fontsize=13, fontweight="bold",
        )

        ax = axes[0, 0]
        ax.plot(h_ang, h_db, "b-", linewidth=1.2)
        ax.axvline(best_h, color="r", linestyle="--", linewidth=1,
                   label=f"Peak H={best_h:.2f}°")
        ax.set_xlabel("H angle (°)")
        ax.set_ylabel("VNA power (dB)")
        ax.set_title("H Sweep — VNA Power")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.4)

        ax = axes[0, 1]
        ax.plot(v_ang, v_db, "g-", linewidth=1.2)
        ax.axvline(best_v, color="r", linestyle="--", linewidth=1,
                   label=f"Peak V={best_v:.2f}°")
        ax.set_xlabel("V angle (°)")
        ax.set_ylabel("VNA power (dB)")
        ax.set_title("V Sweep — VNA Power")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.4)

        t = park_log["t"]
        ax = axes[1, 0]
        ax.plot(t, park_log["h_actual"], "b-", linewidth=1.5, label="H actual")
        ax.axhline(park_log["h_target"], color="b", linestyle="--", linewidth=1,
                   label="H target")
        ax.plot(t, park_log["v_actual"], "g-", linewidth=1.5, label="V actual")
        ax.axhline(park_log["v_target"], color="g", linestyle="--", linewidth=1,
                   label="V target")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (°)")
        ax.set_title("Park Phase — Position vs Time")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.4)

        ax = axes[1, 1]
        ax.plot(t, park_log["h_error"], "b-", linewidth=1.5, label="H error")
        ax.plot(t, park_log["v_error"], "g-", linewidth=1.5, label="V error")
        ax.fill_between(t, park_log["h_error"], alpha=0.1, color="b")
        ax.fill_between(t, park_log["v_error"], alpha=0.1, color="g")
        ax.axhline(0, color="k", linewidth=0.5)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (°)")
        ax.set_title("Park Phase — Position Error vs Time")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.4)

        plt.tight_layout()
        plt.show()

    def _hold_torque(self):
        """Lock all motors at their current encoder position before the COM port closes.

        Dynamixel PH42 motors may disable torque when the host disconnects. Issuing a
        move_pos to the current position with velocity=0 freezes the position controller
        so the motor maintains that target — and its torque — after the serial port closes.
        """
        try:
            mbx.set_velocity(0, 0, 0)
            for motor in (mbx.H, mbx.V, mbx.P):
                try:
                    cur = mbx.current_pos(motor, 1)
                    mbx.move_pos(motor, cur)
                except Exception:
                    pass
        except Exception:
            pass

    def close(self):
        if self._connected:
            try:
                print("Returning to home (H=0°, V=0°, P=0°) before disconnect...")
                mbx.gotoZERO(accuracy="HIGH")
            except Exception:
                pass
            self._hold_torque()
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

    def _on_exit(self):
        if self._connected:
            try:
                mbx.gotoZERO(accuracy="HIGH")
            except Exception:
                pass
            self._hold_torque()
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
    vna_mode = "SIMULATED" if SIMULATE_VNA else f"REAL @ {VNA_ADDRESS}"
    while True:
        print("")
        print(f"VNA source: {vna_mode}   (edit SIMULATE_VNA at top of file to change)")
        print("Select mode:")
        print("  1 - Manual control")
        print("  2 - Manual control with VNA")
        print("  3 - Autonomous VNA grid scan")
        print("  4 - Autonomous closed-loop DIRECT-MOTION alignment")
        print("  5 - 2D sweep to find the VNA peak")
        print("  6 - Adaptive-speed sweep around a proposed best angle")
        print("  7 - Proportional-velocity VNA scan + precision park at measured peak")
        choice = input("Enter 1-7: ").strip()
        if choice in ("1", "2", "3", "4", "5", "6", "7"):
            break
        print("Invalid input -- enter a number from 1 to 7.")

    def _attach_vna(gim):
        if SIMULATE_VNA:
            gim.connect_vna(simulate=True)
        else:
            gim.connect_vna(VNA_ADDRESS)

    with GimbalController(port="COM7") as gim:
        if choice == "2":
            _attach_vna(gim)
            gim.run_keyboard_control()
        elif choice == "3":
            _attach_vna(gim)
            gim.run_scan()
        elif choice == "4":
            _attach_vna(gim)
            gim.run_direct_align()
        elif choice == "5":
            _attach_vna(gim)
            gim.run_scan()
        elif choice == "6":
            _attach_vna(gim)
            gim.run_adaptive_scan()
        elif choice == "7":
            _attach_vna(gim)
            gim.run_velocity_scan()
        else:
            gim.run_keyboard_control()
