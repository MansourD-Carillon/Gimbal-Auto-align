# continuous_cw_radiation_cut.py

import os
import time
import csv
import numpy as np

import mbx_functions as mbx
import mbx_instrument as equip

# ---------------- USER CONFIG ----------------

GIMBAL_COM_PORT = "COM7"          # change if needed
GIMBAL_BAUDRATE = 1000000

VNA_ADDR = ["TCPIP0::127.0.0.1::5001::SOCKET"]  # ShockLine socket address
MEAS_MODE = "VNA"

# USB-serial TTL output used to pulse ShockLine External Trigger In.
# Set to None if you already trigger externally some other way.
TTL_COM_PORT = None               # example: "COM9"

FREQ_HZ = 29.0e9                  # user-defined CW frequency metadata

# Must match your ShockLine CW Time Domain setup
SAMPLE_PERIOD_S = 1e-3            # seconds/sample; edit to match VNA time record
OUTPUT_DIR = r"C:\temp\gimbal_cuts"

# Cuts
H_MINUS = -40.0
H_PLUS  =  40.0
V_MINUS = -40.0
V_PLUS  =  40.0
H0 = 0.0
V0 = 0.0
PREROLL_DEG = 10.0

POLL_S = 0.002                    # gimbal angle polling interval

# ------------------------------------------------


def clamp_angle(motor, angle):
    gm = mbx.get_gim_motion()
    lim = gm[motor]["anglelim"]
    return max(lim[0], min(lim[1], angle))


class SerialTtlPulser:
    def __init__(self, com_port, pulse_s=0.010):
        import serial
        self.ser = serial.Serial(com_port, baudrate=9600, timeout=0)
        self.pulse_s = pulse_s
        self.ser.dtr = False
        self.ser.rts = False

    def pulse(self):
        # Use DTR as TTL-like control line. Use RTS instead if your wiring uses RTS.
        self.ser.dtr = True
        time.sleep(self.pulse_s)
        self.ser.dtr = False

    def close(self):
        self.ser.close()


def setup_gimbal():
    err = mbx.connect_detailed(GIMBAL_COM_PORT, GIMBAL_BAUDRATE)
    if err != 0:
        raise RuntimeError(f"Gimbal connection failed with error {err}")

    mbx.set_gim_motion_default()
    mbx.gotoZERO("HIGH")
    print("Gimbal connected and homed.")


def setup_vna():
    inst = equip.inst_setup(MEAS_MODE, VNA_ADDR)
    inst.init_meas()
    print("VNA connected.")

    # IMPORTANT:
    # Put ShockLine in CW Time Domain manually or from your saved cal/state.
    # These commands are intentionally minimal because CW TD setup differs by ShockLine model.
    # The existing Anritsu driver can read complex trace data using get_sparam().
    try:
        inst.write("INIT:CONT OFF")
        inst.write("TRIG:SOUR EXT")
    except Exception:
        print("Warning: external trigger SCPI setup failed or unsupported; configure trigger manually on VNA.")

    return inst


def arm_vna_for_external_trigger(inst):
    """
    Arm VNA and wait for external TTL.
    Depending on ShockLine firmware, INIT may be correct, or you may need to
    arm manually on the front panel/software.
    """
    try:
        inst.write("INIT:CONT OFF")
        inst.write("TRIG:SOUR EXT")
        inst.write("INIT")
    except Exception:
        print("Warning: could not arm VNA by SCPI. Make sure ShockLine is armed manually.")


def read_vna_complex_trace(inst):
    """
    Reads active ShockLine trace as complex data.
    Existing mbx_instrument.py Anritsu driver parses real/imag pairs into complex samples,
    and get_s_dbphase converts to magnitude dB and phase degrees.
    """
    s = np.asarray(inst.get_sparam(), dtype=np.complex128)
    mag_db = 20.0 * np.log10(np.abs(s))
    phase_raw_deg = np.angle(s, deg=True)
    phase_unwrapped_deg = np.unwrap(np.radians(phase_raw_deg)) * 180.0 / np.pi
    return s, mag_db, phase_raw_deg, phase_unwrapped_deg


def current_angle(axis_motor):
    return mbx.convertpostoangle(axis_motor, mbx.current_pos(axis_motor, 1))


def move_hv_blocking(h_angle=None, v_angle=None):
    kwargs = {}
    if h_angle is not None:
        kwargs["hang"] = clamp_angle(mbx.H, h_angle)
    if v_angle is not None:
        kwargs["vang"] = clamp_angle(mbx.V, v_angle)
    mbx.move_angle(accuracy="HIGH", **kwargs)


def run_one_cut(inst, ttl, axis_name, start_angle, trigger_angle, stop_angle,
                fixed_h=None, fixed_v=None):
    """
    axis_name: "H" or "V"
    start_angle: H- - 10 deg or V- - 10 deg
    trigger_angle: H- or V-
    stop_angle: H+ or V+
    """

    if axis_name.upper() == "H":
        motor = mbx.H
        move_hv_blocking(h_angle=start_angle, v_angle=fixed_v)
    else:
        motor = mbx.V
        move_hv_blocking(h_angle=fixed_h, v_angle=start_angle)

    print(f"\nArming VNA for {axis_name} cut...")
    arm_vna_for_external_trigger(inst)

    time.sleep(0.2)

    end_pos = mbx.convertangletopos(motor, clamp_angle(motor, stop_angle))

    angle_log_t = []
    angle_log_deg = []

    triggered = False
    trigger_time_pc = None

    print(f"Starting {axis_name} sweep: {start_angle:.2f} -> {stop_angle:.2f} deg")
    mbx.write_accel()
    mbx.move_pos(motor, end_pos)  # non-blocking continuous move

    while mbx.check_is_moving():
        t = time.time()
        a = current_angle(motor)

        angle_log_t.append(t)
        angle_log_deg.append(a)

        if not triggered and a >= trigger_angle:
            trigger_time_pc = time.time()
            if ttl is not None:
                ttl.pulse()
            else:
                print("TTL_COM_PORT is None: no physical TTL pulse sent.")
            triggered = True
            print(f"TTL fired at {axis_name}={a:.3f} deg")

        time.sleep(POLL_S)

    # final angle sample
    angle_log_t.append(time.time())
    angle_log_deg.append(current_angle(motor))

    if not triggered:
        raise RuntimeError(f"{axis_name} sweep finished but never crossed trigger angle.")

    print("Motion complete. Reading VNA trace...")
    complex_s, mag_db, phase_raw_deg, phase_unwrapped_deg = read_vna_complex_trace(inst)

    n = len(mag_db)
    vna_time_s = trigger_time_pc + np.arange(n) * SAMPLE_PERIOD_S

    angle_interp = np.interp(
        vna_time_s,
        np.asarray(angle_log_t),
        np.asarray(angle_log_deg),
        left=np.nan,
        right=np.nan
    )

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    fname_base = f"{axis_name}_cut_{FREQ_HZ/1e9:.3f}GHz"
    npz_path = os.path.join(OUTPUT_DIR, fname_base + ".npz")
    csv_path = os.path.join(OUTPUT_DIR, fname_base + ".csv")

    np.savez(
        npz_path,
        axis=axis_name,
        frequency_hz=FREQ_HZ,
        sample_period_s=SAMPLE_PERIOD_S,
        vna_time_s=vna_time_s,
        angle_deg=angle_interp,
        magnitude_db=mag_db,
        phase_deg=phase_unwrapped_deg,
        raw_phase_deg=phase_raw_deg,
        complex_s=complex_s,
        angle_log_time_s=np.asarray(angle_log_t),
        angle_log_deg=np.asarray(angle_log_deg),
        trigger_time_pc=trigger_time_pc,
    )

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["axis", "frequency_hz", "sample_index", "time_s", "angle_deg",
                    "magnitude_db", "phase_deg", "raw_phase_deg",
                    "real_s", "imag_s"])
        for i in range(n):
            w.writerow([
                axis_name,
                FREQ_HZ,
                i,
                vna_time_s[i] - trigger_time_pc,
                angle_interp[i],
                mag_db[i],
                phase_unwrapped_deg[i],
                phase_raw_deg[i],
                complex_s[i].real,
                complex_s[i].imag,
            ])

    print(f"Saved:\n  {npz_path}\n  {csv_path}")
    return npz_path, csv_path


def main():
    setup_gimbal()
    inst = setup_vna()

    ttl = SerialTtlPulser(TTL_COM_PORT) if TTL_COM_PORT else None

    try:
        run_one_cut(
            inst, ttl,
            axis_name="H",
            start_angle=H_MINUS - PREROLL_DEG,
            trigger_angle=H_MINUS,
            stop_angle=H_PLUS,
            fixed_v=V0,
        )

        run_one_cut(
            inst, ttl,
            axis_name="V",
            start_angle=V_MINUS - PREROLL_DEG,
            trigger_angle=V_MINUS,
            stop_angle=V_PLUS,
            fixed_h=H0,
        )

    finally:
        if ttl is not None:
            ttl.close()

        try:
            inst.cont_trigger()
        except Exception:
            pass

        mbx.gotoZERO("HIGH")
        mbx.close()


if __name__ == "__main__":
    main()