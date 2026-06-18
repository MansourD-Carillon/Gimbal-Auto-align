def save_cw_time_cut(inst, axis_name, freq_hz, angle_log_t, angle_log_deg,
                     trigger_time_s, sample_period_s, filename):
    s = inst.get_sparam()  # complex CW time-domain trace from active ShockLine trace

    mag_db = 20*np.log10(np.abs(s))
    phase_deg = np.angle(s, deg=True)
    phase_unwrapped_deg = np.unwrap(np.radians(phase_deg)) * 180/np.pi

    vna_time_s = trigger_time_s + np.arange(len(s)) * sample_period_s

    angle_interp = np.interp(
        vna_time_s,
        angle_log_t,
        angle_log_deg,
        left=np.nan,
        right=np.nan
    )

    np.savez(
        filename,
        axis=axis_name,
        frequency_hz=freq_hz,
        vna_time_s=vna_time_s,
        angle_deg=angle_interp,
        magnitude_db=mag_db,
        phase_deg=phase_unwrapped_deg,
        raw_phase_deg=phase_deg,
        angle_log_time_s=angle_log_t,
        angle_log_deg=angle_log_deg,
    )