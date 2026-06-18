# -*- coding: utf-8 -*-
"""
Created on Thu Jun 18 12:23:42 2026

@author: AlemohammadMiladL5CA
"""

import numpy as np
import matplotlib.pyplot as plt

NPZ_FILE = r"C:\temp\gimbal_cuts\H_cut_29.000GHz.npz"

d = np.load(NPZ_FILE)

angle = d["angle_deg"]
mag = d["magnitude_db"]
phase = d["phase_deg"]

good = np.isfinite(angle)

plt.figure()
plt.plot(angle[good], mag[good])
plt.xlabel("Angle (deg)")
plt.ylabel("Magnitude (dB)")
plt.title("CW radiation cut magnitude")
plt.grid(True)

plt.figure()
plt.plot(angle[good], phase[good])
plt.xlabel("Angle (deg)")
plt.ylabel("Unwrapped phase (deg)")
plt.title("CW radiation cut phase")
plt.grid(True)

plt.show()