#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################################################################
# Copyright 2020-2024 MILLIWAVE SILICON SOLUTIONS, inc.
# Author: Chinh Doan - Milliwave Silicon Solutions

# This file is part of the MilliBox Controller.

# The MilliBox Controller is free software: you can redistribute it and/or modify it under the terms of the GNU General
# Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.

# The MilliBox Controller is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
# details.

# You should have received a copy of the GNU General Public License along with the MilliBox Controller. If not, see
# <https://www.gnu.org/licenses/>.
########################################################################################################################

import os
import sys
import csv
import six
from mbx_realtimeplot import *
from mbx_functions import input_num

if sys.platform == "win32":                                 # if we run windows, we can use getch from OS
    from msvcrt import getch
else:                                                       # but if we use MACos or Linux we need to create getch()
    def getch():
        x = six.moves.input()
        if len(x) > 1:
            x = chr(0)
            print("too long")
        elif len(x) == 0:
            x = chr(13)     # enter
            print("enter")
        return x


def mbx_plot(DISPLAY_TEST_MENU=False):                                          # plotting menu
    """ main menu for MilliBox plotting from file """

    dataDir = os.path.join('..', '..', 'MilliBox_plot_data')                    # default data directory is ..\..\MilliBox_plot_data
    path = ['', dataDir, os.path.join(dataDir, 'sample_data')]                  # search path order

    while True:
        print("")
        print("************* PLOTTING MAIN MENU *********************")
        print("**************** USE KEYBOARD ************************")
        print("* Press <ESC> or <q> to quit plot menu")
        print("* press <a> for 1D sweep - Line Plot")
        print("* press <b> for HV sweep - Line Plot")
        print("* press <c> for 2D sweep - Surface Plot (HV) / Direction Cosine Plot (SPH)")
        print("* press <d> for 2D sweep - Heatmap Plot (HV) / Polar Plot (SPH)")
        print("* press <e> for 2D sweep - Multi-Line Slice Plot (HV)")
        print("* press <f> for 2D sweep - 3D Radiation Pattern Plot (HV/SPH)")
        print("******************************************************")
        if DISPLAY_TEST_MENU:
            print("**********************  TEST MENU  **********************")
            print("* press <!> for 1D line plot of sample 1D sweep")
            print("* press <@> for HV line plot of sample HV sweep")
            print("* press <#> for surface plot of sample 2D sweep")
            print("* press <$> for heatmap plot of sample 2D sweep")
            print("* press <%> for multi-line slice plot of sample 2D sweep")
            print("* press <^> for 3D radiation pattern plot of sample 2D sweep")
            print("*********************************************************")
            print("* press <1> for 1D line plot of sample 1D sweep (VNA)")
            print("* press <2> for HV line plot of sample HV sweep (VNA)")
            print("* press <3> for surface plot of sample 2D sweep (VNA)")
            print("* press <4> for heatmap plot of sample 2D sweep (VNA)")
            print("* press <5> for multi-line slice plot of sample 2D sweep (VNA)")
            print("* press <6> for 3D radiation pattern plot of sample 2D sweep (VNA)")
            print("*********************************************************")

        pressedkey = ord(getch().lower())
        loadfile = ''

        if pressedkey == 27 or pressedkey == ord("q"):                          # esc or "q": quit
            print("exit called bye bye!")
            return

        print(chr(pressedkey))

        if pressedkey-ord("a") in range(ord("f")-ord("a")+1):                   # valid option chosen, enter filename
            print("")
            loadfile = six.moves.input("Filename to plot or [ENTER] to cancel: ")
            print("")

        ####################################################
        #  Special TEST modes - to plot pre-defined files  #
        ####################################################

        if DISPLAY_TEST_MENU:
            if pressedkey == ord("!"):                                          # Shift-1: test mode / 1D plot for stored file
                loadfile = 'mbx_capture_1d_H_example.csv'
                pressedkey = ord("a")
            if pressedkey == ord("@"):                                          # Shift-2: test mode / HV plot for stored file
                loadfile = 'mbx_capture_HV_example.csv'
                pressedkey = ord("b")
            if pressedkey == ord("#"):                                          # Shift-3: test mode / surface plot for stored file
                loadfile = 'mbx_capture_2d_full_sweep_example.csv'
                pressedkey = ord("c")
            if pressedkey == ord("$"):                                          # Shift-4: test mode / heatmap plot for stored file
                loadfile = 'mbx_capture_2d_full_sweep_example.csv'
                pressedkey = ord("d")
            if pressedkey == ord("%"):                                          # Shift-5: test mode / multi-line plot for stored file
                loadfile = 'mbx_capture_2d_full_sweep_example.csv'
                pressedkey = ord("e")
            if pressedkey == ord("^"):                                          # Shift-6: test mode / radiation pattern for stored file
                loadfile = 'mbx_capture_2d_full_sweep_example.csv'
                pressedkey = ord("f")

            if pressedkey == ord("1"):                                          # 1: test mode / 1D plot for stored VNA file
                loadfile = 'mbx_capture_1d_H_VNA_example.csv'
                pressedkey = ord("a")
            if pressedkey == ord("2"):                                          # 2: test mode / HV plot for stored VNA file
                loadfile = 'mbx_capture_HV_VNA_example.csv'
                pressedkey = ord("b")
            if pressedkey == ord("3"):                                          # 3: test mode / surface plot for stored VNA file
                loadfile = 'mbx_capture_2d_full_sweep_VNA_example.csv'
                pressedkey = ord("c")
            if pressedkey == ord("4"):                                          # 4: test mode / heatmap plot for stored VNA file
                loadfile = 'mbx_capture_2d_full_sweep_VNA_example.csv'
                pressedkey = ord("d")
            if pressedkey == ord("5"):                                          # 5: test mode / multi-line plot for stored VNA file
                loadfile = 'mbx_capture_2d_full_sweep_VNA_example.csv'
                pressedkey = ord("e")
            if pressedkey == ord("6"):                                          # 6: test mode / radiation pattern for stored VNA file
                loadfile = 'mbx_capture_2d_full_sweep_VNA_example.csv'
                pressedkey = ord("f")

        if pressedkey - ord("a") in range(ord("f") - ord("a") + 1):             # valid plot option chosen

            filefound = False
            for pathdir in path:                                                # search through the path in order
                if not filefound:                                               # if file hasn't been found
                    fullfile = os.path.join(pathdir, loadfile)                  # append the path to the filename
                    if os.path.isfile(fullfile):
                        filefound = True                                        # found = True
                        loadfile = fullfile                                     # set loadfile with absolute path

            if not filefound:                                                   # if no file found
                if loadfile != '':
                    print("*** ERROR: File %s does not exist! ***" % loadfile)  # if file does not exist, exit routine
                continue
            else:
                print("Plotting file: %s" % loadfile)                           # display filename with full path

            data = []                                                           # format data and plot
            csvplot = open(loadfile, 'r')                                       # open CSV file for read
            capture = csv.reader(csvplot, lineterminator='\n')                  # set line terminator to newline only (no carriage return)
            first = True
            freqList = []
            for row in capture:                                                 # cycle through one row at a time
                if first:
                    if row[0] == "V":                                           # parse the header to determine HV or SPHERICAL data
                        plot_gim_type = "HV"
                    elif row[0] == "PHI":
                        plot_gim_type = "SPHERICAL"

                    if row[4] == "P" or row[4] == "DPHI":                       # parse the header to get freq list
                        polFile = True
                        for x in range(len(row)-6):
                            freqList.append(float(row[6+x]))
                    else:
                        polFile = False
                        for x in range(len(row)-4):
                            freqList.append(float(row[4+x]))
                    first = False
                else:
                    data.append(row)                                            # append each row to data

            if len(freqList) > 1:
                print("")
                print("*** VNA DATA FILE DETECTED ***")
                print("")
                plot_f = float(input_num("Enter the freq [GHz] to plot (or enter 0 to plot min/mid/max): ", 0))
                plot_freq = [plot_f*1e9]

                if plot_freq[0] == 0:                                           # pick first, mid, last freqs for VNA
                    freq_unique = np.unique(freqList)                           # determine unique frequencies

                    freqIdx = []
                    freqIdx.append(np.abs(np.array(freqList) - freq_unique[0]).argmin())    # pick first, mid, last freqs for VNA
                    freqIdx.append(np.abs(np.array(freqList) - freq_unique[int(round(len(freq_unique)/2))]).argmin())
                    freqIdx.append(np.abs(np.array(freqList) - freq_unique[-1]).argmin())

                else:
                    freqIdx = [np.abs(np.array(freqList) - plot_freq[0]).argmin()]  # find index for value that is closest to plot_freq
            else:
                freqIdx = [0]

            if pressedkey == ord("a"):                                          # line plot
                if plot_gim_type == "HV":
                    for idx in freqIdx:
                        Vangle = []                                             # initialize the V angle sweep
                        Hangle = []                                             # initialize the H angle sweep
                        heatmap = []                                            # initialize the sweep data

                        for x in range(len(data)):
                            Vangle.append(float(data[x][0]))                    # append the V angle
                            Hangle.append(float(data[x][2]))                    # append the H angle
                            if not polFile:
                                heatmap.append(float(data[x][4+idx]))           # append the measured data
                            else:
                                heatmap.append(float(data[x][6+idx]))

                        Vangle = np.unique(Vangle)                              # compute unique V angles
                        Hangle = np.unique(Hangle)                              # compute unique H angles

                        if len(Vangle) == 1:
                            direction = 'H'                                     # if # V angles=1 -> dir = H
                        elif len(Hangle) == 1:
                            direction = 'V'                                     # if # H angles=1 -> dir = V
                        else:
                            print("*** ERROR: Incorrect data format ***")       # if H>1 and V>1 -> wrong format (2D)
                            continue

                        vert = np.max(Vangle)                                   # set "current point" to last point
                        hori = np.max(Hangle)

                        if polFile:
                            pangle = float(data[0][4])
                        else:
                            pangle = None

                        display_1dplot(direction, Vangle, Hangle, heatmap, vert, hori, plot_freq=freqList[idx], pangle=pangle)   # plot data

                if plot_gim_type == "SPHERICAL":
                    for idx in freqIdx:
                        PHangle = []                                            # initialize the PH angle sweep
                        THangle = []                                            # initialize the TH angle sweep
                        heatmap = []                                            # initialize the sweep data

                        for x in range(len(data)):
                            PHangle.append(float(data[x][0]))                   # append the PH angle
                            THangle.append(float(data[x][2]))                   # append the TH angle
                            if not polFile:
                                heatmap.append(float(data[x][4 + idx]))         # append the measured data
                            else:
                                heatmap.append(float(data[x][6 + idx]))

                        PHangle = np.unique(PHangle)                            # compute unique PH angles
                        THangle = np.unique(THangle)                            # compute unique TH angles

                        if len(PHangle) == 1:
                            direction = 'T'                                     # if # PH angles=1 -> dir = T
                        elif len(THangle) == 1:
                            direction = 'P'                                     # if # TH angles=1 -> dir = P
                        else:
                            print("*** ERROR: Incorrect data format ***")       # if TH>1 and PH>1 -> wrong format (2D)
                            continue

                        phi = np.max(PHangle)                                   # set "current point" to last point
                        theta = np.max(THangle)

                        if polFile:
                            dphi = float(data[0][4])
                        else:
                            dphi = None

                        display_1dplot_sph(direction, PHangle, THangle, heatmap, phi, theta, plot_freq=freqList[idx], dphi=dphi)  # plot data

            if pressedkey == ord("b"):                                          # HV plot
                if plot_gim_type == "HV":
                    for idx in freqIdx:
                        Vangle = []                                             # initialize the V angle sweep
                        Hangle = []                                             # initialize the H angle sweep
                        heatmapV = []                                           # initialize the V sweep data
                        heatmapH = []                                           # initialize the H sweep data

                        Hsweep = True                                           # CSV file starts with H data
                        for x in range(len(data)):
                            if Hsweep and (float(data[x][0]) != 0.0):           # check until first time V angle <> 0
                                Hsweep = False
                            if Hsweep:                                          # if H sweep
                                Hangle.append(float(data[x][2]))                # append the H angle
                                if not polFile:
                                    heatmapH.append(float(data[x][4+idx]))      # append the H data
                                else:
                                    heatmapH.append(float(data[x][6+idx]))      # append the H data
                            else:                                               # if V sweep
                                Vangle.append(float(data[x][0]))                # append the V angle
                                if not polFile:
                                    heatmapV.append(float(data[x][4+idx]))      # append the V data
                                else:
                                    heatmapV.append(float(data[x][6+idx]))      # append the V data
                        if len(Hangle) <= 1 or len(Vangle) <= 1 or len(Vangle) != len(np.unique(Vangle)):   # sanity check for HV data validity
                            print("*** ERROR: Incorrect data format ***")
                            continue

                        if polFile:
                            pangle = float(data[0][4])
                        else:
                            pangle = None

                        blocking = 1
                        display_hvplot(Vangle, Hangle, heatmapV, heatmapH, blocking, plot_freq=freqList[idx], pangle=pangle)   # plot data

                if plot_gim_type == "SPHERICAL":
                    for idx in freqIdx:
                        PHangle = []                                            # initialize the PH angle sweep
                        THangle = []                                            # initialize the TH angle sweep
                        heatmapPH00 = []                                        # initialize the PH=0 sweep data
                        heatmapPH90 = []                                        # initialize the PH=90 sweep data

                        PH00sweep = True                                        # CSV file starts with PH=0 data
                        for x in range(len(data)):
                            if PH00sweep and (float(data[x][0]) != 0.0):        # check until first time PH angle <> 0
                                PH00sweep = False
                            if PH00sweep:                                       # if PH=0 sweep
                                THangle.append(float(data[x][2]))               # append the TH angle
                                if not polFile:
                                    heatmapPH00.append(float(data[x][4 + idx])) # append the PHI=0 data
                                else:
                                    heatmapPH00.append(float(data[x][6 + idx])) # append the PHI=0 data
                            else:                                               # if PH=90 sweep
                                PHangle.append(float(data[x][0]))               # append the PH angle
                                if not polFile:
                                    heatmapPH90.append(float(data[x][4 + idx])) # append the PHI=90 data
                                else:
                                    heatmapPH90.append(float(data[x][6 + idx])) # append the PHI=90 data
                        if len(THangle) <= 1 or len(PHangle) <= 1 or len(THangle) != len(np.unique(THangle)):  # sanity check for HV data validity
                            print("*** ERROR: Incorrect data format ***")
                            continue

                        if polFile:
                            dphi = float(data[0][4])
                        else:
                            dphi = None

                        blocking = 1
                        display_hvplot_sph(PHangle, THangle, heatmapPH00, heatmapPH90, blocking, plot_freq=freqList[idx], dphi=dphi)  # plot data

            if chr(pressedkey) in ['c', 'd', 'e', 'f']:                         # 2D data
                if plot_gim_type == "HV":
                    for idx in freqIdx:
                        Vangle = []                                             # initialize the V angle sweep
                        Hangle = []                                             # initialize the H angle sweep
                        heatmap = []                                            # initialize the captured data

                        for x in range(len(data)):
                            Vangle.append(float(data[x][0]))                    # append the V angle
                            Hangle.append(float(data[x][2]))                    # append the H angle
                            if not polFile:
                                heatmap.append(float(data[x][4+idx]))           # append the captured data
                            else:
                                heatmap.append(float(data[x][6+idx]))           # append the captured data

                        Vangle = np.unique(Vangle)                              # compute unique V angles
                        Hangle = np.unique(Hangle)                              # compute unique H angles

                        if len(Vangle)*len(Hangle) != len(heatmap) or len(Vangle) <= 1 or len(Hangle) <= 1:     # sanity check for 2D data validity
                            print("*** ERROR: Incorrect data format ***")
                            continue

                        heatmap = np.array(heatmap).reshape(len(Vangle), len(Hangle)).transpose().tolist()      # reshape data

                        vert = np.max(Vangle)                                   # set "current point" to last point
                        hori = np.max(Hangle)
                        step = np.max(np.diff(Hangle))                          # compute the step size

                        if polFile:
                            pangle = float(data[0][4])
                        else:
                            pangle = None

                        if pressedkey == ord('c'):
                            display_surfplot(Vangle, Hangle, heatmap, vert, hori, plot_freq=freqList[idx], pangle=pangle)          # 3D surface plot
                        elif pressedkey == ord('d'):
                            display_heatmap(Vangle, Hangle, heatmap, vert, hori, plot_freq=freqList[idx], pangle=pangle)           # 2D heatmap plot
                        elif pressedkey == ord('e'):
                            display_multilineplot(Vangle, Hangle, heatmap, vert, hori, plot_freq=freqList[idx], pangle=pangle)     # multi-line slice plot
                        elif pressedkey == ord('f'):
                            display_millibox3d_ant_pattern(Vangle, Hangle, heatmap, vert, hori, step, plot_freq=freqList[idx], pangle=pangle, plot_range=50)  # 3D radiation pattern plot

                            print()
                            print("Save STL file for 3D printing? [Y/N]")       # option to save 3D radpat to STL
                            save_stl = 0
                            while chr(save_stl) not in ['y', 'n']:
                                save_stl = ord(getch().lower())
                            if save_stl == ord('y'):

                                (mypath, myfile) = os.path.split(loadfile)
                                (myname, myext) = os.path.splitext(myfile)
                                stlname = "radpat_%s_f%0.0f.stl" % (myname, freqList[idx]/1e6)      # STL file name based on input file and freq
                                outdir = os.path.join('..', '..', 'MilliBox_plot_data', 'stl')
                                if not os.path.isdir(outdir):
                                    print("*** Creating output directory MilliBox_plot_data\\stl ***")
                                    os.mkdir(outdir)                            # create directory if it doesn't exist
                                outname = os.path.join(outdir, stlname)

                                plot_range = input_num("Enter dB dynamic range for STL file (ENTER for default=40): ", 40)
                                stl_max_mm = input_num("Enter maximum radial dimension in mm (ENTER for default=200): ", 200)
                                stl_min_mm = input_num("Enter minimum radial dimension in mm (ENTER for default=25): ", 25)

                                print_millibox3d_ant_pattern(Vangle, Hangle, heatmap, vert, hori, step, plot_freq=freqList[idx], plot_range=plot_range, stl_max_mm=stl_max_mm, stl_min_mm=stl_min_mm, outname=outname)  # 3D radiation pattern plot

                if plot_gim_type == "SPHERICAL":
                    for idx in freqIdx:
                        PHangle = []                                            # initialize the PH angle sweep
                        THangle = []                                            # initialize the TH angle sweep
                        heatmap = []                                            # initialize the captured data

                        for x in range(len(data)):
                            PHangle.append(float(data[x][0]))                   # append the PH angle
                            THangle.append(float(data[x][2]))                   # append the TH angle
                            if not polFile:
                                heatmap.append(float(data[x][4+idx]))           # append the captured data
                            else:
                                heatmap.append(float(data[x][6+idx]))           # append the captured data

                        PHangle = np.unique(PHangle)                            # compute unique PH angles
                        THangle = np.unique(THangle)                            # compute unique TH angles

                        if len(PHangle)*len(THangle) != len(heatmap) or len(PHangle) <= 1 or len(THangle) <= 1:     # sanity check for 2D data validity
                            print("*** ERROR: Incorrect data format ***")
                            continue

                        heatmap = np.array(heatmap).reshape(len(PHangle), len(THangle)).transpose().tolist()      # reshape data

                        phi = np.max(PHangle)                                   # set "current point" to last point
                        theta = np.max(THangle)
                        step = np.max(np.diff(THangle))                         # compute the step size

                        if polFile:
                            dphi = float(data[0][4])
                        else:
                            dphi = None

                        if pressedkey == ord('c'):
                            display_dir_cosine_sph(PHangle, THangle, heatmap, phi, theta, plot_freq=freqList[idx], dphi=dphi)
                        elif pressedkey == ord('d'):
                            display_polar_sph(PHangle, THangle, heatmap, phi, theta, plot_freq=freqList[idx], dphi=dphi)
                        elif pressedkey == ord('e'):
                            print("*** ERROR: Invalid plot for SPHERICAL coordinates ***")
                        elif pressedkey == ord('f'):
                            display_millibox3d_ant_pattern_sph(PHangle, THangle, heatmap, phi, theta, step, plot_freq=freqList[idx], dphi=dphi, plot_range=50)

                            print()
                            print("Save STL file for 3D printing? [Y/N]")       # option to save 3D radpat to STL
                            save_stl = 0
                            while chr(save_stl) not in ['y', 'n']:
                                save_stl = ord(getch().lower())
                            if save_stl == ord('y'):

                                (mypath, myfile) = os.path.split(loadfile)
                                (myname, myext) = os.path.splitext(myfile)
                                stlname = "radpat_%s_f%0.0f.stl" % (myname, freqList[idx]/1e6)      # STL file name based on input file and freq
                                outdir = os.path.join('..', '..', 'MilliBox_plot_data', 'stl')
                                if not os.path.isdir(outdir):
                                    print("*** Creating output directory MilliBox_plot_data\\stl ***")
                                    os.mkdir(outdir)                            # create directory if it doesn't exist
                                outname = os.path.join(outdir, stlname)

                                plot_range = input_num("Enter dB dynamic range for STL file (ENTER for default=40): ", 40)
                                stl_max_mm = input_num("Enter maximum radial dimension in mm (ENTER for default=200): ", 200)
                                stl_min_mm = input_num("Enter minimum radial dimension in mm (ENTER for default=25): ", 25)

                                print_millibox3d_ant_pattern_sph(PHangle, THangle, heatmap, phi, theta, step, plot_freq=freqList[idx], dphi=dphi, plot_range=plot_range, stl_max_mm=stl_max_mm, stl_min_mm=stl_min_mm, outname=outname)  # 3D radiation pattern plot

            print("")


if __name__ == "__main__":
    mbx_plot()
