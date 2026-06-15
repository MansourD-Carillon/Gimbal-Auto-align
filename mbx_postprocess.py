#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################################################################
# Copyright 2023-2024 MILLIWAVE SILICON SOLUTIONS, inc.
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

# *********     POST-PROCESSING FUNCTIONS      *********
#   Correction factor
#   Post-processing computations on measured data
#


# IMPORTS
from __future__ import division                             # division compatibility Python 2.7 and Python 3.6+
import os
import sys
import csv
import time
import six

import numpy as np
import matplotlib.pyplot as plt

# matplotlib specific plotting delays - different for Python2.x vs. Python3.x
if six.PY2:
    # print("Setting plt_pause = 1e-3")
    plt_pause = 1e-3                                        # matplotlib: Py2.x - delay=1e-3
else:
    # print("Setting plt_pause = 2e-1")
    plt_pause = 2e-1                                        # matplotlib: Py3.x - delay=2e-1

if sys.platform == "win32":                                 # if we run windows, we can use getch from OS
    from msvcrt import getch
else:                                                       # but if we use MACos or Linux we need to create getch()
    def getch():
        x = six.moves.input()
        if len(x) > 1:
            x = chr(0)
            print("too long")
        elif len(x) == 0:
            x = chr(13)  # enter
            print("enter")
        return x

# global correction factor variables
corr_fname = ""                                             # filename with correction factor
corr_write = False                                          # flag to determine whether to save corr factor with data
corr_xp = np.array([0.0])                                   # frequency points
corr_yp = np.array([0.0])                                   # magnitude corr factor (dB)


# =======================================================
# ============= CORRECTION FACTOR FUNCTIONS =============
# =======================================================

def get_corr():
    """ return correction factor values (x, y, filename) """
    global corr_xp, corr_yp, corr_fname
    return corr_xp, corr_yp, corr_fname


def set_corr(xp, yp, fname):
    """ set correction factor values (x, y, filename)
    return error_code = 1 if there is an error """
    global corr_xp, corr_yp, corr_fname

    error_code = 0
    if not np.all(np.diff(xp) > 0):                         # check that frequencies are unique and ascending
        print("*** ERROR: Correction factor frequencies must be in increasing order -- NO CHANGE MADE")
        print("")
        error_code = 1
    elif len(xp) != len(yp):                                # check freq and mag vectors are same length
        print("*** ERROR: Frequencies and correction factors must be vectors of same length -- NO CHANGE MADE")
        print("")
        error_code = 1
    else:                                                   # set the global variables
        corr_xp = np.array(xp)
        corr_yp = np.array(yp)
        corr_fname = fname
    return error_code


def get_corr_write():
    """ return whether to save corr factor with measured data """
    global corr_write
    return corr_write


def set_corr_write(write):
    """ set whether to save corr factor with measured data """
    global corr_write
    corr_write = write
    return


def load_corr_file(fname):
    """ attempts to load correction factor (freq, corr_fact) from file """
    xp = []
    yp = []
    try:
        csvfile = open(fname, "r")                          # open CSV file for read
        reader = csv.reader(csvfile, lineterminator='\n')   # set line terminator to newline only (no carriage return)
        for row in reader:
            xp.append(float(row[0]))
            yp.append(float(row[1]))
    except IOError:                                         # cannot open file
        print("*** ERROR: Could not open %s" % fname)
        print("")
    return np.array(xp), np.array(yp)


def load_and_set_corr_file(fname):
    """ load corr factor file and set global corr factor, if no errors loading """
    xp, yp = load_corr_file(fname)
    if len(xp) and len(yp):
        set_corr(xp, yp, fname)                             # set corr factor if len(xp) and len(yp) are non-zero
    xp, yp, filename = get_corr()                           # get the current correction factors
    return xp, yp, filename


def clear_corr():
    """ clear correction factor - only save raw data """
    global corr_xp, corr_yp, corr_fname
    set_corr([0.0], [0.0], "")
    return


def is_corr_on():
    """ returns whether a correction factor file has been loaded """
    global corr_xp, corr_yp, corr_fname
    corr_off = (len(corr_xp) == 1 and corr_xp == np.array([0.0])) and \
               (len(corr_yp) == 1 and corr_yp == np.array([0.0])) and \
               (corr_fname == "")
    return not corr_off


def corr_power(freq, db):
    """ returns the corrected power in dB for a list of freq and power"""
    xp, yp, fname = get_corr()
    corr_db = []
    for k in range(len(freq)):
        corr = np.interp(freq[k], xp, yp)                   # interpolate correction factor, use endpoints if beyond correction range
        corr_db.append(db[k] - corr)                        # corr_db = raw - corr_fact
    return corr_db


def save_corr_file(fname):
    """ save the corr factor to file """
    xp, yp, filename = get_corr()
    data = np.transpose(np.array([xp, yp]))

    csvfile = open(fname, "w", buffering=1)
    writer = csv.writer(csvfile, lineterminator="\n")
    writer.writerows(data)
    csvfile.close()
    return


def print_corr():
    """ print corr factor to screen """
    xp, yp, fname = get_corr()

    print("")
    print("Current correction factor")
    print("-------------------------")
    print("Correction file: %s" % fname)
    print("-------------------------")
    for i in range(len(xp)):
        print("%7.2fGHz : %7.2f" % (xp[i] / 1e9, yp[i]))
    print("")


def plot_corr():
    """ plot corr factor values to figure """
    xp, yp, fname = get_corr()

    plt.figure(1)                                           # plot in figure 1
    plt.clf()                                               # clear figure before plotting new one

    plt.plot(xp / 1e9, yp, color='r', marker='.', linestyle='-')    # plot the curve in RED
    plt.xlabel("Frequency (GHz)")
    plt.ylabel("Corr Factor (dB)")
    plt.title("Corr Factor vs. Frequency\n[ %s ]" % fname)

    plt.grid(True)                                          # turn grid on
    plt.draw()                                              # draw the surface on figure 1
    plt.pause(plt_pause)                                    # allow time for the drawing to show on screen
    time.sleep(0.01)

    print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")

    plt.ioff()                                              # turn off plot interactive, makes graph blocking
    plt.show()                                              # closing the plot unblock the function and go to menu
    return


def mbx_corr_menu(DISPLAY_TEST_MENU=False):                 # corr factor menu
    """ main menu for MilliBox correction factor setting """

    path = [os.path.join('..', '..', 'MilliBox_plot_data', 'corr_factor'),
            os.path.join('..', '..', 'MilliBox_plot_data'),
            os.path.join('.', 'corr_factor')]               # search path order

    while True:
        print("")
        print("************* CORRECTION FACTOR MAIN MENU *************")
        print("*")
        if is_corr_on():
            xp, yp, fname = get_corr()
            write = get_corr_write()
            print("*  Corr File: %s" % fname)
            print("*  Save Corr: %s" % write)
        else:
            print("*  Corr File: ---")
            print("*  Save Corr: ---")
        print("*")
        print("******************** USE KEYBOARD *********************")
        print("* Press <ESC> or <q> to quit corr factor menu")
        print("* press <a> to display the current correction factors")
        print("* press <b> to load correction factor file")
        print("* press <c> to clear the correction factors")
        print("* press <d> to toggle saving correction factor with measured data")
        print("*******************************************************")
        if DISPLAY_TEST_MENU:
            print("**********************  TEST MENU  **********************")
            print("* press <!> to load sample correction factor file")
            print("*********************************************************")

        pressedkey = ord(getch().lower())
        if pressedkey == 27 or pressedkey == ord("q"):      # esc or "q": quit
            return

        print(chr(pressedkey))

        fname = ''
        if pressedkey == ord("b"):                          # valid option chosen, enter filename to load
            print("")
            fname = six.moves.input("Filename to load or [ENTER] to cancel: ")
            print("")

        ####################################################
        #  Special TEST modes - to load pre-defined files  #
        ####################################################

        if DISPLAY_TEST_MENU:
            if pressedkey == ord("!"):                      # Shift-1: test mode / load example corr factor file
                fname = 'corr_fact_example.csv'
                pressedkey = ord("b")

        if pressedkey == ord("a"):                          # "a": display corr factor
            print_corr()                                    # print corr factor
            plot_corr()                                     # plot corr factor

        if pressedkey == ord("b"):                          # "b": load corr factor file
            (mypath, myfile) = os.path.split(fname)
            (myname, myext) = os.path.splitext(myfile)
            if myext != ".csv":
                myfile = myfile + ".csv"                    # for any filename without .csv extension, append .csv

            filefound = False
            for pathdir in path:                            # search through the path in order
                if not filefound:                           # if file hasn't been found
                    fullfile = os.path.join(pathdir, mypath, myfile)    # append the path to the filename
                    if os.path.isfile(fullfile):
                        filefound = True                    # found = True
                        fname = fullfile                    # set loadfile with absolute path

            if not filefound:                               # if no file found
                if fname != '':
                    print("*** ERROR: File %s does not exist!" % fname)  # if file does not exist, exit routine
                    print("")
                continue
            else:
                print("Loading correction factor file: %s" % fname)  # display filename with full path
                print("")

            load_and_set_corr_file(fname)

        if pressedkey == ord("c"):                          # "c": clear corr factor
            print("WARNING: Clear all correction factors? [Y/N]")
            print("")
            key = ord(getch().upper())
            while key not in [ord("Y"), ord("N")]:
                key = ord(getch().upper())

            if key == ord("Y"):
                clear_corr()
                print("Clearing correction factors")
            else:
                print("Keeping old correction factors")

            print("")

        if pressedkey == ord("d"):                          # "d": toggle save corr factor
            write = not (get_corr_write())
            set_corr_write(write)
            print("Save Corr with Data = %s" % write)


# ============================================================
# ============= OUTPUT POST-PROCESSING FUNCTIONS =============
# ============================================================

def compute_diff(fname1, fname2, outfile):
    """ compute power difference of 2 measurements """
    data1 = []
    csvfile1 = open(fname1, 'r')                                                # open CSV file for read
    reader1 = csv.reader(csvfile1, lineterminator='\n')                         # set line terminator to newline only (no carriage return)
    for row in reader1:
        data1.append(row)                                                       # read data1
    csvfile1.close()

    data2 = []
    csvfile2 = open(fname2, 'r')                                                # open CSV file for read
    reader2 = csv.reader(csvfile2, lineterminator='\n')                         # set line terminator to newline only (no carriage return)
    for row in reader2:
        data2.append(row)                                                       # read data2
    csvfile2.close()

    if len(data1) != len(data2):                                                # check both files have same number of data points
        print("*** ERROR: number of measurement points not same in the source files - ABORTED")
        print("")
        return

    if data1[0] != data2[0]:                                                    # check both files have same header
        print("*** ERROR: header not same in the source files - ABORTED")
        print("")
        return

    if data1[0][4] == "P" or data1[0][4] == "DPHI":                             # check if data is for 2-axis or 3-axis positioner
        start = 6
    else:
        start = 4

    outdata = data1                                                             # set outdata to take header and motor positions from data1 file
    for y in range(1, len(outdata)):                                            # for each data point
        if data1[y][0] != data2[y][0] or data1[y][2] != data2[y][2]:            # check the 2D sweep are for same points
            print("*** ERROR: sweep points not same in the source files - ABORTED")
            print("")
            return
        for x in range(start, len(outdata[0])):                                 # for each data column
            outdata[y][x] = float(data1[y][x]) - float(data2[y][x])             # return sum of power difference data1 - data2

    csvfile = open(outfile, 'w')                                                # open CSV file for write
    writer = csv.writer(csvfile, lineterminator='\n')                           # set line terminator to newline only (no carriage return)
    writer.writerows(outdata)                                                   # save the combined power
    csvfile.close()

    print("*** SAVED! - %s" % outfile)
    print("")
    return


def compute_power_sum(fname1, fname2, outfile):
    """ compute power sum of 2 orthogonal measurements and output total power """
    data1 = []
    csvfile1 = open(fname1, 'r')                                                # open CSV file for read
    reader1 = csv.reader(csvfile1, lineterminator='\n')                         # set line terminator to newline only (no carriage return)
    for row in reader1:
        data1.append(row)                                                       # read data1
    csvfile1.close()

    data2 = []
    csvfile2 = open(fname2, 'r')                                                # open CSV file for read
    reader2 = csv.reader(csvfile2, lineterminator='\n')                         # set line terminator to newline only (no carriage return)
    for row in reader2:
        data2.append(row)                                                       # read data2
    csvfile2.close()

    if len(data1) != len(data2):                                                # check both files have same number of data points
        print("*** ERROR: number of measurement points not same in the source files - ABORTED")
        print("")
        return

    if data1[0] != data2[0]:                                                    # check both files have same header
        print("*** ERROR: header not same in the source files - ABORTED")
        print("")
        return

    if data1[0][4] == "P" or data1[0][4] == "DPHI":                             # check if data is for 2-axis or 3-axis positioner
        start = 6
    else:
        start = 4

    outdata = data1                                                             # set outdata to take header and motor positions from data1 file
    for y in range(2, len(outdata)):                                            # for each data point
        if data1[y][0] != data2[y][0] or data1[y][2] != data2[y][2]:            # check the 2D sweep are for same points
            print("*** ERROR: sweep points not same in the source files - ABORTED")
            print("")
            return
        for x in range(start, len(outdata[0])):                                 # for each data column
            outdata[y][x] = 10 * np.log10(10 ** (float(data1[y][x]) / 10) + 10 ** (float(data2[y][x]) / 10))    # return sum of power from data1 and data2

    csvfile = open(outfile, 'w')                                                # open CSV file for write
    writer = csv.writer(csvfile, lineterminator='\n')                           # set line terminator to newline only (no carriage return)
    writer.writerows(outdata)                                                   # save the combined power
    csvfile.close()

    print("*** SAVED! - %s" % outfile)
    print("")
    return


def mbx_postproc_menu(DISPLAY_TEST_MENU=False):                                 # post-processing menu
    """ main menu for MilliBox data post-procesing """

    while True:
        print("")
        print("************** POST-PROCESSING MAIN MENU **************")
        print("******************** USE KEYBOARD *********************")
        print("* Press <ESC> or <q> to quit post-process menu")
        print("* press <a> to calculate difference of two measurements")
        print("* press <b> to calculate total power from orthogonal measurements")
        # print("* press <c> to compute TRP")
        print("*******************************************************")
        # if DISPLAY_TEST_MENU:
        #     print("**********************  TEST MENU  **********************")
        #     print("* press <!> to ")
        #     print("*********************************************************")

        pressedkey = ord(getch().lower())
        if pressedkey == 27 or pressedkey == ord("q"):                          # esc or "q": quit
            return

        print(chr(pressedkey))

        if pressedkey == ord("a") or pressedkey == ord("b"):                    # "a": calculate difference or "b": total power
            print("")
            rundir = six.moves.input("Run directory or [ENTER] to cancel: ")
            fname1 = six.moves.input("First file to load or [ENTER] to cancel: ")
            fname2 = six.moves.input("Second file to load or [ENTER] to cancel: ")
            outfile = six.moves.input("Output file to save or [ENTER] to cancel: ")
            print("")

            fullfile = os.path.join(rundir, fname1)                             # append the path to the filename
            if os.path.isfile(fullfile):
                fname1 = fullfile
            else:
                print("*** ERROR: File %s does not exist!" % fname1)            # if file does not exist, exit routine
                print("")
                continue

            fullfile = os.path.join(rundir, fname2)                             # append the path to the filename
            if os.path.isfile(fullfile):
                fname2 = fullfile
            else:
                print("*** ERROR: File %s does not exist!" % fname2)            # if file does not exist, exit routine
                print("")
                continue

            outfile = os.path.join(rundir, outfile)                             # append the path to the filename

            if pressedkey == ord("a"):
                compute_diff(fname1, fname2, outfile)                           # compute difference
            elif pressedkey == ord("b"):
                compute_power_sum(fname1, fname2, outfile)                      # compute total power

        # if pressedkey == ord("c"):                                              # "c": compute TRP
        #     print("Placeholder to compute TRP")

    return
