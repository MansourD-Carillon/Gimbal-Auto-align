#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################################################################
# Copyright 2018-2024 MILLIWAVE SILICON SOLUTIONS, inc.
# Author: Jeanmarc Laurent, Chinh Doan, Antonin Laurent - Milliwave Silicon Solutions

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

from __future__ import division                             # division compatibility Python 2.7 and Python 3.6+
import sys
import six
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
from mbx_functions import *
from mbx_plot import *
import mbx_instrument as equip
import mbx_test_config as config
import mbx_postprocess as proc
import pickle
import os

ERROR_DICT = {
    1: "No saved device name",                              # Unused text
    2: "No saved Baudrate",                                 # Unused text
    3: "Could not connect to port. U2D2 may be disconnected from USB port, or the port may be in use by another process.",
    4: "Could not set baudrate. U2D2 may be disconnected, or unknown error",
    10: "GIM could not be identified. A motor may be disconnected or unpowered or at a different baudrate, or its ID could be incorrectly set.",
    11: "Motors not found. Motors may be unpowered or unplugged, or motors may have changed baudrate.",
    12: "Error during motor identification",
}

MAN_STEP                    = 11.25                         # Initial step size used during manual alignment 128*(360/4096)
ZIGZAG                      = 1                             # set to True to enable zigzag movement mode for 2D sweeps
DISPLAY_TEST_MENU           = 0                             # set to True to display special TEST menu as default
RELEASE_VER                 = 24.1                          # software release version
CONFIG                      = {}                            # global dictionary that contains variables saved to file
GIM_AUTOMOVE                = 0                             # set to True to only show GIM automove test mode

print("")
print("MilliBox Software Release: %0.1f" % (RELEASE_VER))   # display SW release version
print("Python Version: " + sys.version)                     # display Python version

config_fname = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mbx.cfg")      # look for mbx.cfg in cwd

if os.path.isfile(config_fname):                            # if mbx.cfg exists
    CONFIG = pickle.load(open(config_fname, "rb"))          # load the previous config variables
    if "DEVICENAME" in CONFIG.keys():
        DEVICENAME = CONFIG["DEVICENAME"]
    else:
        DEVICENAME = None
    if "BAUDRATE" in CONFIG.keys():
        BAUDRATE = CONFIG["BAUDRATE"]
    else:
        BAUDRATE = None
else:
    DEVICENAME = None
    BAUDRATE = None

p_num = None

if DEVICENAME and BAUDRATE:
    print("\n**** Previously connected to %s at baudrate %d. Attempting to re-connect... ****\n" % (DEVICENAME, BAUDRATE or -1))

while 1:                                                    # Loop until a motor is chosen
    if DEVICENAME:
        print("Connecting to %s at baudrate %d" % (DEVICENAME, BAUDRATE or -1))
    error_num = connect_detailed(DEVICENAME, BAUDRATE, p_num)        # initiate connection with motors, check communication and register settings
    if error_num == 0:
        print("Device connected")
        break
    elif error_num > 2:                                     # if connect failed because no devicename or baudrate, don't prompt user
        print("Connection Error %d: %s" % (error_num, ERROR_DICT.get(error_num, "Unknown Error")))
        print("Would you like to scan all ports? [Y/N]")
        key = None                                          # ask to scan ports or not
        while key not in ['Y', 'N']:
            key = chr(ord(getch().upper()))
            if key == 'N':
                print("Press any key to terminate...")      # terminate if user refuses
                getch()
                quit()
    else:
        print("\n**** Found no previous connection to a gimbal. Running autoscan. ****\n")

    if error_num != 0:
        port_list, error_list = port_scan()
        if len(port_list) > 1:
            print("Multiple possible GIM detected, selection required:")
            while 1:
                for i in range(len(port_list)):
                    print("* Enter <%d> for port %s at baudrate %d" % (i+1, port_list[i][0], port_list[i][1]))
                print("* Enter <0> to quit")
                result = int(input_num(": "))
                index = result - 1
                if result == 0:
                    print("Press any key to terminate...")  # terminate if quit selected
                    getch()
                    quit()
                elif 0 < result <= len(port_list):
                    DEVICENAME = port_list[index][0]
                    BAUDRATE = port_list[index][1]
                    p_num = port_list[index][2]
                    alt_baud = 0
                    if index > 0 and port_list[index - 1][0] == DEVICENAME:
                        alt_baud = port_list[index - 1][1]
                    elif len(port_list) > index + 1 and port_list[index + 1][0] == DEVICENAME:
                        alt_baud = port_list[index + 1][1]

                    if alt_baud:                            # if there are motors on same port but other baudrates
                        print("Some motors on this port are on a different baudrate. Would you like them to be on this baudrate? [Y/N]")
                        key = None                          # ask to change baud or not
                        while key not in ['Y', 'N']:
                            key = chr(ord(getch().upper()))
                            if key == 'Y':
                                print("Setting all motors to %d" % BAUDRATE)
                                baudrate_broadcast(DEVICENAME, alt_baud, BAUDRATE)
                                close()
                    break
                else:
                    print("\n That's not one of the options")
        elif len(port_list) == 1:
            DEVICENAME = port_list[0][0]
            BAUDRATE = port_list[0][1]
            p_num = port_list[0][2]
        else:
            print("Failed to find gimbal")
            for failed_port in error_list:
                print("%s at baudrate %d: %s" % (failed_port[0], failed_port[1], failed_port[2]))
            if len(error_list) == 0:
                print("U2D2 may not be plugged into computer")
            print("Press any key to terminate...")          # terminate if error communicating with motor
            getch()
            quit()
        print("\n**** CHOOSING PORT %s AT BAUDRATE %d ****\n" % (DEVICENAME, BAUDRATE))

gim_type = get_gimtype()                                    # get gimbal type - HV or SPHERICAL
if gim_type == HV:
    print("GIM TYPE = HV")
elif gim_type == SPHERICAL:
    print("GIM TYPE = SPHERICAL")

num_motors = get_nummotors()                                # get number of motors
print("Found %d motor(s)" % num_motors)

gim_motion = get_gim_motion()                               # get structure with gim motion control

gotoZERO()                                                  # start at HOME position

print("++++++++++++++++ MilliBox Gimbal is ready  +++++++++++++++")
print("")

# load config information from previous run (instrument setup, gim motion control)
config_fname = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mbx.cfg")      # look for mbx.cfg in cwd

if os.path.isfile(config_fname):                            # if mbx.cfg exists
    CONFIG = pickle.load(open(config_fname, "rb"))          # load the previous config variables
    if "meas_mode" in CONFIG.keys():
        meas_mode = CONFIG["meas_mode"]
    else:
        meas_mode = "NONE"
    if "addr" in CONFIG.keys():
        addr = CONFIG["addr"]
    else:
        addr = ["SIMULATION"]
    if "corr_fname" in CONFIG.keys():
        corr_fname = CONFIG["corr_fname"]
    else:
        corr_fname = ""
    if "corr_write" in CONFIG.keys():
        corr_write = CONFIG["corr_write"]
    else:
        corr_write = True
    if "gim_motion" in CONFIG.keys():                       # look for gim motion control parameters
        gim_motion_tmp = CONFIG["gim_motion"]
        if gim_motion_tmp["gim_type"] == gim_motion["gim_type"] and gim_motion_tmp["num_motors"] == gim_motion["num_motors"]:
            print("Restoring GIM motion parameter settings")        # check if previous parameters are for same type of gimbal
            set_gim_motion(gim_motion_tmp)                  # restore previous settings
            gim_motion = get_gim_motion()
            ACCURACY = gim_motion["accuracy"]
            print_gim_motion()
        else:                                               # parameters are for different type of gimbal
            print("GIM motion parameter mismatch! Using default values")
            set_gim_motion_default()                        # set to default gimbal motion parameters
            gim_motion = get_gim_motion()
            ACCURACY = gim_motion["accuracy"]
            print_gim_motion()
else:                                                       # set default values
    meas_mode = "NONE"                                      # default to NONE / SIMULATION]
    addr = ["SIMULATION"]                                   # addr is list of VISA addresses
    corr_fname = ""                                         # no corr factor
    corr_write = True                                       # save corr factor with data
    set_gim_motion_default()                                # set to default gimbal motion parameters
    gim_motion = get_gim_motion()
    ACCURACY = gim_motion["accuracy"]
    print("GIM motion parameter not defined! Using default values")
    print_gim_motion()

if corr_fname != "":
    proc.load_and_set_corr_file(corr_fname)                 # attempt to load previous corr factor file
if not proc.is_corr_on():                                   # if corr file could not load
    corr_fname = ""                                         # clear corr file for next run
proc.set_corr_write(corr_write)                             # restore state of corr_write

CONFIG["meas_mode"] = meas_mode                             # assign variables
CONFIG["addr"] = addr
CONFIG["corr_fname"] = corr_fname
CONFIG["corr_write"] = corr_write
CONFIG["gim_motion"] = gim_motion
CONFIG["DEVICENAME"] = DEVICENAME
CONFIG["BAUDRATE"] = BAUDRATE
fileObject = open(config_fname, 'wb')
pickle.dump(CONFIG, fileObject, 2)                          # store the variables in the file
fileObject.close()                                          # close the file
print("mbx.cfg saved to file")                              # use previous config on next run
ACCURACY = gim_motion["accuracy"]

# initalize VISA instrument control or set to use SIMULATION mode
inst = equip.inst_setup(meas_mode, addr)
inst.init_meas()

print("\n\n******* Measurement Mode = %s *******" % meas_mode)

while True:
    if gim_type == HV:                                      # get current position (H, V, P)
        ang1 = convertpostoangle(H, current_pos(H, 1))
        ang2 = convertpostoangle(V, current_pos(V, 1))
        ang3 = convertpostoangle(P, current_pos(P, 1))
    elif gim_type == SPHERICAL:                             # get current position (TH, PH, DPH)
        ang1 = convertpostoangle(TH, current_pos(TH, 1))
        ang2, ang3 = convertpostoangle(PH, current_pos(PH, 1))
    print("")
    print("**************************************************")
    print("*")
    print("*       Mode: %s" % meas_mode)
    print("* Instrument: %s" % inst.addr)
    if proc.is_corr_on():                                   # check if correction factor is applied
        corr_xp, corr_yp, corr_fname = proc.get_corr()
        corr_write = proc.get_corr_write()
        print("*  Corr File: %s" % corr_fname)              # display correction factor file name
        print("*  Save Corr: %s" % corr_write)              # display whether to save corr factor with data file
    else:
        print("*  Corr File: ---")
        print("*  Save Corr: ---")
    # print("*   Accuracy: %s" % ACCURACY)
    print("*     Zigzag: %r" % bool(ZIGZAG))
    if num_motors >= 4:
        print("*   Position: (%0.2f, %0.2f, %0.2f)" % (ang1, ang2, ang3))
    elif num_motors >= 2:
        print("*   Position: (%0.2f, %0.2f)" % (ang1, ang2))
    else:
        print("*   Position: (%0.2f)" % ang1)
    print("*")
    print("************* MAIN MENU **************************")
    print("************ USE KEYBOARD ************************")
    print("* Press <ESC> or <q> to close ports and quit!")

    print("__________ MOVEMENT/ALIGNMENT __________")
    if gim_type == HV:
        if num_motors >= 2:
            print("* use <arrow keys> or <ijkl> to adjust H and V angles")
        else:
            print("* use <arrow keys> or <jl> to adjust H angle")
        if num_motors >= 4:
            print("* use <[> or <]> to adjust polarization angle")
        print("* use <a> to reduce step size for finer alignment resolution")
        print("* use <s> to increase step size for coarser alignment resolution")
        if num_motors >= 4:
            print("* press <ZERO> key to store H0 V0 P0 position")
            print("* press <h> to go home to last saved H0 V0 P0 home position")
        elif num_motors >= 2:
            print("* press <ZERO> key to store H0 V0 position")
            print("* press <h> to go home to last saved H0 V0 home position")
        else:
            print("* press <ZERO> key to store H0 position")
            print("* press <h> to go home to last saved H0 home position")
    elif gim_type == SPHERICAL:
        if num_motors >= 5:
            print("* use <arrow keys> or <ijkl> to adjust THETA and PHI angles")
        if num_motors >= 6:
            print("* use <[> or <]> to adjust DELTA_PHI angle")
            print("* use <{> or <}> to adjust T MOTOR angle ONLY")
        print("* use <a> to reduce step size for finer alignment resolution")
        print("* use <s> to increase step size for coarser alignment resolution")
        if num_motors == 5:
            print("* press <ZERO> key to store TH0 PH0 position")
            print("* press <h> to go home to last saved TH0 PH0 home position")
        elif num_motors >= 6:
            print("* press <ZERO> key to store TH0 PH0 DPH0 position")
            print("* press <h> to go home to last saved TH0 PH0 DPH0 home position")
    print("* press <m> to open the direct move travel menu")
    print("* press <b> to start electronic beam alignment")
    print("* press <.> to print current position")

    print("_________ GIM MOTION SETTINGS __________")
    print("* press <g> to display GIM motion parameters")
    print("* press <v> to open the velocity setting menu")
    print("* press <r> to open the accel limit setting menu")
    print("* press <t> to open the angle limit setting menu")
    print("* press <e> to toggle accuracy setting")
    print("* press <w> to reset GIM parameters to default")

    print("__________ MEASUREMENT SETUP ___________")
    print("* press <y> to set measurement mode (VNA/SG+SA/SA/NONE) and set equipment VISA address")
    print("* press <x> to get current power measurement")
    print("* press <n> to save current power measurement to file")
    print("* press <o> to display correction factor menu")
    print("* press <f> to force instrument re-initialization")

    print("__________ MEASUREMENT SWEEPS __________")
    if gim_type == HV:
        if num_motors >= 2:
            print("* press <d> to start default plot H -90 90 V -90 90 Step 15deg ")    # 2D sweep for GIM01 or GIM03 or GIM04
        else:
            print("* press <d> to start default plot H -180 180 Step 10deg")    # single axis sweep for GIM1D
    elif gim_type == SPHERICAL:
        if num_motors >= 5:
            print("* press <d> to start default plot TH 0 90 PH -180 180 Step 15deg ")  # 2D spherical sweep for GIM05
    print("* press <c> to start accuracy position check menu ")
    print("* press <z> to toggle zigzag movement mode for 2D sweeps")
    print("* press <1> to start a 1-D sweep")
    if num_motors >= 2:                                                         # 2D sweep options available for all but GIM1D
        print("* press <2> to start a 2-D sweep")
        print("* press <3> to start a 1-D sweep in E-plane and H-plane")

    print("_________ DATA POST-PROCESSING _________")
    print("* press <p> to plot from previous measurement")
    print("* press </> to display post-processing menu")

    print("___________ SPECIAL SETTINGS ___________")
    print("* press <:> to change motor baudrate configuration")
    print("* press <+> to reset offset for home position")
    print("* press <\\> to toggle test menu")

    print("*********************************************************")

    if DISPLAY_TEST_MENU:
        if gim_type == HV:
            print("**********************  TEST MENU  **********************")
            print("* press <!> to show Gimbal platform")
            print("* press <@> to test Gimbal full range of motion")
            if not GIM_AUTOMOVE:
                if num_motors >= 2:
                    print("* press <#> to run a full Gimbal sweep H -180 180 V -180 180 Step 20deg")    # full 2D sweep for GIM01 or GIM03
                else:
                    print("* press <#> to run a full Gimbal sweep H -180 180 Step 5deg")                # full single axis sweep for GIM1D
            print("* press <$> to run Gimbal autonomous move")
            if not GIM_AUTOMOVE:
                if num_motors >= 2:
                    print("* press <%> to run Gimbal sweep H -40 40 V -40 40 Step 5deg - 2D heatmap plot")
                else:
                    print("* press <%> to run Gimbal sweep H -40 40 Step 1deg")
                # print("* press <^> to run CSV-file defined pattern sweep")
            print("*********************************************************")

        if gim_type == SPHERICAL:
            print("**********************  TEST MENU  **********************")
            print("* press <!> to show Gimbal platform")
            print("* press <@> to test Gimbal full range of motion")
            if not GIM_AUTOMOVE:
                if num_motors >= 5:
                    print("* press <#> to run a full Gimbal sweep TH -180 180 PH -180 180 Step 20deg")  # full 2D sweep for GIM05
            if num_motors >= 5:
                print("* press <$> to GIM05 motion loop")                       # Phi rotation for GIM05
            if not GIM_AUTOMOVE:
                print("* press <%> to run Gimbal sweep TH 0 40 PH -180 180 Step 10deg - 2D heatmap plot")
                # print("* press <^> to run CSV-file defined pattern sweep")
            print("*********************************************************")

    pressedkey = ord(getch())                                                   # get key press

    if pressedkey == 27 or pressedkey == ord("q"):                              # esc or "q": close port
        print("Quit program? [Y/N]")
        key = None                                                              # ask to disable torque on motors at close
        while key not in ['Y', 'N']:
            key = chr(ord(getch().upper()))
            if key == 'Y':
                gotoZERO(ACCURACY)                                              # park Gimbal home upon exiting controller

                print("")
                print("Disable TORQUE on motors? [Y/N]")
                key = None                                                      # ask to disable torque on motors at close
                while key not in ['Y', 'N']:
                    key = chr(ord(getch().upper()))
                    if key == 'Y':
                        print("disabling torque on all motors")
                        disable_torque(H)
                        if gim_type == HV:
                            if num_motors >= 2:
                                disable_torque(V)
                            # if num_motors >= 3:
                            #     disable_torque(R)
                            if num_motors >= 4:
                                disable_torque(P)
                        if gim_type == SPHERICAL:
                            if num_motors >= 5:
                                disable_torque(T)
                            if num_motors >= 6:
                                disable_torque(Z)

                print("")
                print("exit called bye bye!")
                inst.close_instrument()                                         # close all instruments
                exit()

    # ****************** MOVEMENT/ALIGNMENT ****************** #

    if pressedkey == 224:                                                       # check for arrow keys
        nextkey = ord(getch())
        if nextkey == 72:                                                       # up -> map to "i" for vertical up move
            pressedkey = ord("i")
        elif nextkey == 80:                                                     # down -> map to "k" for vertical down move
            pressedkey = ord("k")
        elif nextkey == 75:                                                     # left -> map to "j" for horizontal left move
            pressedkey = ord("j")
        elif nextkey == 77:                                                     # right -> map to "l" for horizontal right move
            pressedkey = ord("l")

    if pressedkey == ord("i"):                                                  # "i": vertical/phi up move
        if gim_type == HV:
            if num_motors >= 2:
                print("up")
                move_angle_rel(V, MAN_STEP, ACCURACY)
        if gim_type == SPHERICAL:
            if num_motors >= 5:
                print("phi up")
                move_angle_rel(PH, MAN_STEP, ACCURACY)

    elif pressedkey == ord("k"):                                                # "k": vertical/phi down move
        if gim_type == HV:
            if num_motors >= 2:
                print("down")
                move_angle_rel(V, MAN_STEP * -1, ACCURACY)
        if gim_type == SPHERICAL:
            if num_motors >= 5:
                print("phi down")
                move_angle_rel(PH, MAN_STEP * -1, ACCURACY)

    elif pressedkey == ord("j"):                                                # "j": horizontal/theta left move
        if gim_type == HV:
            print("left")
            move_angle_rel(H, MAN_STEP * -1, ACCURACY)
        if gim_type == SPHERICAL:
            print("theta down")
            move_angle_rel(TH, MAN_STEP * -1, ACCURACY)

    elif pressedkey == ord("l"):                                                # "l": horizontal/theta right move
        if gim_type == HV:
            print("right")
            move_angle_rel(H, MAN_STEP, ACCURACY)
        if gim_type == SPHERICAL:
            print("theta up")
            move_angle_rel(TH, MAN_STEP, ACCURACY)

    elif pressedkey == ord("["):                                                # "[": polarization/delta_phi left
        if gim_type == HV:
            if num_motors >= 4:
                print("polarization left")
                move_angle_rel(P, MAN_STEP * -1, ACCURACY)
        if gim_type == SPHERICAL:
            if num_motors >= 6:
                print("delta_phi down")
                move_angle_rel(DPH, MAN_STEP*-1, ACCURACY)

    elif pressedkey == ord("]"):                                                # "]": polarization/delta_phi right
        if gim_type == HV:
            if num_motors >= 4:
                print("polarization right")
                move_angle_rel(P, MAN_STEP, ACCURACY)
        if gim_type == SPHERICAL:
            if num_motors >= 6:
                print("delta_phi up")
                move_angle_rel(DPH, MAN_STEP, ACCURACY)

    elif pressedkey == ord("{"):                                                # "{": T MOTOR left
        if gim_type == SPHERICAL:
            if num_motors >= 6:
                print("T MOTOR down")
                move_angle_rel(T, MAN_STEP*-1, ACCURACY)

    elif pressedkey == ord("}"):                                                # "}": T motor right
        if gim_type == SPHERICAL:
            if num_motors >= 6:
                print("T MOTOR up")
                move_angle_rel(T, MAN_STEP, ACCURACY)

    elif pressedkey == ord("a"):                                                # "a": reduce step size by half
        print("a")
        MAN_STEP = MAN_STEP/2
        print("step size is now: %g" % MAN_STEP)

    elif pressedkey == ord("s"):                                                # "s": increase step size by two
        print("s")
        MAN_STEP = MAN_STEP*2
        print("step size is now: %g" % MAN_STEP)

    elif pressedkey == ord("0"):                                                # "0": write current position as home position
        print("Save position as new HOME? [Y/N]")
        key = None                                                              # ask if user wants to commit position as home
        while key not in ['Y', 'N']:
            key = chr(ord(getch().upper()))
            if key == 'Y':
                setoffset_all()
                gotoZERO(ACCURACY)
                print("this position is now HOME position")

    elif pressedkey == ord("h"):                                                # "h": go home
        print("h")
        print("go to 0 position")                                               # move to (0,0,0) and prints move time
        if gim_type == HV:
            gim_move(0, 0, 0, ACCURACY)
        elif gim_type == SPHERICAL:
            gim_move_sph(0, [0, 0], ACCURACY)

    elif pressedkey == ord("m"):                                                # "m": direct move menu
        print("m")
        if gim_type == HV:
            H_TARGET = float(input_num("Enter targeted angle in horizontal plane in degree: "))
            if num_motors >= 2:
                V_TARGET = float(input_num("Enter targeted angle in vertical plane in degree: "))
            else:
                V_TARGET = None
            if num_motors >= 4:
                P_TARGET = float(input_num("Enter targeted angle in polarization plane in degree: "))
            else:
                P_TARGET = None
            print("")

            if move_angle(hang=H_TARGET, vang=V_TARGET, pang=P_TARGET, checkonly=True):
                print("## Please make sure everything is ready to start measurement ##")  # warning
                print("#####      Automatic motion of MilliBox will start!!       ####")
                print("##   Press SPACE BAR when all is ready to start plotting     ##")
                if sys.platform == "win32":                                     # if we run windows, we can abort with <ESC>
                    print("##   Press ESC to abort                                      ##")
                key = None
                while key != 32 and key != 27:                                  # wait for space bar
                    key = ord(getch())
                    if key == 32:
                        gim_move(H_TARGET, V_TARGET, P_TARGET, ACCURACY)        # make the move
            else:
                print("*** Movement cancelled ***")

        if gim_type == SPHERICAL:
            THETA_TARGET = float(input_num("Enter targeted angle in THETA in degree: "))
            if num_motors >= 5:
                PHI_TARGET = float(input_num("Enter targeted angle in PHI in degree: "))
            else:
                PHI_TARGET = None
            if num_motors >= 6:
                DPHI_TARGET = float(input_num("Enter targeted angle in DELTA_PHI in degree: "))
            else:
                DPHI_TARGET = None
            print("")

            if move_angle(thang=THETA_TARGET, phang=[PHI_TARGET, DPHI_TARGET], checkonly=True):
                print("## Please make sure everything is ready to start measurement ##")  # warning
                print("#####      Automatic motion of MilliBox will start!!       ####")
                print("##   Press SPACE BAR when all is ready to start plotting     ##")
                if sys.platform == "win32":                                     # if we run windows, we can abort with <ESC>
                    print("##   Press ESC to abort                                      ##")
                key = None
                while key != 32 and key != 27:                                  # wait for space bar
                    key = ord(getch())
                    if key == 32:
                        gim_move_sph(THETA_TARGET, [PHI_TARGET, DPHI_TARGET], ACCURACY)  # make the move
            else:
                print("*** Movement cancelled ***")

    elif pressedkey == ord("b"):                                                # "b": electronic beam alignment
        print("b")
        print("##                 ELECTRONIC BEAM ALIGNMENT                 ##")# warning
        print("##                 -------------------------                 ##")# warning
        print("## Please make sure everything is ready to start measurement ##")# warning
        print("#####      Automatic motion of MilliBox will start!!       ####")
        print("##   Press SPACE BAR when all is ready to start plotting     ##")
        if sys.platform == "win32":                                             # if we run windows, we can abort with <ESC>
            print("##   Press ESC to abort                                      ##")
        key = None                                                              # block on space bar
        while key != 32 and key != 27:
            key = ord(getch())
            if key == 32:
                if gim_type == HV:
                    x1, x2 = beam_align_hv(inst, 0, ACCURACY)
                elif gim_type == SPHERICAL:
                    x1, x2 = beam_align_sph(inst, ACCURACY)
            else:
                x1 = x2 = None

        if x1 is not None and x2 is not None:                                   # if valid return values (alignment was not aborted)
            print("Save position as new HOME? [Y/N]")
            key = None                                                          # ask if user wants to commit position as home
            while key not in ['Y', 'N']:
                key = chr(ord(getch().upper()))
                if key == 'Y':
                    setoffset_all()
                    gotoZERO(ACCURACY)
                    print("this position is now HOME position")

    elif pressedkey == ord("."):                                                # ".": get motors current position
        print("")
        getposition()

    # ****************** GIM MOTION SETTINTS ****************** #

    elif pressedkey == ord("g"):                                                # "g": show GIM motion control parameters
        print("g")
        print_gim_motion()

    elif pressedkey == ord("v"):                                                # "v": set velocity
        print("v")                                                              # initiates rotational velocity menu
        vel1, vel2, vel3 = get_velocity(log_to_screen=True)

        if gim_type == HV:
            H_VEL = float(input_num("Enter your desired rotation velocity for H in RPM (0=maximum, BLANK=%g (no change)): " % vel1, vel1))
            if num_motors >= 2:
                V_VEL = float(input_num("Enter your desired rotation velocity for V in RPM: (0=maximum, BLANK=%g (no change)): " % vel2, vel2))
            else:
                V_VEL = 0
            if num_motors >= 4:
                P_VEL = float(input_num("Enter your desired rotation velocity for P in RPM: (0=maximum, BLANK=%g (no change)): " % vel3, vel3))
            else:
                P_VEL = 0
            set_velocity(H_VEL, V_VEL, P_VEL)

        if gim_type == SPHERICAL:
            H_VEL = float(input_num("Enter your desired rotation velocity for H in RPM (0=maximum, BLANK=%g (no change)): " % vel1, vel1))
            if num_motors >= 5:
                T_VEL = float(input_num("Enter your desired rotation velocity for T in RPM: (0=maximum, BLANK=%g (no change)): " % vel2, vel2))
            else:
                T_VEL = 0
            if num_motors >= 6:
                Z_VEL = float(input_num("Enter your desired rotation velocity for Z in RPM: (0=maximum, BLANK=%g (no change)): " % vel3, vel3))
            else:
                Z_VEL = 0
            set_velocity(H_VEL, T_VEL, Z_VEL)

        get_velocity(log_to_screen=True)

        gim_motion = get_gim_motion()
        CONFIG["gim_motion"] = gim_motion
        fileObject = open(config_fname, 'wb')
        pickle.dump(CONFIG, fileObject, 2)                                      # store the variables in the file
        fileObject.close()                                                      # close the file
        print("gimbal motion settings saved to mbx.cfg")                        # save measurement config to use on next run

    elif pressedkey == ord("r"):                                                # "r": set acceleration limit
        print("r")                                                              # initiates accel limit menu
        acc1, acc2, acc3 = get_accel(log_to_screen=True)

        if gim_type == HV:
            H_ACCEL = int(input_num("Enter your desired acceleration limit for H (0=set to default, BLANK=%g (no change)): " % acc1, acc1))
            if num_motors >= 2:
                V_ACCEL = int(input_num("Enter your desired acceleration limit for V (0=set to default, BLANK=%g (no change)): " % acc2, acc2))
            else:
                V_ACCEL = 0
            if num_motors >= 4:
                P_ACCEL = int(input_num("Enter your desired acceleration limit for P (0=set to default, BLANK=%g (no change)): " % acc3, acc3))
            else:
                P_ACCEL = 0
            set_accel(H_ACCEL, V_ACCEL, P_ACCEL)

        if gim_type == SPHERICAL:
            H_ACCEL = int(input_num("Enter your desired acceleration limit for H (0=set to default, BLANK=%g (no change)): " % acc1, acc1))
            if num_motors >= 5:
                T_ACCEL = int(input_num("Enter your desired acceleration limit for T: (0=set to default, BLANK=%g (no change)): " % acc2, acc2))
            else:
                T_ACCEL = 0
            if num_motors >= 6:
                Z_ACCEL = int(input_num("Enter your desired acceleration limit for Z: (0=set to default, BLANK=%g (no change)): " % acc3, acc3))
            else:
                Z_ACCEL = 0
            set_accel(H_ACCEL, T_ACCEL, Z_ACCEL)

        get_accel(log_to_screen=True)

        gim_motion = get_gim_motion()
        CONFIG["gim_motion"] = gim_motion
        fileObject = open(config_fname, 'wb')
        pickle.dump(CONFIG, fileObject, 2)                                      # store the variables in the file
        fileObject.close()                                                      # close the file
        print("gimbal motion settings saved to mbx.cfg")                        # save measurement config to use on next run

    elif pressedkey == ord("t"):                                                # "t": set angle limit
        print("t")                                                              # initiates angle limit menu
        anglelim1, anglelim2, anglelim3 = get_anglelim(log_to_screen=True)

        if gim_type == HV:
            print("")
            print("Current H angle limit = %s" % str(anglelim1))
            done = False
            while not done:
                low = float(input_num("Enter your desired lower angle limit for H (BLANK=%g (no change)): " % anglelim1[0], anglelim1[0]))
                high = float(input_num("Enter your desired upper angle limit for H (BLANK=%g (no change)): " % anglelim1[1], anglelim1[1]))
                H_ANGLELIM = list(np.sort(np.array([low, high])))
                done = set_anglelim(H_ANGLELIM, anglelim2, anglelim3)           # try to set new angle limits
                if done:
                    print("*** H angle limit set ***")
                else:
                    print("*** Invalid H angle limits. Try again ***")          # if not valid limits, try again
                    print("")
            anglelim1 = H_ANGLELIM

            if num_motors >= 2:
                print("")
                print("Current V angle limit = %s" % str(anglelim2))
                done = False
                while not done:
                    low = float(input_num("Enter your desired lower angle limit for V (BLANK=%g (no change)): " % anglelim2[0], anglelim2[0]))
                    high = float(input_num("Enter your desired upper angle limit for V (BLANK=%g (no change)): " % anglelim2[1], anglelim2[1]))
                    V_ANGLELIM = list(np.sort(np.array([low, high])))
                    done = set_anglelim(anglelim1, V_ANGLELIM, anglelim3)       # try to set new angle limits
                    if done:
                        print("*** V angle limit set ***")
                    else:
                        print("*** Invalid V angle limits. Try again ***")      # if not valid limits, try again
                        print("")
                anglelim2 = V_ANGLELIM

            if num_motors >= 4:
                print("")
                print("Current P angle limit = %s" % str(anglelim3))
                done = False
                while not done:
                    low = float(input_num("Enter your desired lower angle limit for P (BLANK=%g (no change)): " % anglelim3[0], anglelim3[0]))
                    high = float(input_num("Enter your desired upper angle limit for P (BLANK=%g (no change)): " % anglelim3[1], anglelim3[1]))
                    P_ANGLELIM = list(np.sort(np.array([low, high])))
                    done = set_anglelim(anglelim1, anglelim2, P_ANGLELIM)       # try to set new angle limits
                    if done:
                        print("*** P angle limit set ***")
                    else:
                        print("*** Invalid P angle limits. Try again ***")      # if not valid limits, try again
                        print("")
                anglelim3 = P_ANGLELIM

        if gim_type == SPHERICAL:
            print("")
            print("Current H angle limit = %s" % str(anglelim1))
            done = False
            while not done:
                low = float(input_num("Enter your desired lower angle limit for H (BLANK=%g (no change)): " % anglelim1[0], anglelim1[0]))
                high = float(input_num("Enter your desired upper angle limit for H (BLANK=%g (no change)): " % anglelim1[1], anglelim1[1]))
                H_ANGLELIM = list(np.sort(np.array([low, high])))
                done = set_anglelim(H_ANGLELIM, anglelim2, anglelim3)           # try to set new angle limits
                if done:
                    print("*** H angle limit set ***")
                else:
                    print("*** Invalid H angle limits. Try again ***")          # if not valid limits, try again
                    print("")
            anglelim1 = H_ANGLELIM

            if num_motors >= 5:
                print("")
                print("Current T angle limit = %s" % str(anglelim2))
                done = False
                while not done:
                    low = float(input_num("Enter your desired lower angle limit for T (BLANK=%g (no change)): " % anglelim2[0], anglelim2[0]))
                    high = float(input_num("Enter your desired upper angle limit for T (BLANK=%g (no change)): " % anglelim2[1], anglelim2[1]))
                    T_ANGLELIM = list(np.sort(np.array([low, high])))
                    done = set_anglelim(anglelim1, T_ANGLELIM, anglelim3)       # try to set new angle limits
                    if done:
                        print("*** T angle limit set ***")
                    else:
                        print("*** Invalid T angle limits. Try again ***")      # if not valid limits, try again
                        print("")
                anglelim2 = T_ANGLELIM

            if num_motors >= 6:
                done = False
                print("")
                print("Current Z angle limit = %s" % str(anglelim3))
                while not done:
                    low = float(input_num("Enter your desired lower angle limit for Z (BLANK=%g (no change)): " % anglelim3[0], anglelim3[0]))
                    high = float(input_num("Enter your desired upper angle limit for Z (BLANK=%g (no change)): " % anglelim3[1], anglelim3[1]))
                    Z_ANGLELIM = list(np.sort(np.array([low, high])))
                    done = set_anglelim(anglelim1, anglelim2, Z_ANGLELIM)       # try to set new angle limits
                    if done:
                        print("*** Z angle limit set ***")
                    else:
                        print("*** Invalid Z angle limits. Try again ***")      # if not valid limits, try again
                        print("")
                anglelim3 = Z_ANGLELIM

        get_anglelim(log_to_screen=True)

        gim_motion = get_gim_motion()
        CONFIG["gim_motion"] = gim_motion
        fileObject = open(config_fname, 'wb')
        pickle.dump(CONFIG, fileObject, 2)                                      # store the variables in the file
        fileObject.close()                                                      # close the file
        print("gimbal motion settings saved to mbx.cfg")                        # save measurement config to use on next run

    elif pressedkey == ord("e"):                                                # "e": toggle accuracy setting
        print("e")
        if ACCURACY == "HIGH":                                                  # HIGH -> VERY HIGH
            ACCURACY = "VERY HIGH"
        elif ACCURACY == "VERY HIGH":                                           # VERY HIGH -> HIGH
            ACCURACY = "HIGH"
        print("Gimbal accuracy setting = %s" % ACCURACY)

        set_accuracy(ACCURACY)                                                  # set accuracy parameter in gim_motion data structure
        print_gim_motion()

        gim_motion = get_gim_motion()
        CONFIG["gim_motion"] = gim_motion
        fileObject = open(config_fname, 'wb')
        pickle.dump(CONFIG, fileObject, 2)                                      # store the variables in the file
        fileObject.close()                                                      # close the file
        print("gimbal motion settings saved to mbx.cfg")                        # save measurement config to use on next run

    elif pressedkey == ord("w"):                                                # "w": set GIM motion control parameters to default
        print("w")
        print("Reset GIM motion control parameters to default? [Y/N]")
        key = None                                                              # ask if user wants to reset GIM motion control parameters
        while key not in ['Y', 'N']:
            key = chr(ord(getch().upper()))
            if key == 'Y':
                set_gim_motion_default()                                        # set gimbal motion control parameters to default
                print_gim_motion()

                gim_motion = get_gim_motion()
                ACCURACY = GIM_MOTION["accuracy"]
                CONFIG["gim_motion"] = gim_motion
                fileObject = open(config_fname, 'wb')
                pickle.dump(CONFIG, fileObject, 2)                              # store the variables in the file
                fileObject.close()                                              # close the file
                print("gimbal motion settings saved to mbx.cfg")                # save measurement config to use on next run
        print("Erase saved port and baudrate? [Y/N]")
        key = None                                                              # ask if user wants to erase saved port and baudrate
        while key not in ['Y', 'N']:
            key = chr(ord(getch().upper()))
            if key == 'Y':
                CONFIG["DEVICENAME"] = None
                CONFIG["BAUDRATE"] = None
                fileObject = open(config_fname, 'wb')
                pickle.dump(CONFIG, fileObject, 2)                              # store the empty variables in the file
                fileObject.close()                                              # close the file
                print("Saved port and baudrate erased. On next launch, all ports will be scanned.") # forget port and baudrate, causing a rescan


    # ****************** MEASUREMENT SETUP ****************** #

    elif pressedkey == ord("y"):                                                # "y": list and select VISA instrument
        print("y")
        (meas_mode, inst) = visa(meas_mode, inst)
        addr = inst.addr

        CONFIG["meas_mode"] = meas_mode
        CONFIG["addr"] = addr
        fileObject = open(config_fname, 'wb')
        pickle.dump(CONFIG, fileObject, 2)                                      # store the variables in the file
        fileObject.close()                                                      # close the file
        print("measurement mode and equipment saved to mbx.cfg")                # save measurement config to use on next run

    elif pressedkey == ord("x"):                                                # "x": readback motor position and power level
        print("x")
        wait_stop_moving(ACCURACY)

        sTitle = ""
        if gim_type == HV:
            if num_motors >= 4:
                sTitle = "power at (H,V,P) = (%0.1f,%0.1f,%0.1f):" % (convertpostoangle(H,current_pos(H,1)),convertpostoangle(V,current_pos(V,1)),convertpostoangle(P,current_pos(P,1)))
            elif num_motors >= 2:
                sTitle = "power at (H,V) = (%0.1f,%0.1f):" % (convertpostoangle(H,current_pos(H,1)),convertpostoangle(V,current_pos(V,1)))
            else:
                sTitle = "power at (H) = (%0.1f):" % (convertpostoangle(H,current_pos(H,1)))
            print(sTitle)
        elif gim_type == SPHERICAL:
            if num_motors >= 6:
                th = convertpostoangle(TH,current_pos(TH,1))
                ph,dphi = convertpostoangle(PH,current_pos(PH,1))
                sTitle = "power at (TH,PH,DPH) = (%0.1f,%0.1f,%0.1f):" % (th,ph,dphi)
                print(sTitle)

        val, freq = get_power(inst)                                             # get power values
        for i in range(len(val)):
            print("%7.2fGHz : %7.2f" % (freq[i]/1e9, val[i]))                   # print all freq/val if multiple points

        try:
            inst.cont_trigger()                                                 # set to cont sweep after power measurement
        except:
            pass

        if len(val) > 1:                                                        # check for VNA file and plot
            freq_unique = np.unique(freq)
            mult = int(float(len(val))/float(len(freq_unique)))                 # determine how many points per freq (e.g., mag+phase=2)
            freq_unique = freq[0::mult]
            val_db = val[0::mult]                                               # keep only the first point (mag_db)

            display_xyplot(np.array(freq_unique)/1e9, val_db, sTitle)

    elif pressedkey == ord("n"):                                                # "n": save measurement to file
        print("n")
        wait_stop_moving(ACCURACY)
        meas_to_file(inst)

    elif pressedkey == ord("o"):                                                # "o": correction factor menu
        print("o")
        proc.mbx_corr_menu(DISPLAY_TEST_MENU)

        corr_xp, corr_yp, corr_fname = proc.get_corr()
        corr_write = proc.get_corr_write()

        CONFIG["corr_fname"] = corr_fname                                       # store the corr factor in mbx.cfg
        CONFIG["corr_write"] = corr_write                                       # store whether to save corr factor in mbx.cfg
        fileObject = open(config_fname, 'wb')
        pickle.dump(CONFIG, fileObject, 2)                                      # store the variables in the file
        fileObject.close()                                                      # close the file
        print("corr factor filename saved to mbx.cfg")                          # save corr factor to use on next run

    elif pressedkey == ord("f"):                                                # force instrument re-initialization
        print("\nresetting instruments\n")
        if inst.port_open:
            inst.init_meas()

    # ****************** MEASUREMENT SWEEPS ****************** #

    elif pressedkey == ord("d"):                                                # "d": run default sweep
        print("d")
        gotoZERO(ACCURACY)                                                      # make sure millibox is reset to (0,0)
        print("## Please make sure everything is ready to start measurement ##")# warning
        print("#####      Automatic motion of MilliBox will start!!       ####")
        print("##   Press SPACE BAR when all is ready to start plotting     ##")
        if sys.platform == "win32":                                             # if we run windows, we can abort with <ESC>
            print("##   Press ESC to abort                                      ##")
        key = None                                                              # block on space bar
        while key != 32 and key != 27:
            key = ord(getch())
            if key == 32:
                if gim_type == HV:
                    if num_motors >= 4:
                        millibox_2dsweep(-90, 90, -90, 90, 15, 0, 1, 'default', inst, ACCURACY, zigzag=ZIGZAG)      # start default 2d sweep
                    elif num_motors >= 2:
                        millibox_2dsweep(-90, 90, -90, 90, 15, None, 1, 'default', inst, ACCURACY, zigzag=ZIGZAG)   # start default 2d sweep
                    else:
                        millibox_1dsweep('H', -180, 180, 0, 0, 10, None, 1, 'default', inst, ACCURACY)              # start default 1d sweep
                elif gim_type == SPHERICAL:
                    if num_motors >= 5:
                        millibox_2dsweep_sph(0, 90, -180, 180, 15, 0, 1, 'default', inst, ACCURACY, zigzag=ZIGZAG)  # start default 2d sweep

    elif pressedkey == ord("c"):                                                # "c": start accuracy plot menu
        print("c")
        gotoZERO(ACCURACY)
        check_ok = 0
        enter_params = True
        while enter_params and check_ok == 0:
            if gim_type == HV:
                MINH = float(input_num("Enter your start angle in horizontal plane in degree: "))
                MAXH = float(input_num("Enter your last angle in horizontal plane in degree: "))
                if num_motors >= 2:
                    MINV = float(input_num("Enter your start angle in vertical plane in degree: "))
                    MAXV = float(input_num("Enter your last angle in vertical plane in degree: "))
                else:
                    MINV = MAXV = 0
                STEP = float(input_num("Enter your step size in degree : "))    # capture user entries
                if num_motors >= 2:
                    check_ok = check_plot(MINH, MAXH, MINV, MAXV, STEP)
                else:
                    check_ok = check_plot_1d('H', MINH, MAXH, MINV, MAXV, STEP)
            elif gim_type == SPHERICAL:
                MINTH = float(input_num("Enter your start angle in THETA in degree: "))
                MAXTH = float(input_num("Enter your last angle in THETA in degree: "))
                MINPH = float(input_num("Enter your start angle in PHI in degree: "))
                MAXPH = float(input_num("Enter your last angle in PHI in degree: "))
                STEP = float(input_num("Enter your step size in degree : "))    # capture user entries
                check_ok = check_plot_sph(MINTH, MAXTH, MINPH, MAXPH, STEP)

            if check_ok == 1:
                print("## Please make sure everything is ready to start measurement ##")# warning
                print("#####      Automatic motion of MilliBox will start!!       ####")
                print("##   Press SPACE BAR when all is ready to start plotting     ##")
                if sys.platform == "win32":                                     # if we run windows, we can abort with <ESC>
                    print("##   Press ESC to abort                                      ##")
                key = None
                while key != 32 and key != 27:                                  # wait for space bar
                    key = ord(getch())
                    if key == 32:
                        if gim_type == HV:
                            milliboxacc(MINH, MAXH, MINV, MAXV, STEP, ACCURACY, zigzag=ZIGZAG)          # start position accuracy check with user inputs
                        elif gim_type == SPHERICAL:
                            milliboxacc_sph(MINTH, MAXTH, MINPH, MAXPH, STEP, ACCURACY, zigzag=ZIGZAG)  # start position accuracy check with user inputs
            else:
                print("\n\n#####################################################################")
                print("##    ERROR :  THOSE VALUES CAN'T PLOT, Please try other values    ##")
                print("#####################################################################\n\n")

                print("Re-enter sweep parameters? [Y/N]\n")
                keypress = None
                while keypress not in ['Y', 'N']:
                    keypress = chr(ord(getch().upper()))
                enter_params = (keypress == 'Y')

    elif pressedkey == ord("z"):                                                # "z": toggle zigzag movement mode
        print("z")
        ZIGZAG = not ZIGZAG
        print("Zigzag movement mode = %s" % ZIGZAG)

    elif pressedkey == ord("1"):                                                # "1": start 1-D sweep menu
        print("1")
        gotoZERO(ACCURACY)
        print("\n\n************ 1-D Single Direction Sweep ************\n")
        print("Plot display options:")
        print("  0 - no interactive plot")                                      # no graphic - save data to CSV file only
        print("  1 - interactive plot")                                         # line plot
        print("")
        PLOT = -1
        while PLOT not in [0,1]:
            PLOT = int(input_num("Select the plot display option: "))
        print("")

        if gim_type == HV:
            MINH = MAXH = MINV = MAXV = POLA = 0
            STEP = 10
            DIR = 'H'
            while DIR != 'Q' and check_plot_1d(DIR, MINH, MAXH, MINV, MAXV, STEP, POLA) == 0:       # loop until valid data is entered or QUIT
                if num_motors >= 2:
                    print("Select sweep direction:")
                    print("  H - horizontal sweep")
                    print("  V - vertical sweep")
                    print("  Q - quit and return to main menu")
                    print("")
                    DIR = None
                    while DIR not in ['H', 'V', 'Q']:
                        DIR = chr(ord(getch().upper()))
                else:
                    DIR = "H"
                if DIR == "H":
                    if num_motors >= 2:
                        MINV = float(input_num("Enter your FIXED angle in vertical plane in degree: "))
                    else:
                        MINV = 0.0
                    MAXV = MINV
                    MINH = float(input_num("Enter your start angle in horizontal plane in degree: "))
                    MAXH = float(input_num("Enter your last angle in horizontal plane in degree: "))
                    STEP = float(input_num("Enter your step size in degree : "))                    # capture user entries for H sweep
                    if num_motors >= 4:
                        POLA = float(input_num("Enter your polarization position in degree : "))
                    else:
                        POLA = None
                elif DIR == "V":
                    MINH = float(input_num("Enter your FIXED angle in horizontal plane in degree: "))
                    MAXH = MINH
                    MINV = float(input_num("Enter your start angle in vertical plane in degree: "))
                    MAXV = float(input_num("Enter your last angle in vertical plane in degree: "))
                    STEP = float(input_num("Enter your step size in degree : "))                    # capture user entries for V sweep
                    if num_motors >= 4:
                        POLA = float(input_num("Enter your polarization position in degree : "))
                    else:
                        POLA = None
                if DIR != 'Q' and check_plot_1d(DIR, MINH, MAXH, MINV, MAXV, STEP, POLA) == 0:
                    print("\n\n#####################################################################")
                    print("##    ERROR :  THOSE VALUES CAN'T PLOT, Please try other values    ##")
                    print("#####################################################################\n\n")

            if DIR != 'Q':
                print("")
                tag = six.moves.input("Enter a tag to append to filename or [ENTER] for no tag: ")
                print("")
                config.millibox_1dsweep_wrapper(DIR, MINH, MAXH, MINV, MAXV, STEP, POLA, PLOT, tag, inst, ACCURACY)     # start sweep with user inputs

        elif gim_type == SPHERICAL:
            MINTH = MAXTH = MINPH = MAXPH = DPHI = 0
            STEP = 10
            DIR = 'T'
            while DIR != 'Q' and check_plot_1d_sph(DIR, MINTH, MAXTH, MINPH, MAXPH, STEP, DPHI) == 0:   # loop until valid data is entered or QUIT
                if num_motors >= 5:
                    print("Select sweep direction:")
                    print("  T - theta sweep")
                    print("  P - phi sweep")
                    print("  Q - quit and return to main menu")
                    print("")
                    DIR = None
                    while DIR not in ['T', 'P', 'Q']:
                        DIR = chr(ord(getch().upper()))
                else:
                    DIR = "T"
                if DIR == "T":
                    if num_motors >= 5:
                        MINPH = float(input_num("Enter your FIXED angle for PHI in degree: "))
                    else:
                        MINPH = 0.0
                    MAXPH = MINPH
                    MINTH = float(input_num("Enter your start angle in THETA in degree: "))
                    MAXTH = float(input_num("Enter your last angle in THETA in degree: "))
                    STEP = float(input_num("Enter your step size in degree : "))                    # capture user entries for TH sweep
                    if num_motors >= 6:
                        DPHI = float(input_num("Enter your DELTA_PHI in degree : "))
                    else:
                        DPHI = None
                elif DIR == "P":
                    MINTH = float(input_num("Enter your FIXED angle in THETA in degree: "))
                    MAXTH = MINTH
                    MINPH = float(input_num("Enter your start angle in PHI in degree: "))
                    MAXPH = float(input_num("Enter your last angle in PHI in degree: "))
                    STEP = float(input_num("Enter your step size in degree : "))                    # capture user entries for PH sweep
                    if num_motors >= 6:
                        DPHI = float(input_num("Enter your DELTA_PHI in degree : "))
                    else:
                        DPHI = None
                if DIR != 'Q' and check_plot_1d_sph(DIR, MINTH, MAXTH, MINPH, MAXPH, STEP, DPHI) == 0:
                    print("\n\n#####################################################################")
                    print("##    ERROR :  THOSE VALUES CAN'T PLOT, Please try other values    ##")
                    print("#####################################################################\n\n")

            if DIR != 'Q':
                print("")
                tag = six.moves.input("Enter a tag to append to filename or [ENTER] for no tag: ")
                print("")
                config.millibox_1dsweep_wrapper_sph(DIR, MINTH, MAXTH, MINPH, MAXPH, STEP, DPHI, PLOT, tag, inst, ACCURACY)   # start sweep with user inputs

    elif pressedkey == ord("2") and num_motors >= 2:                            # "2": start 2-D sweep menu
        print("2")
        if gim_type == HV:
            gotoZERO(ACCURACY)
            print("This is your zero position: center of the plot")
            print("\n\n************ 2-Axis Sweep ************\n")
            print("Plot display options:")
            print("  0 - no interactive plot display")                          # no graphic - save data to CSV file only
            print("  1 - 3d surface plot")                                      # 3D surface plot + 3D radiation pattern
            print("  2 - 2d heatmap plot")                                      # 2D heatmap plot + 3D radiation pattern
            print("  3 - multi-trace line plot")                                # multi-trace line plot + 3D radiation pattern
            print("  4 - 3d radiation pattern ONLY")                            # 3D radiation pattern only
            print("")
            PLOT = -1
            while PLOT not in [0, 1, 2, 3, 4]:
                PLOT = int(input_num("Select the plot display option: "))
            print("")

            MINH = MAXH = MINV = MAXV = POLA = 0
            STEP = 10
            enter_params = True
            while enter_params and check_plot(MINH, MAXH, MINV, MAXV, STEP, POLA) == 0:             # loop until valid data is entered or QUIT
                MINH = float(input_num("Enter your start angle in horizontal plane in degree: "))
                MAXH = float(input_num("Enter your last angle in horizontal plane in degree: "))
                MINV = float(input_num("Enter your start angle in vertical plane in degree: "))
                MAXV = float(input_num("Enter your last angle in vertical plane in degree: "))
                STEP = float(input_num("Enter your step size in degree : "))                        # capture user entries
                if num_motors >= 4:
                    POLA = float(input_num("Enter your polarization position in degree : "))
                else:
                    POLA = None

                if check_plot(MINH, MAXH, MINV, MAXV, STEP, POLA) == 0:
                    print("\n\n#####################################################################")
                    print("##    ERROR :  THOSE VALUES CAN'T PLOT, Please try other values    ##")
                    print("#####################################################################\n\n")

                    print("Re-enter sweep parameters? [Y/N]\n")
                    keypress = None
                    while keypress not in ['Y', 'N']:
                        keypress = chr(ord(getch().upper()))
                    enter_params = (keypress == 'Y')

            if enter_params:
                print("")
                tag = six.moves.input("Enter a tag to append to filename or [ENTER] for no tag: ")
                print("")

                config.millibox_2dsweep_wrapper(MINH, MAXH, MINV, MAXV, STEP, POLA, PLOT, tag, inst, ACCURACY, zigzag=ZIGZAG)  # start plot with user inputs

        elif gim_type == SPHERICAL:
            gotoZERO(ACCURACY)
            print("This is your zero position: center of the plot")
            print("\n\n************ 2-Axis Sweep ************\n")
            print("Plot display options:")
            print("  0 - no interactive plot display")                          # no graphic - save data to CSV file only
            print("  1 - 2d direction cosine plot")                             # 2D direction cosine plot + 3D radiation pattern
            print("  2 - 2d polar spherical plot")                              # 2D polar spherical plot + 3D radiation pattern
            print("  3 - 3d radiation pattern ONLY")                            # 3D radiation pattern only
            print("")
            PLOT = -1
            while PLOT not in [0, 1, 2, 3]:
                PLOT = int(input_num("Select the plot display option: "))
            print("")

            MINTH = MAXTH = MINPH = MAXPH = DPHI = 0
            STEP = 10
            enter_params = True
            while enter_params and check_plot_sph(MINTH, MAXTH, MINPH, MAXPH, STEP, DPHI) == 0:     # loop until valid data is entered or QUIT
                MINTH = float(input_num("Enter your start angle in THETA in degree: "))
                MAXTH = float(input_num("Enter your last angle in THETA in degree: "))
                MINPH = float(input_num("Enter your start angle in PHI in degree: "))
                MAXPH = float(input_num("Enter your last angle in PHI in degree: "))
                STEP = float(input_num("Enter your step size in degree : "))                        # capture user entries
                if num_motors >= 6:
                    DPHI = float(input_num("Enter your DELTA_PHI in degree : "))
                else:
                    DPHI = None

                if check_plot_sph(MINTH, MAXTH, MINPH, MAXPH, STEP, DPHI) == 0:
                    print("\n\n#####################################################################")
                    print("##    ERROR :  THOSE VALUES CAN'T PLOT, Please try other values    ##")
                    print("#####################################################################\n\n")

                    print("Re-enter sweep parameters? [Y/N]\n")
                    keypress = None
                    while keypress not in ['Y', 'N']:
                        keypress = chr(ord(getch().upper()))
                    enter_params = (keypress == 'Y')

            if enter_params:
                print("")
                tag = six.moves.input("Enter a tag to append to filename or [ENTER] for no tag: ")
                print("")

                config.millibox_2dsweep_wrapper_sph(MINTH, MAXTH, MINPH, MAXPH, STEP, DPHI, PLOT, tag, inst, ACCURACY, zigzag=ZIGZAG)  # start plot with user inputs

    elif pressedkey == ord("3") and num_motors >= 2:                            # "3": start 1-D sweep menu for E- and H-plane
        print("3")
        gotoZERO(ACCURACY)
        print("\n\n************ 1-D Single Direction Sweep in E-plane and H-plane ************\n")
        print("Plot display options:")
        print("  0 - no interactive plot")                                      # no graphic - save data to CSV file only
        print("  1 - interactive plot")                                         # line plot
        print("")
        PLOT = -1
        while PLOT not in [0,1]:
            PLOT = int(input_num("Select the plot display option: "))
        print("")

        if gim_type == HV:
            MIN = MAX = POLA = 0
            STEP = 10
            enter_params = True
            while enter_params and check_plot(MIN, MAX, MIN, MAX, STEP, POLA) == 0:                 # loop until valid data is entered or QUIT
                MIN = float(input_num("Enter your start angle in degree: "))
                MAX = float(input_num("Enter your last angle in degree: "))
                STEP = float(input_num("Enter your step size in degree : "))                        # capture user entries
                if num_motors >= 4:
                    POLA = float(input_num("Enter your polarization position in degree : "))
                else:
                    POLA = None
                if check_plot(MIN, MAX, MIN, MAX, STEP, POLA) == 0:
                    print("\n\n#####################################################################")
                    print("##    ERROR :  THOSE VALUES CAN'T PLOT, Please try other values    ##")
                    print("#####################################################################\n\n")

                    print("Re-enter sweep parameters? [Y/N]\n")
                    keypress = None
                    while keypress not in ['Y', 'N']:
                        keypress = chr(ord(getch().upper()))
                    enter_params = (keypress == 'Y')

            if enter_params:
                print("")
                tag = six.moves.input("Enter a tag to append to filename or [ENTER] for no tag: ")
                print("")

                config.millibox_hvsweep_wrapper(MIN, MAX, STEP, POLA, PLOT, tag, inst, ACCURACY)    # start sweep with user inputs

        if gim_type == SPHERICAL:
            MIN = MAX = DPHI = 0
            STEP = 10
            enter_params = True
            while enter_params and check_plot_sph(MIN, MAX, 0, 90, STEP, DPHI) == 0:                # loop until valid data is entered or QUIT
                MIN = float(input_num("Enter your start angle in degree: "))
                MAX = float(input_num("Enter your last angle in degree: "))
                STEP = float(input_num("Enter your step size in degree : "))  # capture user entries
                if num_motors >= 6:
                    DPHI = float(input_num("Enter your DELTA_PHI in degree : "))
                else:
                    DPHI = None
                if check_plot_sph(MIN, MAX, 0, 90, STEP, DPHI) == 0:
                    print("\n\n#####################################################################")
                    print("##    ERROR :  THOSE VALUES CAN'T PLOT, Please try other values    ##")
                    print("#####################################################################\n\n")

                    print("Re-enter sweep parameters? [Y/N]\n")
                    keypress = None
                    while keypress not in ['Y', 'N']:
                        keypress = chr(ord(getch().upper()))
                    enter_params = (keypress == 'Y')

            if enter_params:
                print("")
                tag = six.moves.input("Enter a tag to append to filename or [ENTER] for no tag: ")
                print("")

                config.millibox_hvsweep_wrapper_sph(MIN, MAX, STEP, DPHI, PLOT, tag, inst, ACCURACY)  # start plot with user inputs

    # ****************** DATA POST-PROCESSING ****************** #

    elif pressedkey == ord("p"):                                                # "p": plot from file
        print("p")
        mbx_plot(DISPLAY_TEST_MENU)

    elif pressedkey == ord("/"):                                                # "/": post-processing menu
        print("/")
        proc.mbx_postproc_menu(DISPLAY_TEST_MENU)

    # ****************** SPECIAL SETTINGS ****************** #

    elif pressedkey == ord(":"):                                                # ":": change baudrate
        print(":")
        print("### This menu changes the motor communication baudrate ####")
        print("### proceed with caution as you may loose communication with motors ####")
        print("### the mbx.cfg will be edited to the new baudrate ####")
        BRATE = int(input_num("Enter (1) for 57600 kbps, enter (3) for 1 MBps, any other number to exit: "))
        if (BRATE == 1) or (BRATE == 3):
            if BRATE == 1: CONFIG["BAUDRATE"] = 57600
            if BRATE == 3: CONFIG["BAUDRATE"] = 1000000
            fileObject = open(config_fname, 'wb')
            pickle.dump(CONFIG, fileObject, 2)                                  # store the variables in the file
            fileObject.close()  # close the file
            changerate(BRATE)

    elif pressedkey == ord("+"):                                                # "+": reset the offset for home position
        print("+")
        print(" WARNING you need to have the gimbal close to its home position to proceed")
        print("press any key to go back to main menu and SPACE BAR to proceed with Offset reset")
        key = ord(getch())
        if key == 32:
            resetoffset()
        else:
            print("back top menu")

    elif pressedkey == ord("\\"):                                               # "\": toggle test menu display
        print("\\")
        DISPLAY_TEST_MENU = not DISPLAY_TEST_MENU
        print("DISPLAY_TEST_MENU = %r" % DISPLAY_TEST_MENU)


    # test modes
    elif DISPLAY_TEST_MENU:
        if pressedkey == ord("!"):                                              # Shift-1: test mode, move to -110deg angle to show platform
            if gim_type == HV:
                move_angle(hang=-110, vang=0, pang=0)
            else:
                move_angle(thang=-110, phang=[0, 0])

        elif pressedkey == ord("@"):                                            # Shift-2: test mode, demonstrate range of gimbal
            print("")
            print("## Please make sure everything is ready to start measurement ##")# warning
            print("#####      Automatic motion of MilliBox will start!!       ####")
            print("##   Press SPACE BAR when all is ready to start plotting     ##")
            if sys.platform == "win32":                                         # if we run windows, we can abort with <ESC>
                print("##   Press ESC to abort                                      ##")
            key = None                                                          # block on space bar
            print("")
            while key != 32 and key != 27:
                key = ord(getch())
                if key == 32:
                    if gim_type == HV:
                        gim_motion = get_gim_motion()
                        gotoZERO()
                        move_angle(hang=gim_motion[1]["anglelim"][0])
                        move_angle(hang=gim_motion[1]["anglelim"][1])
                        move_angle(hang=0)
                        if num_motors >= 2:
                            move_angle(vang=gim_motion[2]["anglelim"][0])
                            move_angle(vang=gim_motion[2]["anglelim"][1])
                            move_angle(vang=0)
                            if num_motors >= 4:
                                move_angle(pang=gim_motion[4]["anglelim"][0])
                                move_angle(pang=gim_motion[4]["anglelim"][1])
                                move_angle(pang=0)
                        gotoZERO()
                    elif gim_type == SPHERICAL:
                        gim_motion = get_gim_motion()
                        gotoZERO()
                        move_angle(thang=gim_motion[1]["anglelim"][0])
                        move_angle(thang=gim_motion[1]["anglelim"][1])
                        move_angle(thang=0)

                        if num_motors >= 5:
                            move_angle(tang=gim_motion[5]["anglelim"][0])
                            move_angle(tang=gim_motion[5]["anglelim"][1])
                            move_angle(tang=0)

                        if num_motors >= 6:
                            move_angle(zang=gim_motion[6]["anglelim"][0])
                            move_angle(zang=gim_motion[6]["anglelim"][1])
                            move_angle(zang=0)
                        gotoZERO()

        elif pressedkey == ord("#") and not GIM_AUTOMOVE:                       # Shift-3: test mode, full sweep -180 180 -180 180 20
            ok = 1
            if gim_type == HV:
                if num_motors >= 2 and check_plot(-180, 180, -180, 180, 20, 0) == 0:
                    ok = 0
                if num_motors == 1 and check_plot(-180, 180, 0, 0, 5, None) == 0:
                    ok = 0
            if gim_type == SPHERICAL:
                if num_motors >= 5 and check_plot_sph(-180, 180, -180, 180, 20, None) == 0:
                    ok = 0
            if not ok:
                print("\n\n#####################################################################")
                print("##    ERROR :  Current ANGLE LIMITS do not allow full sweep.       ##")
                print("#####################################################################\n\n")
                print_gim_motion()
            else:
                gotoZERO(ACCURACY)                                              # make sure millibox is reset to (0,0)
                print("")
                print("## Please make sure everything is ready to start measurement ##")# warning
                print("#####      Automatic motion of MilliBox will start!!       ####")
                print("##   Press SPACE BAR when all is ready to start plotting     ##")
                if sys.platform == "win32":                                     # if we run windows, we can abort with <ESC>
                    print("##   Press ESC to abort                                      ##")
                key = None                                                      # block on space bar
                while key != 32 and key != 27:
                    key = ord(getch())
                    if key == 32:
                        if gim_type == HV:
                            if num_motors >= 4:
                                millibox_2dsweep(-180, 180, -180, 180, 20, 0, 1, 'full', inst, ACCURACY, zigzag=ZIGZAG)     # start full 2d sweep
                            elif num_motors >= 2:
                                millibox_2dsweep(-180, 180, -180, 180, 20, None, 1, 'full', inst, ACCURACY, zigzag=ZIGZAG)  # start full 2d sweep
                            else:
                                millibox_1dsweep('H', -180, 180, 0, 0, 5, None, 1, 'full', inst, ACCURACY)                  # start full 1d sweep
                        elif gim_type == SPHERICAL:
                            if num_motors >= 5:
                                millibox_2dsweep_sph(-180, 180, -180, 180, 20, 0, 2, 'full', inst, ACCURACY, zigzag=ZIGZAG) # start full 2d sweep

        elif pressedkey == ord("$"):                                            # Shift-4: autonomous gimbal move
            if gim_type == HV:
                gim_motion = get_gim_motion()
                ok = 1
                if gim_motion[1]["anglelim"][0] > -180 or gim_motion[1]["anglelim"][1] < 180:
                    ok = 0
                if num_motors >= 2 and (gim_motion[2]["anglelim"][0] > -180 or gim_motion[2]["anglelim"][1] < 180):
                    ok = 0
                if num_motors >= 4 and (gim_motion[4]["anglelim"][0] > -90 or gim_motion[4]["anglelim"][1] < 90):
                    ok = 0
                if not ok:
                    print("\n\n#####################################################################")
                    print("##    ERROR :  Current ANGLE LIMITS do not allow full sweep.       ##")
                    print("#####################################################################\n\n")
                    print_gim_motion()
                else:
                    print("")
                    gotoZERO(ACCURACY)                                          # make sure millibox is reset to (0,0)
                    h_vel, v_vel, p_vel = get_velocity()
                    set_velocity(3, 3, 3/2)
                    print("")
                    print("## Please make sure everything is ready to start measurement ##")  # warning
                    print("#####      Automatic motion of MilliBox will start!!       ####")
                    print("##        Press SPACE BAR when all is ready to start         ##")
                    if sys.platform == "win32":                                 # if we run windows, we can abort with <ESC>
                        print("##        Press ESC to abort                                 ##")
                    key = None                                                  # block on space bar
                    while key != 32 and key != 27:
                        key = ord(getch())
                        if key == 32:
                            H_list = [-180, 180]
                            V_list = [-180, 180]
                            if num_motors >= 4:
                                P_list = [-90, +90]
                            else:
                                P_list = [0, 0]
                            idx = 0
                            done = False
                            while not done:
                                if not check_is_moving(False):
                                    time.sleep(0.5)
                                    idx = (idx + 1) % len(H_list)
                                    h_pos = convertangletopos(H, H_list[idx])
                                    v_pos = convertangletopos(V, V_list[idx])
                                    p_pos = convertangletopos(P, P_list[idx])

                                    # move but do not wait
                                    move_pos(H, h_pos)                          # go to final H position
                                    if num_motors >= 2:
                                        move_pos(V, v_pos)                      # go to final V position
                                        if num_motors >= 4:
                                            move_pos(P, p_pos)                  # go to final P position

                                if kbhit():                                     # check for abort
                                    if check_abort():
                                        done = True

                                time.sleep(0.2)

                    set_velocity(h_vel, v_vel, p_vel)
                    gotoZERO(ACCURACY)

            elif gim_type == SPHERICAL:                                         # GIM05 motion loop
                if num_motors >= 5:
                    move_angle(thang=90, phang=[90, 0])
                    time.sleep(0.1)
                    move_angle(thang=-90, phang=[-90, 0])
                    time.sleep(0.1)
                    gotoZERO()

        elif pressedkey == ord("%") and not GIM_AUTOMOVE:                       # Shift-5: test mode, zoomed in accurate demo
            ok = 1
            if gim_type == HV:
                if num_motors >= 2 and check_plot(-40, 40, -40, 40, 5, 0) == 0:
                    ok = 0
                if num_motors == 1 and check_plot(-40, 40, 0, 0, 1, None) == 0:
                    ok = 0
            if gim_type == SPHERICAL:
                if num_motors >= 5 and check_plot_sph(0, 40, -180, 180, 10, None) == 0:
                    ok = 0
            if not ok:
                print("\n\n#####################################################################")
                print("##    ERROR :  Current ANGLE LIMITS do not allow zoom sweep.       ##")
                print("#####################################################################\n\n")
                print_gim_motion()
            else:
                gotoZERO(ACCURACY)                                              # make sure millibox is reset to (0,0)
                print("## Please make sure everything is ready to start measurement ##")# warning
                print("#####      Automatic motion of MilliBox will start!!       ####")
                print("##   Press SPACE BAR when all is ready to start plotting     ##")
                if sys.platform == "win32":                                     # if we run windows, we can abort with <ESC>
                    print("##   Press ESC to abort                                      ##")
                key = None                                                      # block on space bar
                while key != 32 and key != 27:
                    key = ord(getch())
                    if key == 32:
                        if gim_type == HV:
                            if num_motors >= 4:
                                millibox_2dsweep(-40, 40, -40, 40, 5, 0, 2, 'zoom', inst, ACCURACY, zigzag=ZIGZAG)      # start zoom in accurate sweep, 2D heatmap plot
                            elif num_motors >= 2:
                                millibox_2dsweep(-40, 40, -40, 40, 5, None, 2, 'zoom', inst, ACCURACY, zigzag=ZIGZAG)   # start zoom in accurate sweep, 2D heatmap plot
                            else:
                                millibox_1dsweep('H', -40, 40, 0, 0, 1, None, 1, 'zoom', inst, ACCURACY)                # start zoom in accurate 1d sweep
                        elif gim_type == SPHERICAL:
                            if num_motors >= 5:
                                millibox_2dsweep_sph(0, 40, -180, 180, 10, 0, 1, 'zoom', inst, ACCURACY, zigzag=ZIGZAG)     # start zoom in accurate sweep, 2D heatmap plot

        # elif pressedkey == ord("^") and not GIM_AUTOMOVE:                       # Shift-6: test mode, CSV-file defined pattern
        #     gotoZERO(ACCURACY)                                                  # make sure millibox is reset to (0,0)
        #     print("")
        #     pat_file = six.moves.input("Enter CSV filename with Gimbal coordinates or [ENTER] for default (gimpat.csv): ")
        #     print("")
        #     if pat_file == '':
        #         pat_file = 'gimpat.csv'
        #     tag = six.moves.input("Enter a tag to append to filename or [ENTER] for no tag: ")
        #     print("")
        #     print("## Please make sure everything is ready to start measurement ##")# warning
        #     print("#####      Automatic motion of MilliBox will start!!       ####")
        #     print("##   Press SPACE BAR when all is ready to start plotting     ##")
        #     if sys.platform == "win32":                                         # if we run windows, we can abort with <ESC>
        #         print("##   Press ESC to abort                                      ##")
        #     key = None                                                          # block on space bar
        #     print("")
        #     while key != 32 and key != 27:
        #         key = ord(getch())
        #         if key == 32:
        #             millibox_pat_sweep(pat_file, tag, inst, ACCURACY)
