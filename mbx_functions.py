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


################################################################################
# Copyright 2017 ROBOTIS CO., LTD.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################
# Author: Ryu Woon Jung (Leon) ROBOTIS

# *********     MILLIBOX FUNCTIONS      *********
#   Gimbal registers
#   Gimbal setup functions
#   Gimbal movement
#   Equipment selection and measurement
#   Sweep functions
#   Deprecated functions
#

# IMPORTS
from __future__ import division                             # division compatibility Python 2.7 and Python 3.6+
import os
import sys
import csv
import datetime
import time
import six

from mbx_realtimeplot import *
import numpy as np
import matplotlib.pyplot as plt
import mbx_instrument as equip
import mbx_postprocess as proc

if "--oldsdk" in sys.argv:
    USE_OLD_SDK = True
    import dynamixel_functions as dynamixel
    print("Using Old SDK")
else:
    USE_OLD_SDK = False
    import dynamixel_sdk as dynamixel                       # Uses Dynamixel SDK library

import serial.tools.list_ports

if sys.platform == "win32":                                 # if we run windows, we can use getch from OS
    from msvcrt import getch, kbhit
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

    def kbhit():
        return False

if sys.platform == "darwin":
    MACOS = True
else:
    MACOS = False

# Python 2.7 doesn't have int.from_bytes, so this compatibility layer is added
if six.PY2:
    def from_bytes(bytes, byteorder='big', signed=False):                       # https://docs.python.org/3/library/stdtypes.html#int.from_bytes
        if byteorder == 'little':
            little_ordered = list(bytes)
        elif byteorder == 'big':
            little_ordered = list(reversed(bytes))
        else:
            raise ValueError("byteorder must be either 'little' or 'big'")

        n = sum(b << i * 8 for i, b in enumerate(little_ordered))
        if signed and little_ordered and (little_ordered[-1] & 0x80):
            n -= 1 << 8 * len(little_ordered)

        return n
else:
    def from_bytes(bytes, byteorder, signed=False):
        return int.from_bytes(bytes, byteorder=byteorder, signed=signed)


# Register set for MX64AT
# EEPROM SPACE               ADDRESS                       SIZE     R/W    DEFAULT    DESCRIPTION
ADD_MODEL_NUMBER                 = 0                           # 	2   R      311        Model Number for MX64AT(2.0)
ADD_MODEL                        = 2                           #    4   R      -          Model Information
ADD_FW_VER                       = 6                           #    1   R      -          Firmware Version  should stay 41
ADD_MOTOR_ID                     = 7                           #    1 	RW     1          ID 	DYNAMIXEL ID set H to 1 V to 2
ADD_BAUD_RATE                    = 8                           #    1   RW     1          Communication Baud Rate 	 SET to 3 -> 1Mbps
ADD_RETURN_TIME                  = 9                           #    1   RW     250        Return Delay Time
ADD_DRIVE_MODE                   = 10                          #    1   RW     0          Drive Mode
ADD_OPERATING_MODE               = 11                          #    1   RW     3          Operating Mode
ADD_SEC_ID                       = 12                          #    1   RW     255        Secondary(Shadow) ID
ADD_PROTOCOL                     = 13                          #    1   RW     2          Protocol Version
ADD_HOME_OFFSET                  = 20                          #    4   RW     0          Homing Offset
ADD_MOVE_THRESHOLD               = 24                          #    4   RW     10         Velocity Threshold for Movement Detection
ADD_TEMP_LIMIT                   = 31                          #    1   RW     80         Temperature Limit in degree C
ADD_VOLT_MAX_LIMIT               = 32                          #    2 	RW     160        Maximum Input Voltage Limit
ADD_VOLT_MIN_LIMIT               = 34                          #    2 	RW     95         Minimum Input Voltage Limit
ADD_PWM_LIMIT                    = 36                          #    2 	RW     885        Maximum PWM Limit
ADD_MAX_CURRENT                  = 38                          #    2 	RW     1941       Maximum Current Limit
ADD_MAX_ACCEL                    = 40                          #    4 	RW     32767      Maximum Acceleration Limit
ADD_MAX_VELOCITY                 = 44                          #    4 	RW     435        Maximum Velocity Limit
ADD_MAX_POS                      = 48                          #    4 	RW     4095       Maximum Position Limit
ADD_MIN_POS                      = 52                          #    4 	RW     0          Minimum Position Limit
ADD_ERROR_INFO                   = 63                          #    1 	RW     52         Shutdown Error Information

# RAM  SPACE                 ADDRESS                       SIZE     R/W    DEFAULT    DESCRIPTION
ADD_TORQUE_ENABLE                = 64                          #    1 	RW     0          Motor Torque On/Off
ADD_LED                          = 65                          #    1 	RW     0          Status LED On/Off
ADD_STATUS                       = 68                          #    1 	RW     2          Select Types of Status Return
ADD_REG_WRITE_FLAG               = 69                          #    1 	R      0          REG_WRITE Instruction Flag
ADD_HW_ERROR                     = 70                          #    1 	R      0          Hardware Error Status
ADD_VEL_I_GAIN                   = 76                          #    2 	RW     1920       I Gain of Velocity
ADD_VEL_P_GAIN                   = 78                          #    2 	RW     100        P Gain of Velocity
ADD_POS_D_GAIN                   = 80                          #    2 	RW     0          D Gain of Position
ADD_POS_I_GAIN                   = 82                          #    2 	RW     0          I Gain of Position
ADD_POS_P_GAIN                   = 84                          #    2 	RW     850        P Gain of Position
ADD_FF_2ND_GAIN                  = 88                          #    2 	RW     0          2nd Gain of Feed-Forward
ADD_FF_1ST_GAIN                  = 90                          #    2 	RW     0          1st Gain of Feed-Forward
ADD_WATCHDOG                     = 98                          #    1 	RW     0          Dynamixel BUS Watchdog
ADD_GOAL_PWM                     = 100                         #    2 	RW     -          Target PWM Value
ADD_GOAL_CURRENT                 = 102                         #    2 	RW     -          Target Current Value
ADD_GOAL_VELOC                   = 104                         #    4 	RW     -          Target Velocity Value
ADD_ACCEL_PROFILE                = 108                         #    4 	RW     0          Acceleration Value of Profile
ADD_VELOC_PROFILE                = 112                         #    4   RW     0          Velocity Value of Profile
ADD_GOAL_POSITION                = 116                         #    4 	RW     0          Target Position
ADD_TIME_TICK                    = 120 	                       #    2 	R      -          Count Time in Millisecond
ADD_MOVING                       = 122                         #    1 	R      0          Moving 	Movement Flag
ADD_MOVING_STATUS                = 123 	                       #    1 	R      0          Detailed Information of Movement Status
ADD_PRESENT_PWM                  = 124                         #    2 	R      -          Present PWM Value
ADD_PRESENT_CURR                 = 126                         #    2 	R      -          Present Current Value
ADD_PRESENT_VELOC                = 128                         #    4 	R      -          Present Velocity Value
ADD_PRESENT_POS                  = 132 	                       #    4 	R      -          Present Position Value
ADD_VELOC_TRAJ                   = 136                         #    4 	R      -          Target Velocity Trajectory from Profile
ADD_POS_TRAJ                     = 140                         #    4 	R      -          Target Position Trajectory from Profile
ADD_PRESENT_VOLT                 = 144                         #    2 	R      -          Present Input Voltage
ADD_PRESENT_TEMP                 = 146 	                       #    1   R      -          Present Internal Temperature


# SET GLOBALS
PROTOCOL_VERSION            = 2                         # See which protocol version is used in the Dynamixel
H                           = 1                         # Horizontal Motor is ID: 1
V                           = 2                         # Vertical Motor  is ID: 2
R                           = 3                         # Reverse Vertical motor is ID: 3 in case of GIM03/GIM04
P                           = 4                         # Polarization motor is ID: 4
T                           = 5                         # Phi rotation on gimbal side (GIM05)
Z                           = 6                         # DPhi rotation on horn post side (GIM05)
TH                          = 101                       # "Virtual" theta motor - mapped to H motor
PH                          = 105                       # "Virtual" phi motor - mapped to T/Z motors
DPH                         = 106                       # "Virtual" dphi motor - mapped to T/Z motor

HV                          = 1                         # gim_type 1 is HV coordinate
SPHERICAL                   = 5                         # gim_type 5 is spherical coordinate

XH_MOTOR_CLASS              = 1000                      # XH series motors - minimum model number
PH_MOTOR_CLASS              = 2000                      # PH series motors - minimum model number

MX64AT_MOTOR_TYPE           = 311                       # MX64AT(2.0) series motor
XH540_W150_MOTOR_TYPE       = 1110                      # XH540-W150 series motor (W=12V, 150=high speed)
XH540_W270_MOTOR_TYPE       = 1100                      # XH540-W270 series motor (W=12V, 270=high torque)
XH540_V270_MOTOR_TYPE       = 1140                      # XH540-V270 series motor (V=24V, 270=high torque)
XH540_V150_MOTOR_TYPE       = 1150                      # XH540-V150 series motor (V=24V, 150=high speed)
PH42_020_MOTOR_TYPE         = 2000                      # PH42-020-S300-R series motor

PH42_RAM_ADD_OFFSET         = 448                       # RAM address offset for PH42 motor series

XH540_MIN_VERSION           = 41                        # Minimum FW version supported (XH540 motor)
PH42_MIN_VERSION            = 11                        # Minimum FW version supported (PH42 motor)

OPER_MODE                   = 4                         # Operating mode set to multiturn
TORQUE_ENABLE               = 1                         # Value for enabling the torque
TORQUE_DISABLE              = 0                         # Value for disabling the torque
ESC_ASCII_VALUE             = 0x1b
COMM_SUCCESS                = 0                         # Communication Success result value
COMM_TX_FAIL                = -1001                     # Communication Tx Failed

DRIVE_MODE_NORM             = 0                         # standard drive mode
DRIVE_MODE_V_G6             = 1                         # reverse mode for GIM06 V motor
DRIVE_MODE_R                = 3                         # slave reverse mode for GIM03 reserse vertical motor
DRIVE_MODE_Z                = 1                         # reverse mode for GIM05 horn post motor

XH540_0                     = 2048                      # center position relative to offset (XH540 motor)
PH42_0                      = 0                         # center position relative to offset (PH42 motor)
XH540_MAX_OFFSET            = 1044479                   # +/- 255 revolutions is the max supported offset (XH540 motor)
PH42_MAX_OFFSET             = 2147483647                # +/- ~3500 revolutions is the max supported offset (PH42 motor)
XH540_RES                   = 4096.0                    # XH540 series resolution
PH42_RES                    = 607500.0                  # PH42 series resolution

XH540_H_RATIO               = 5                         # gear ratio for XH540 motor in H
PH42_H_RATIO                = 2                         # gear ratio for PH42 motor in H
XH540_V_RATIO               = 1                         # gear ratio for XH540 motor in V
XH540_P_RATIO               = 120/50                    # gear ratio for XH540 motor in P
XH540_T_RATIO               = 120/50                    # gear ratio for XH540 motor in T
XH540_Z_RATIO               = 2                         # gear ratio for XH540 motor in Z

ACCURACY_DEFAULT            = "HIGH"                    # set to HIGH or VERY HIGH for default gimbal positional accuracy

XH540_MOVING_THRESHOLD      = 0                         # resolution for movement detection (XH540)
PH42_MOVING_THRESHOLD       = 5                         # resolution for movement detection (PH42)
XH540_POS_ACC_THRESH        = 1                         # settled position threshold for HIGH or VERY HIGH accuracy (1/4096) - XH540 motor
PH42_POS_ACC_THRESH         = 40                        # settled position threshold for HIGH or VERY HIGH accuracy (40/607500) - PH42 motor
OVERSHOOT_ANG               = 2                         # in VERY HIGH accuracy mode, overshoot by 2deg
# DEBUG_MOVING                = 1					        # flag to debug movement with extra print statements
DEBUG_MOVING                = 0					        # flag to debug movement with extra print statements

XH540_POSITION_D_G          = 0                         # XH540 D gain
XH540_POSITION_I_G          = 3000                      # XH540 I gain
XH540_POSITION_P_G          = 1000                      # XH540 P gain
XH540_FF1_G                 = 0                         # XH540 feed forward gain

PH42_POSITION_D_G           = 0                         # PH42 Horizontal D gain
PH42_POSITION_I_G           = 3000                      # PH42 Horizontal I gain
PH42_POSITION_P_G           = 3000                      # PH42 Horizontal P gain
PH42_FF1_G                  = 0                         # PH42 Horizontal feed forward gain

XH540_MAX_PROFILE_VELOCITY  = 1023                      # Maximum rotation speed (XH540 motor)
PH42_MAX_PROFILE_VELOCITY   = 2920                      # Maximum rotation speed (PH42 motor)
XH540_VELOCITY_UNIT         = 0.229                     # XH540 series velocity unit (0.229 rev/min)
PH42_VELOCITY_UNIT          = 0.01                      # PH42 series velocity unit (0.01 rev/min)

PH42_H_ACCEL_LIMIT_DEF      = 2500                      # Default accel limit (PH42 H motor)
XH540_H_ACCEL_LIMIT_DEF_G1  = 40                        # Default accel limit (XH540 H motor - GIM01/GIM06)
XH540_H_ACCEL_LIMIT_DEF_G4  = 20                        # Default accel limit (XH540 H motor - GIM03/GIM04)
XH540_V_ACCEL_LIMIT_DEF_G1  = 40                        # Default accel limit (XH540 V motor - GIM01/GIM06)
XH540_V_ACCEL_LIMIT_DEF_G4  = 10                        # Default accel limit (XH540 V motor - GIM03/GIM04)
XH540_P_ACCEL_LIMIT_DEF     = 20                        # Default accel limit (XH540 P motor)
XH540_T_ACCEL_LIMIT_DEF     = 20                        # Default accel limit (XH540 T motor)
XH540_Z_ACCEL_LIMIT_DEF     = 20                        # Default accel limit (XH540 Z motor)

PH42_H_ACCEL_HARD_LIMIT     = [1, 5000]                 # Min/max for accel limit setting (PH42 H motor)
XH540_H_ACCEL_HARD_LIMIT    = [1, 100]                  # Min/max for accel limit setting (XH540 H motor)
XH540_V_ACCEL_G1_HARD_LIMIT = [1, 100]                  # Min/max for accel limit setting (XH540 V motor - GIM01)
XH540_V_ACCEL_G4_HARD_LIMIT = [1, 100]                  # Min/max for accel limit setting (XH540 V motor - GIM03/GIM04)
XH540_P_ACCEL_HARD_LIMIT    = [1, 100]                  # Min/max for accel limit setting (XH540 P motor)
XH540_T_ACCEL_HARD_LIMIT    = [1, 100]                  # Min/max for accel limit setting (XH540 T motor)
XH540_Z_ACCEL_HARD_LIMIT    = [1, 100]                  # Min/max for accel limit setting (XH540 Z motor)

H_ANGLE_HARD_LIMIT          = [-60, 60]               # Min/max for angle limit setting (H motor)
V_ANGLE_HARD_LIMIT          = [-60, 60]               # Min/max for angle limit setting (V motor)
P_ANGLE_HARD_LIMIT          = [-180, 180]               # Min/max for angle limit setting (P motor)
T_ANGLE_HARD_LIMIT          = [-180, 180]               # Min/max for angle limit setting (T motor)
Z_ANGLE_HARD_LIMIT          = [-180, 180]               # Min/max for angle limit setting (Z motor)

DEPRECATED_WARNING          = 1                         # print warning when using deprecated functions

num_motors                  = 2                         # number of gimbal motors (defaults to GIM01 = 2 motors (H&V))
port                        = None
port_num                    = None

# placeholder values (GIM01 base) - these globals will be overwritten during initialiation if GIM04 base is found
base_type                   = 1                         # base_type = 1 for XH540 motor and base type is 4 for PH42 motor
base_ratio                  = XH540_H_RATIO
base_res                    = XH540_RES
ram_offset                  = 0
h_zero                      = XH540_0
base_pos_acc_thresh         = XH540_POS_ACC_THRESH
base_vel_unit               = XH540_VELOCITY_UNIT
max_H_velocity              = XH540_MAX_PROFILE_VELOCITY
h_P                         = XH540_POSITION_P_G
h_I                         = XH540_POSITION_I_G
h_D                         = XH540_POSITION_D_G
h_FF1                       = XH540_FF1_G
base_moving_threshold       = XH540_MOVING_THRESHOLD
max_offset                  = XH540_MAX_OFFSET
gim_type                    = 0

# structure containing gimbal movement settings
GIM_MOTION                  = {"gim_type": 0,
                               "num_motors": 0,
                               "accuracy": ACCURACY_DEFAULT}

# Initialize PacketHandler Structs
dxl_comm_result = COMM_TX_FAIL                          # Communication result
dxl_error = 0                                           # Dynamixel error

# List of motors used by MBX
MOTOR_DICT = {
    MX64AT_MOTOR_TYPE: "MX64AT",
    XH540_W150_MOTOR_TYPE: "XH540_W150",
    XH540_W270_MOTOR_TYPE: "XH540_W270",
    XH540_V270_MOTOR_TYPE: "XH540_V270",
    XH540_V150_MOTOR_TYPE: "XH540_V150",
    PH42_020_MOTOR_TYPE: "PH42_020",
    }

BAUDRATE_LIST = [1000000, 57600] # List of baudrates used by MBX
ID_LIST = [H, V, R, P, T, Z, 101, 102, 103, 104, 105, 106, 107, 108, 109]
WARNING_DICT = {
    21: "Could not connect to port. U2D2 may be disconnected from USB port, or the port may be in use by another process.",
    22: "Could not set baudrate. U2D2 may be disconnected, or unknown error",
    24: "One or more of the motors connected is not recognized. For safety, this is prevented from running",
    25: "Motors not found, motors may be unpowered or unplugged",
    26: "OSX does not support baudrates over 230400",
    27: "This port has a LIN connected to it. Use the LIN controller to control the LIN.",
}


# ============================================
# ============= GIMBAL REGISTERS =============
# ============================================

def write1(motor, address, value):
    """ generic function to write 1 Byte to motor register """
    if USE_OLD_SDK:
        write1old(motor, address, value)
    else:
        write1new(motor, address, value)


def write2(motor, address, value):
    """ generic function to write 2 Byte to motor register """
    if USE_OLD_SDK:
        write2old(motor, address, value)
    else:
        write2new(motor, address, value)


def write4(motor, address, value):
    """ generic function to write 4 Byte to motor register """
    if USE_OLD_SDK:
        write4old(motor, address, value)
    else:
        write4new(motor, address, value)


def read1(motor, address, quiet=False):
    """ generic function to read 1 Byte from motor register """
    if USE_OLD_SDK:
        return read1old(motor, address, quiet)
    else:
        return read1new(motor, address, quiet)


def read2(motor, address, quiet=False):
    """ generic function to read 2 Byte from motor register """
    if USE_OLD_SDK:
        return read2old(motor, address, quiet)
    else:
        return read2new(motor, address, quiet)


def read4(motor, address, quiet=False):
    """ generic function to read 4 Byte from motor register """
    if USE_OLD_SDK:
        return read4old(motor, address, quiet)
    else:
        return read4new(motor, address, quiet)


def write1new(motor, address, value):
    """ generic function to write 1 Byte to motor register """
    packetHandler = dynamixel.PacketHandler(PROTOCOL_VERSION)
    (dxl_comm_result, dxl_error) = packetHandler.write1ByteTxRx(port, motor, address, value)
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))
    # else:                                                                      # debug
    #     print("Motor " + str(motor) + " write sucessful")
    return


def write2new(motor, address, value):
    """ generic function to write 2 Byte to motor register """
    packetHandler = dynamixel.PacketHandler(PROTOCOL_VERSION)
    (dxl_comm_result, dxl_error) = packetHandler.write2ByteTxRx(port, motor, address, value)
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))
    # else:                                                                      # debug
    #     print("Motor " + str(motor) + " write sucessful")
    return


def write4new(motor, address, value):
    """ generic function to write 4 Byte to motor register """
    packetHandler = dynamixel.PacketHandler(PROTOCOL_VERSION)
    (dxl_comm_result, dxl_error) = packetHandler.write4ByteTxRx(port, motor, address, value)
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))
    # else:                                                                      # debug
    #     print("Motor " + str(motor) + " write sucessful")
    return


def read1new(motor, address, quiet=False):
    """ generic function to read 1 Byte from motor register """
    packetHandler = dynamixel.PacketHandler(PROTOCOL_VERSION)
    (data, dxl_comm_result, dxl_error) = packetHandler.readTxRx(port, motor, address, 1)
    read = from_bytes(data, byteorder='little', signed=True)
    #print("address " + str(address) + " read 1 result " + str(read))
    if dxl_comm_result != COMM_SUCCESS:
        if not quiet:
            print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        if not quiet:
            print(packetHandler.getRxPacketError(dxl_error))
    # else:
    #     if not quiet:
    #         print("Motor " + str(motor) + " read sucessfull")
    return read


def read2new(motor, address, quiet=False):
    """ generic function to read 2 Byte from motor register """
    packetHandler = dynamixel.PacketHandler(PROTOCOL_VERSION)
    (data, dxl_comm_result, dxl_error) = packetHandler.readTxRx(port, motor, address, 2) # Read data as bytes
    read = from_bytes(data, byteorder='little', signed=True) # Convert bytes to signed int
    #print("address " + str(address) + " read 2 result " + str(read))
    if dxl_comm_result != COMM_SUCCESS:
        if not quiet:
            print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        if not quiet:
            print(packetHandler.getRxPacketError(dxl_error))
    # else:
    #     if not quiet:
    #         print("Motor " + str(motor) + " read sucessfull")
    return read


def read4new(motor, address, quiet=False):
    """ generic function to read 4 Byte from motor register """
    packetHandler = dynamixel.PacketHandler(PROTOCOL_VERSION)
    (data, dxl_comm_result, dxl_error) = packetHandler.readTxRx(port, motor, address, 4) # Read data as bytes
    read = from_bytes(data, byteorder='little', signed=True) # Convert bytes to signed int
    # print("address " + str(address) + " read 4 result " + str(read))
    if dxl_comm_result != COMM_SUCCESS:
        if not quiet:
            print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        if not quiet:
            print(packetHandler.getRxPacketError(dxl_error))
    # else:
    #     if not quiet:
    #         print("Motor " + str(motor) + " read sucessfull")
    return read


def write1old(motor, address, value):
    """ generic function to write 1 Byte to motor register """
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, motor, address, value)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
    # else:                                                                      # debug
    #     print("Motor " + str(motor) + " write sucessful")
    return


def write2old(motor, address, value):
    """ generic function to write 2 Byte to motor register """
    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, motor, address, value)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
    # else:                                                                      # debug
    #     print("Motor " + str(motor) + " write sucessful")
    return


def write4old(motor, address, value):
    """ generic function to write 4 Byte to motor register """
    dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, motor, address, value)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
    # else:                                                                      # debug
    #     print("Motor " + str(motor) + " write sucessful")
    return


def read1old(motor, address, quiet=False):
    """ generic function to read 1 Byte from motor register """
    read = dynamixel.read1ByteTxRx(port_num, PROTOCOL_VERSION, motor, address)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        if not quiet:
            print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        if not quiet:
            print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
    # else:
    #     if not quiet:
    #         print("Motor " + str(motor) + " read sucessful")
    return read


def read2old(motor, address, quiet=False):
    """ generic function to read 2 Byte from motor register """
    read = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, motor, address)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        if not quiet:
            print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        if not quiet:
            print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
    # else:
    #     if not quiet:
    #         print("Motor " + str(motor) + " read sucessfull")
    return read


def read4old(motor, address, quiet=False):
    """ generic function to read 4 Byte from motor register """
    read = dynamixel.read4ByteTxRx(port_num, PROTOCOL_VERSION, motor, address)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        if not quiet:
            print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        if not quiet:
            print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
    # else:
    #     if not quiet:
    #         print("Motor " + str(motor) + " read sucessful")
    return read


def close():
    """ close com port and menu """
    if USE_OLD_SDK:
        dynamixel.closePort(port_num)
    else:
        if port.is_open:
            port.closePort()
    return


# ==================================================
# ============= GIMBAL SETUP FUNCTIONS =============
# ==================================================

def connect(DEVICENAME, BAUDRATE):
    if connect_detailed(DEVICENAME, BAUDRATE) == 0:
        return True
    else:
        return False

def connect_detailed(DEVICENAME, BAUDRATE, p_num=None):
    """ initiate communication with motors, check communication """
    if USE_OLD_SDK:
        return connectold(DEVICENAME, BAUDRATE, p_num)
    else:
        return connectnew(DEVICENAME, BAUDRATE, p_num)


def connectold(DEVICENAME, BAUDRATE, p_num=None):
    """ initiate communication with motors, check communication """
    global port_num
    if DEVICENAME is None:
        return 1
    if BAUDRATE is None:
        return 2

    if p_num is None:                                                           # p_num makes it possible to avoid calling portHandler again
        port_num = dynamixel.portHandler(DEVICENAME.encode('utf-8'))
    else:
        port_num = p_num
    print("device %s  port %s" % (DEVICENAME, port_num))

    dynamixel.packetHandler()
    if dynamixel.openPort(port_num):
        print("Succeeded to open the port (%s)!" % DEVICENAME)
    else:
        print("Failed to open the port (%s)!" % DEVICENAME)
        return 3

    if dynamixel.setBaudRate(port_num, BAUDRATE):
        print("Succeeded to set the baudrate to %d!" % BAUDRATE)
    else:
        print("Failed to change the baudrate to %d!" % BAUDRATE)
        return 4

    status_ok = test()                                                          # test register configuration
    if status_ok > 0:
        close()
    return status_ok


def connectnew(DEVICENAME, BAUDRATE, p_num=None):
    """ initiate communication with motors, check communication """
    global port
    if DEVICENAME is None:
        return 1
    if BAUDRATE is None:
        return 2

    if p_num is None:                                                           # p_num makes it possible to avoid calling portHandler again
        port = dynamixel.PortHandler(DEVICENAME)
    else:
        port = p_num
    print("device %s  port %s" % (DEVICENAME, port.getPortName()))

    dynamixel.PacketHandler(PROTOCOL_VERSION)
    try:
        port.openPort()                                                         # In the new sdk, this will never return false, only crash
        print("Succeeded to open the port (%s)!" % DEVICENAME)
    except serial.serialutil.SerialException as _:
        print("Failed to open the port (%s)!" % DEVICENAME)
        return 3

    if port.setBaudRate(BAUDRATE):                                              # Set port baudrate
        print("Succeeded to set the baudrate to %d!" % BAUDRATE)
    else:
        print("Failed to set the baudrate to %d!" % BAUDRATE)
        return 4

    status_ok = test()                                                          # test register configuration
    if status_ok > 0:
        close()
    return status_ok


def port_scan(quiet = True):
    """ Get a list of every port, check if each port has a GIM attached to it """
    if USE_OLD_SDK:
        return port_scan_old(quiet)
    else:
        return port_scan_new(quiet)


def port_scan_old(quiet = True):
    """ Get a list of every port, check if each port has a GIM attached to it """

    global port_num

    print("Checking all comports")
    serial_port_list = serial.tools.list_ports.comports()
    valid_port_list = []
    warning_list = []
    for p in serial_port_list:
        if p.vid == 1027 and p.pid == 24596:                                    # FTDI vendor ID, FT232H product ID
            print("\nPort %s is a FT232H (U2D2)" % p.device)
            port_num = dynamixel.portHandler(p.device.encode('utf-8'))
            dynamixel.packetHandler()
            for baudrate in BAUDRATE_LIST:
                print("\nScanning at baudrate %d bps" % baudrate)
                warn_num = port_test_old(p, baudrate, quiet)
                close()
                if warn_num == 0:
                    print("==> Port %s is a GIM on baudrate %d" % (p.device, baudrate))
                    valid_port_list.append((p.device, baudrate, port_num))      # valid_port_list is an array of tuple (devicename, baudrate, port_num)
                else:
                    print("==> Port %s is not a GIM on baudrate %d, error %d" % (p.device, baudrate, warn_num))
                    if not quiet: print(WARNING_DICT.get(warn_num,""))
                    warning_list.append((p.device, baudrate, WARNING_DICT.get(warn_num,"")))
        else:
            print("\n\nPort %s is not a FT232H (U2D2)" % p.device)
    return valid_port_list, warning_list                                        # warning_List is an array of tuple (devicename, baudrate, warning string)


def port_scan_new(quiet = True):
    """ Get a list of every port, check if each port has a GIM attached to it """

    global port

    print("\nChecking all comports...")
    serial_port_list = serial.tools.list_ports.comports()
    valid_port_list = []
    warning_list = []
    for p in serial_port_list:
        if p.vid == 1027 and p.pid == 24596:                                    # FTDI vendor ID, FT232H product ID
            print("\nPort %s is a FT232H (U2D2)" % p.device)
            port = dynamixel.PortHandler(p.device)
            print("Checking all supported baudrates...")
            for baudrate in BAUDRATE_LIST:
                print("\nScanning at baudrate %d bps" % baudrate)
                warn_num = port_test_new(p, baudrate, quiet)
                close()
                if warn_num == 0:
                    print("==> Port %s is a GIM on baudrate %d" % (p.device, baudrate))
                    valid_port_list.append((p.device, baudrate, port))          # valid_port_list is an array of tuple (devicename, baudrate, port obj)
                else:
                    print("==> Port %s is not a GIM on baudrate %d, error %d" % (p.device, baudrate, warn_num))
                    if not quiet: print(WARNING_DICT.get(warn_num,""))
                    warning_list.append((p.device, baudrate, WARNING_DICT.get(warn_num,"")))
        else:
            print("\n\nPort %s is not a FT232H (U2D2)" % p.device)
    print("\nDone scanning all comports...")
    return valid_port_list, warning_list                                        # warning_List is an array of tuple (devicename, baudrate, warning string)


def port_test_old(serial_port, BAUDRATE, quiet):
    """ for a given port and baudrate, check if the motor on that port has a model number on the motor list """
    global port_num
    # This assumes the port is already connected to, to avoid dynamixel messages about connecting to the same port multiple times

    if MACOS and BAUDRATE > 230400:
        if not quiet: print("Baudrate too large for OSX")
        return 26

    if dynamixel.openPort(port_num):
        print("Succeeded to open the port (%s)!" % serial_port.device)
    else:
        if not quiet: print("Failed to open the port (%s)!" % serial_port.device)
        return 21

    if dynamixel.setBaudRate(port_num, BAUDRATE):
        print("Succeeded to set the baudrate to %d!" % BAUDRATE)
    else:
        if not quiet: print("Failed to set the baudrate to %d!" % BAUDRATE)
        return 22

    motor_found = False
    for ID in ID_LIST:
        model_number = read2(ID, ADD_MODEL_NUMBER, quiet=True)
        if model_number in MOTOR_DICT:
            print("%s identified at ID %d" % (MOTOR_DICT[model_number], ID))
            motor_found = True
        elif model_number == 0:
            if not quiet: print("Failed to connect to motor at ID %d at baudrate %d" % (ID, BAUDRATE))
        elif model_number == 1010:                                              # Model number for MBX LIN
            if not quiet: print("LIN found on %d at baudrate %d" % (ID, BAUDRATE))
            return 27
        else:
            if not quiet: print("Unsupported motor at id %d at baudrate %d" % (ID, BAUDRATE))
            return 24                                                           # Exit function here because this problem cannot be accounted for

    if motor_found:
        return 0
    else:
        if not quiet: print("Failed to find any motors at baudrate %d" % BAUDRATE)
        return 25


def port_test_new(serial_port, BAUDRATE, quiet):
    """ for a given port and baudrate, check if the motor on that port has a model number on the motor list """
    global port
    # This function assumes the port is already connected to, to avoid dynamixel messages about connecting to the same port multiple times

    if MACOS and BAUDRATE > 230400:
        if not quiet: print("Baudrate too large for OSX")
        return 26

    try:
        port.openPort()
        print("Succeeded to open the port (%s)!" % serial_port.device)
    except serial.serialutil.SerialException as _:
        if not quiet: print("Failed to open the port (%s)!" % serial_port.device)
        return 21

    if port.setBaudRate(BAUDRATE):
        print("Succeeded to set the baudrate to %d!" % BAUDRATE)
    else:
        if not quiet: print("Failed to set the baudrate to %d!" % BAUDRATE)
        return 22

    motor_found = False
    for ID in ID_LIST:
        model_number = read2(ID, ADD_MODEL_NUMBER, quiet=True)
        if model_number in MOTOR_DICT:
            print("%s identified at ID %d" % (MOTOR_DICT[model_number], ID))
            motor_found = True
        elif model_number == 0:
            if not quiet: print("Failed to connect to motor at ID %d at baudrate %d" % (ID, BAUDRATE))
        elif model_number == 1010:                                              # Model number for MBX LIN
            if not quiet: print("LIN found on %d at baudrate %d" % (ID, BAUDRATE))
            return 27
        else:
            if not quiet: print("Unsupported motor at id %d at baudrate %d" % (ID, BAUDRATE))
            return 24                                                           # Exit function here because this problem cannot be accounted for

    if motor_found:
        return 0
    else:
        if not quiet: print("Failed to find any motors at baudrate %d" % BAUDRATE)
        return 25


def baudrate_broadcast(DEVICENAME, old_baudrate, new_baudrate):
    """For a given devicename and baudrate, send a baudrate change message to all motors on that baudrate."""
    if USE_OLD_SDK:
        baudrate_broadcast_old(DEVICENAME, old_baudrate, new_baudrate)
    else:
        baudrate_broadcast_new(DEVICENAME, old_baudrate, new_baudrate)


def baudrate_broadcast_old(DEVICENAME, old_baudrate, new_baudrate):
    """For a given devicename and baudrate, send a baudrate change message to all motors on that baudrate."""
    global port_num
    port_num = dynamixel.portHandler(DEVICENAME.encode('utf-8'))
    dynamixel.packetHandler()

    if dynamixel.openPort(port_num):
        print("Succeeded to open the port (%s)!" % DEVICENAME)
    else:
        print("Failed to open the port (%s)!" % DEVICENAME)
        return 3

    if dynamixel.setBaudRate(port_num, old_baudrate):
        print("Succeeded to connect to baudrate %d!" % old_baudrate)
    else:
        print("Failed to connect to baudrate %d!" % old_baudrate)
        return 4

    if new_baudrate == 57600:
        rate_id = 1
    elif new_baudrate == 1000000:
        rate_id = 3
    else:
        print("Unsupported baudrate, using 1000000")
        rate_id = 3

    write1(254, ADD_TORQUE_ENABLE, TORQUE_DISABLE)
    write1(254, ADD_BAUD_RATE, rate_id)
    # This code can't actually confirm that the baudrate was changed
    print("Baudrate successfully changed to %d. Reconnecting." % new_baudrate)


def baudrate_broadcast_new(DEVICENAME, old_baudrate, new_baudrate):
    """For a given devicename and baudrate, send a baudrate change message to all motors on that baudrate."""
    global port
    port = dynamixel.PortHandler(DEVICENAME)

    try:
        port.openPort()
        print("Succeeded to open the port (%s)!" % DEVICENAME)
    except serial.serialutil.SerialException as _:
        print("Failed to open the port (%s)!" % DEVICENAME)
        return 3

    if port.setBaudRate(old_baudrate):
        print("Succeeded to connect to baudrate %d!" % old_baudrate)
    else:
        print("Failed to connect to baudrate %d!" % old_baudrate)
        return 4

    if new_baudrate == 57600:
        rate_id = 1
    elif new_baudrate == 1000000:
        rate_id = 3
    else:
        print("Unsupported baudrate, using 1000000")
        rate_id = 3

    write1(254, ADD_TORQUE_ENABLE, TORQUE_DISABLE)
    write1(254, ADD_BAUD_RATE, rate_id)
    # This code can't actually confirm that the baudrate was changed
    print("Baudrate successfully changed to %d. Reconnecting." % new_baudrate)


def test():
    """ test register setting and restore to MilliBox settings """

    global num_motors
    global base_type
    global base_ratio
    global base_res
    global ram_offset
    global h_zero
    global base_pos_acc_thresh
    global base_vel_unit
    global max_H_velocity
    global h_P
    global h_I
    global h_D
    global h_FF1
    global base_moving_threshold
    global max_offset
    global gim_type
    global GIM_MOTION

    print("===== Gimbal type CHECK =====")                                      # Gimbal check - search for highest motor number to lowest

    if read2(Z, ADD_MODEL_NUMBER, quiet=True) >= XH_MOTOR_CLASS:                # check if Z motor is found (GIM05)
        print("====> Motor6 (Z) motor identified")
        num_motors = 6
        if read2(T, ADD_MODEL_NUMBER, quiet=True) >= XH_MOTOR_CLASS:            # check if T motor is also found (GIM05)
            print("====> Motor5 (T) motor identified")
            gim_type = SPHERICAL
            print("====> GIM05 identified")
        else:
            print("Motor Missing - Z motor without T motor!!")                  # invalid config, report error and quit
            return 12

    elif read2(T, ADD_MODEL_NUMBER, quiet=True) >= XH_MOTOR_CLASS:              # check if T motor is found (GIM05_FIXED)
        print("====> Motor5 (T) motor identified")
        num_motors = 5
        gim_type = SPHERICAL
        print("====> GIM05_FIXED identified")

    elif read2(P, ADD_MODEL_NUMBER, quiet=True) >= XH_MOTOR_CLASS:              # check if P motor is found (GIM04x)
        print("====> Motor4 (P) motor identified")
        num_motors = 4
        gim_type = HV
        if read2(R, ADD_MODEL_NUMBER, quiet=True) >= XH_MOTOR_CLASS:            # check if R motor is also found (GIM04x)
            print("====> Motor3 (R) motor identified")
        else:
            print("Motor Missing - P motor without R motor!!")                  # invalid config, report error and quit
            return 12

        if read2(V, ADD_MODEL_NUMBER, quiet=True) >= XH_MOTOR_CLASS:            # check if V motor is also found (GIM04x)
            print("====> Motor2 (V) motor identified")
        else:
            print("Motor Missing - P motor without V motor!!")                  # invalid config, report error and quit
            return 12

        print("====> GIM04x identified")

    elif read2(R, ADD_MODEL_NUMBER, quiet=True) >= XH_MOTOR_CLASS:              # check if slave motor is found (GIM03/GIM04)
        print("====> Motor3 (R) motor identified")
        num_motors = 3
        gim_type = HV
        print("====> GIM03/04 identified")
        if read2(V, ADD_MODEL_NUMBER, quiet=True) >= XH_MOTOR_CLASS:            # check if V motor is also found (GIM03/GIM04)
            print("====> Motor2 (V) motor identified")
        else:
            print("Motor Missing - R motor without V motor!!")                  # invalid config, report error and quit
            return 12

    elif read2(V, ADD_MODEL_NUMBER, quiet=True) > 0:                            # check if V motor is found (GIM01/GIM06)
        print("====> Motor2 (V) motor identified")
        num_motors = 2
        gim_type = HV
        if read1(V, ADD_FW_VER) < 50 and read1(V, ADD_DRIVE_MODE) == DRIVE_MODE_NORM:       # old FW and normal drive ==> GIM01
            print("====> GIM01 identified")
        elif read1(V, ADD_FW_VER) >= 50 and read1(V, ADD_DRIVE_MODE) == DRIVE_MODE_V_G6:    # new FW and reverse drive ==> GIM06
            print("====> GIM06 identified")
        else:
            print("Unable to identify GIM01 or GIM06. Invalid FW/Drive Mode combination.")  # invalid combo, report error and quit
            return 12

    elif read2(H, ADD_MODEL_NUMBER, quiet=True) > 0:                            # check if H motor is found (GIM1D/GIM05A)
        num_motors = 1
        gim_type = HV
        print("====> GIM1D/GIM05A identified")
    else:
        print("Failed MOTOR READBACK!!")                                        # no motors found, report error and quit
        return 11

    H_type = read2(H, ADD_MODEL_NUMBER)
    if H_type == PH42_020_MOTOR_TYPE:                                           # check for PH42 motor in base
        base_type = 4
        base_ratio = PH42_H_RATIO
        base_res = PH42_RES
        ram_offset = PH42_RAM_ADD_OFFSET
        h_zero = PH42_0
        base_pos_acc_thresh = PH42_POS_ACC_THRESH
        base_vel_unit = PH42_VELOCITY_UNIT
        max_H_velocity = PH42_MAX_PROFILE_VELOCITY
        h_P = PH42_POSITION_P_G
        h_I = PH42_POSITION_I_G
        h_D = PH42_POSITION_D_G
        h_FF1 = PH42_FF1_G
        base_moving_threshold = PH42_MOVING_THRESHOLD
        max_offset = PH42_MAX_OFFSET
        print("====> PH42 Base identified")
    elif H_type >= XH_MOTOR_CLASS or H_type == MX64AT_MOTOR_TYPE:               # otherwise XH540 or MX64AT motor in base
        base_type = 1                                                           # settings are compatible
        base_ratio = XH540_H_RATIO
        base_res = XH540_RES
        ram_offset = 0
        h_zero = XH540_0
        base_pos_acc_thresh = XH540_POS_ACC_THRESH
        base_vel_unit = XH540_VELOCITY_UNIT
        max_H_velocity = XH540_MAX_PROFILE_VELOCITY
        h_P = XH540_POSITION_P_G
        h_I = XH540_POSITION_I_G
        h_D = XH540_POSITION_D_G
        h_FF1 = XH540_FF1_G
        base_moving_threshold = XH540_MOVING_THRESHOLD
        max_offset = XH540_MAX_OFFSET
        print("====> XH540 or MX64AT Base identified")
    elif H_type == 0:
        print("Motor Missing - H motor not found")                              # could not read H, report error and quit
        return 12
    else:
        print("Motor Misidentified - H motor type identified as %d" % H_type)   # invalid config, report error and quit
        return 12

    # set up the gimbal motion data structure based on connected gimbal
    GIM_MOTION = {"gim_type": gim_type, "num_motors": num_motors, "accuracy": ACCURACY_DEFAULT}
    if gim_type == HV:
        GIM_MOTION[1] = {"model": 0, "vel": 0, "accel": 0, "anglelim": [-180, 180]}
        if num_motors >= 2:
            GIM_MOTION[2] = {"model": 0, "vel": 0, "accel": 0, "anglelim": [-180, 180]}
            if num_motors >= 4:
                GIM_MOTION[4] = {"model": 0, "vel": 0, "accel": 0, "anglelim": [-180, 180]}
    elif gim_type == SPHERICAL:
        GIM_MOTION[1] = {"model": 0, "vel": 0, "accel": 0, "anglelim": [-180, 180]}
        if num_motors >= 5:
            GIM_MOTION[5] = {"model": 0, "vel": 0, "accel": 0, "anglelim": [-180, 180]}
            if num_motors >= 6:
                GIM_MOTION[6] = {"model": 0, "vel": 0, "accel": 0, "anglelim": [-180, 180]}

    print("===== motor configuration CHECK =====")                              # disable torque to access flash
    disable_torque(H)
    if gim_type == SPHERICAL:
        if num_motors >= 5:
            disable_torque(T)
            if num_motors >= 6:
                disable_torque(Z)
    elif gim_type == HV:
        if num_motors >= 2:
            disable_torque(V)
        if num_motors >= 3:
            disable_torque(R)
        if num_motors >= 4:
            disable_torque(P)
    print("flash access set ")

    if gim_type == HV:                                                          # test operating mode and drive mode
        opmodeH = read1(H, ADD_OPERATING_MODE)                                  # check H operating mode
        print("operating mode H = %d" % opmodeH)
        if opmodeH == OPER_MODE:
            print("**** Operating mode is OK ****")
        else:
            print(" resetting operating mode ")
            write1(H, ADD_OPERATING_MODE, OPER_MODE)
            print("operating mode reset to : %d" % OPER_MODE)

        if num_motors >= 2:
            opmodeV = read1(V, ADD_OPERATING_MODE)                              # check V operating mode
            print("operating mode V = %d" % opmodeV)
            if opmodeV == OPER_MODE:
                print("**** Operating mode is OK ****")
            else:
                print(" resetting operating mode ")
                write1(V, ADD_OPERATING_MODE, OPER_MODE)
                print("operating mode reset to : %d" % OPER_MODE)

        if num_motors >= 3:
            opmodeR = read1(R, ADD_OPERATING_MODE)                              # check R operating mode
            print("operating mode R = %d" % opmodeR)
            if opmodeR == OPER_MODE:
                print("**** Operating mode is OK ****")
            else:
                print(" resetting operating mode ")
                write1(R, ADD_OPERATING_MODE, OPER_MODE)
                print("operating mode reset to : %d" % OPER_MODE)

            drmodeR = read1(R, ADD_DRIVE_MODE)                                  # check R drive mode
            print("drive mode R = %d" % drmodeR)
            if drmodeR == DRIVE_MODE_R:                                         # This part is to make sure that if Motors are upgraded
                print("reverse motor drive mode is OK")                         # GIM03/GIM04 reverse slave motor (ID:3) does not lose its slave mode
            else:                                                               # otherwsie it could damage the gimbal
                print(" resetting reverse motor drive mode")
                write1(R, ADD_DRIVE_MODE, DRIVE_MODE_R)
                print("drive mode reset to : %d" % DRIVE_MODE_R)

        if num_motors >= 4:
            opmodeP = read1(P, ADD_OPERATING_MODE)                              # check P operating mode
            print("operating mode P = %d" % opmodeP)
            if opmodeP == OPER_MODE:
                print("**** Operating mode is OK ****")
            else:
                print(" resetting operating mode ")
                write1(P, ADD_OPERATING_MODE, OPER_MODE)
                print("operating mode reset to : %d" % OPER_MODE)

    if gim_type == SPHERICAL:                                                   # test operating mode
        opmodeH = read1(H, ADD_OPERATING_MODE)                                  # check H operating mode
        print("operating mode H = %d" % opmodeH)
        if opmodeH == OPER_MODE:
            print("**** Operating mode is OK ****")
        else:
            print(" resetting operating mode ")
            write1(H, ADD_OPERATING_MODE, OPER_MODE)
            print("operating mode reset to : %d" % OPER_MODE)

        if num_motors >= 5:
            opmodeT = read1(T, ADD_OPERATING_MODE)                              # check T operating mode
            print("operating mode T = %d" % opmodeT)
            if opmodeT == OPER_MODE:
                print("**** Operating mode is OK ****")
            else:
                print(" resetting operating mode ")
                write1(T, ADD_OPERATING_MODE, OPER_MODE)
                print("operating mode reset to : %d" % OPER_MODE)

        if num_motors >= 6:
            opmodeZ = read1(Z, ADD_OPERATING_MODE)                              # check Z operating mode
            print("operating mode Z = %d" % opmodeZ)
            if opmodeZ == OPER_MODE:
                print("**** Operating mode is OK ****")
            else:
                print(" resetting operating mode ")
                write1(Z, ADD_OPERATING_MODE, OPER_MODE)
                print("operating mode reset to : %d" % OPER_MODE)

            drmodeZ = read1(Z, ADD_DRIVE_MODE)                                  # check Z drive mode
            print("drive mode Z = %d" % drmodeZ)
            if drmodeZ == DRIVE_MODE_Z:                                         # This part is to make sure that if Motors are upgraded
                print("gim05 reverse motor drive mode is OK")                   # GIM05 stays in reverse mode (to match polarity of T)
            else:
                print(" resetting reverse motor drive mode")
                write1(Z, ADD_DRIVE_MODE, DRIVE_MODE_Z)
                print("drive mode reset to : %d" % DRIVE_MODE_Z)

    if gim_type == HV:
        if read4(H, ADD_MOVE_THRESHOLD) == base_moving_threshold:               # test H moving Threshold
            print("H moving threshold OK")
        else:
            print("resetting H moving threshold")
            write4(H, ADD_MOVE_THRESHOLD, base_moving_threshold)
            print("H moving threshold set to : %d" % base_moving_threshold)

        if num_motors >= 2:
            if read4(V, ADD_MOVE_THRESHOLD) == XH540_MOVING_THRESHOLD:          # test V moving Threshold - XH540
                print("V moving threshold OK")
            else:
                print("resetting V moving threshold")
                write4(V, ADD_MOVE_THRESHOLD, XH540_MOVING_THRESHOLD)
                print("V moving threshold set to : " + str(XH540_MOVING_THRESHOLD))

            if num_motors >= 4:
                if read4(P, ADD_MOVE_THRESHOLD) == XH540_MOVING_THRESHOLD:      # test P moving Threshold - XH540
                    print("P moving threshold OK")
                else:
                    print("resetting P moving threshold")
                    write4(P, ADD_MOVE_THRESHOLD, XH540_MOVING_THRESHOLD)
                    print("P moving threshold set to : " + str(XH540_MOVING_THRESHOLD))

    elif gim_type == SPHERICAL:
        if read4(H, ADD_MOVE_THRESHOLD) == base_moving_threshold:               # test H moving Threshold
            print("H moving threshold OK")
        else:
            print("resetting H moving threshold")
            write4(H, ADD_MOVE_THRESHOLD, base_moving_threshold)
            print("H moving threshold set to : %d" % base_moving_threshold)

        if num_motors >= 5:
            if read4(T, ADD_MOVE_THRESHOLD) == XH540_MOVING_THRESHOLD:          # test T moving Threshold - XH540
                print("T moving threshold OK")
            else:
                print("resetting T moving threshold")
                write4(T, ADD_MOVE_THRESHOLD, XH540_MOVING_THRESHOLD)
                print("T moving threshold set to : " + str(XH540_MOVING_THRESHOLD))

            if num_motors >= 6:
                if read4(Z, ADD_MOVE_THRESHOLD) == XH540_MOVING_THRESHOLD:      # test Z moving Threshold - XH540
                    print("Z moving threshold OK")
                else:
                    print("resetting Z moving threshold")
                    write4(Z, ADD_MOVE_THRESHOLD, XH540_MOVING_THRESHOLD)
                    print("Z moving threshold set to : " + str(XH540_MOVING_THRESHOLD))

    print("=== setting motors dynamic parameters ===")                          # those setting should be done at every power on
    if gim_type == HV:
        enable_torque(H)
        if num_motors >= 2:
            enable_torque(V)
        if num_motors >= 3:
            enable_torque(R)
        if num_motors >= 4:
            enable_torque(P)
    elif gim_type == SPHERICAL:
        enable_torque(H)
        if num_motors >= 5:
            enable_torque(T)
        if num_motors >= 6:
            enable_torque(Z)

    print("ram access enabled")                                                 # close access to flash and enable acces to RAM area

    write2(H, (ADD_POS_D_GAIN + ram_offset), h_D)                               # set H PID values
    write2(H, (ADD_POS_P_GAIN + ram_offset), h_P)
    write2(H, (ADD_POS_I_GAIN + ram_offset), h_I)
    write2(H, (ADD_FF_1ST_GAIN+ ram_offset), h_FF1)
    print("H PID configuration set")

    if gim_type == HV:
        if num_motors >= 2:
            write2(V, ADD_POS_D_GAIN, XH540_POSITION_D_G)                       # set V PID values
            write2(V, ADD_POS_P_GAIN, XH540_POSITION_P_G)
            write2(V, ADD_POS_I_GAIN, XH540_POSITION_I_G)
            write2(V, ADD_FF_1ST_GAIN, XH540_FF1_G)
            print("V PID configuration set")

        if num_motors >= 4:
            write2(P, ADD_POS_D_GAIN, XH540_POSITION_D_G)                       # set P PID values
            write2(P, ADD_POS_P_GAIN, XH540_POSITION_P_G)
            write2(P, ADD_POS_I_GAIN, XH540_POSITION_I_G)
            write2(P, ADD_FF_1ST_GAIN, XH540_FF1_G)
            print("P PID configuration set")

    if gim_type == SPHERICAL:
        if num_motors >= 5:
            write2(T, ADD_POS_D_GAIN, XH540_POSITION_D_G)                       # set T PID values
            write2(T, ADD_POS_P_GAIN, XH540_POSITION_P_G)
            write2(T, ADD_POS_I_GAIN, XH540_POSITION_I_G)
            write2(T, ADD_FF_1ST_GAIN, XH540_FF1_G)
            print("T PID configuration set")

        if num_motors >= 6:
            write2(Z, ADD_POS_D_GAIN, XH540_POSITION_D_G)                       # set Z PID values
            write2(Z, ADD_POS_P_GAIN, XH540_POSITION_P_G)
            write2(Z, ADD_POS_I_GAIN, XH540_POSITION_I_G)
            write2(Z, ADD_FF_1ST_GAIN, XH540_FF1_G)
            print("Z PID configuration set")

    print("verifying all settings")                                             # check all actual values in RAM

    print("H position D gain : %d = %d" % (read2(H, ADD_POS_D_GAIN + ram_offset), h_D))
    print("H position P gain : %d = %d" % (read2(H, ADD_POS_P_GAIN + ram_offset), h_P))
    print("H position I gain : %d = %d" % (read2(H, ADD_POS_I_GAIN + ram_offset), h_I))
    print("H FF1 gain : %d = %d" % (read2(H, ADD_FF_1ST_GAIN + ram_offset), h_FF1))

    if gim_type == HV:
        if num_motors >= 2:
            print("V position D gain : %d = %d" % (read2(V, ADD_POS_D_GAIN), XH540_POSITION_D_G))
            print("V position P gain : %d = %d" % (read2(V, ADD_POS_P_GAIN), XH540_POSITION_P_G))
            print("V position I gain : %d = %d" % (read2(V, ADD_POS_I_GAIN), XH540_POSITION_I_G))
            print("V FF1 gain : %d = %d" % (read2(V, ADD_FF_1ST_GAIN), XH540_FF1_G))

        if num_motors >= 4:
            print("P position D gain : %d = %d" % (read2(P, ADD_POS_D_GAIN), XH540_POSITION_D_G))
            print("P position P gain : %d = %d" % (read2(P, ADD_POS_P_GAIN), XH540_POSITION_P_G))
            print("P position I gain : %d = %d" % (read2(P, ADD_POS_I_GAIN), XH540_POSITION_I_G))
            print("P FF1 gain : %d = %d" % (read2(P, ADD_FF_1ST_GAIN), XH540_FF1_G))

    if gim_type == SPHERICAL:
        if num_motors >= 5:
            print("T position D gain : %d = %d" % (read2(T, ADD_POS_D_GAIN), XH540_POSITION_D_G))
            print("T position P gain : %d = %d" % (read2(T, ADD_POS_P_GAIN), XH540_POSITION_P_G))
            print("T position I gain : %d = %d" % (read2(T, ADD_POS_I_GAIN), XH540_POSITION_I_G))
            print("T FF1 gain : %d = %d" % (read2(T, ADD_FF_1ST_GAIN), XH540_FF1_G))

        if num_motors >= 6:
            print("Z position D gain : %d = %d" % (read2(Z, ADD_POS_D_GAIN), XH540_POSITION_D_G))
            print("Z position P gain : %d = %d" % (read2(Z, ADD_POS_P_GAIN), XH540_POSITION_P_G))
            print("Z position I gain : %d = %d" % (read2(Z, ADD_POS_I_GAIN), XH540_POSITION_I_G))
            print("Z FF1 gain : %d = %d" % (read2(Z, ADD_FF_1ST_GAIN), XH540_FF1_G))

    print("BOOT UP MOTOR POSITION CHECK before re-alignement")                  # print motor absolute position at boot up
    getposition(0)                                                              # print all offsets before re-alignment

    versionH = read1(H, ADD_FW_VER)
    if base_type == 1:
        if versionH < XH540_MIN_VERSION:                                        # make sure Firmware is up to date
            print("firmware version not supported: %d" % versionH)
        else:
            print("firmware version of motor H is OKAY ")
            if versionH > 42:                                                   # offset handling after FW 42 has changed
                realign(H)                                                      # we may need to re-align current position
    elif base_type == 4:
        if versionH < PH42_MIN_VERSION:                                         # make sure Firmware is up to date
            print("firmware version not supported: %d" % versionH)
        else:
            print("firmware version of motor H is OKAY ")
            if versionH > PH42_MIN_VERSION:                                     # offset handling after FW 11 has changed
                realign(H)

    if gim_type == HV:
        if num_motors >= 2:
            versionV = read1(V, ADD_FW_VER)
            if versionV < XH540_MIN_VERSION:                                    # make sure Firmware is up to date
                print("firmware version not supported: %d" % versionV)
            else:
                print("firmware version of motor V is OKAY ")
                if versionV > 42:                                               # offset handling after FW 42 has changed
                    realign(V)                                                  # we may need to re-align current position
            if num_motors >= 4:
                versionP = read1(P, ADD_FW_VER)
                if versionP < XH540_MIN_VERSION:                                # make sure Firmware is up to date
                    print("firmware version not supported: %d" % versionP)
                else:
                    print("firmware version of motor P is OKAY ")
                    if versionP > 42:                                           # offset handling after FW 42 has changed
                        realign(P)                                              # we may need to re-align current position

    if gim_type == SPHERICAL:
        if num_motors >= 5:
            versionT = read1(T, ADD_FW_VER)
            if versionT < XH540_MIN_VERSION:                                    # make sure Firmware is up to date
                print("firmware version not supported: %d" % versionT)
            else:
                print("firmware version of motor T is OKAY ")
                if versionT > 42:                                               # offset handling after FW 42 has changed
                    realign(T)                                                  # we may need to re-align current position
            if num_motors >= 6:
                versionZ = read1(Z, ADD_FW_VER)
                if versionZ < XH540_MIN_VERSION:                                # make sure Firmware is up to date
                    print("firmware version not supported: %d" % versionZ)
                else:
                    print("firmware version of motor Z is OKAY ")
                    if versionZ > 42:                                           # offset handling after FW 42 has changed
                        realign(Z)                                              # we may need to re-align current position

    print_offset_all()                                                          # print all offsets after re-alignment

    print("*** set_motor_motion_default ***")
    set_gim_motion_default()                                                    # set default motor motion
    print_gim_motion()

    return 0


def enable_torque(motor):
    """ allow motor to move and block eeprom register access """
    if motor <= num_motors:
        if motor == H:
            if read1(motor, (ADD_TORQUE_ENABLE + ram_offset)) == TORQUE_ENABLE:
                print("torque for motor %d is already enabled" % motor)
            else:
                write1(motor, (ADD_TORQUE_ENABLE + ram_offset), TORQUE_ENABLE)
                print("torque now enabled for motor %d" % motor)
        else:
            if read1(motor, ADD_TORQUE_ENABLE) == TORQUE_ENABLE:
                print("torque for motor %d is already enabled" % motor)
            else:
                write1(motor, ADD_TORQUE_ENABLE, TORQUE_ENABLE)
                print("torque now enabled for motor %d" % motor)
    else:
        print("WARNING: Attempting to enable torque on Motor %d that does not exist" % motor)
    return


def disable_torque(motor):
    """ stop motor from being moved and allow eeprom register access """
    if motor <= num_motors:
        if motor == H:
            if read1(motor, (ADD_TORQUE_ENABLE + ram_offset)) == TORQUE_DISABLE:
                print("torque for motor %d is already disabled" % motor)
            else:
                write1(motor, (ADD_TORQUE_ENABLE + ram_offset), TORQUE_DISABLE)
                print("torque now disabled for motor %d" % motor)
        else:
            if read1(motor, ADD_TORQUE_ENABLE) == TORQUE_DISABLE:
                print("torque for motor %d is already disabled" % motor)
            else:
                write1(motor, ADD_TORQUE_ENABLE, TORQUE_DISABLE)
                print("torque now disabled for motor %d" % motor)
    else:
        print("WARNING: Attempting to disable torque on Motor %d that does not exist" % motor)
    return


def get_gimtype():
    """ return the gimbal type (HV or SPHERICAL) """
    global gim_type
    return gim_type


def get_nummotors():
    """ return the number of detected motors """
    global num_motors
    return num_motors


def set_gim_motion_default():
    """ set gimbal motion control parameters to default """
    global GIM_MOTION

    get_model_number()                                                          # read back model number for all motors
    GIM_MOTION["accuracy"] = ACCURACY_DEFAULT                                   # set accuracy to default

    GIM_MOTION[1]["vel"] = 0                                                    # set default values for H motor
    if base_type == 4:
        GIM_MOTION[1]["accel"] = PH42_H_ACCEL_LIMIT_DEF
    else:
        if num_motors >= 3:
            GIM_MOTION[1]["accel"] = XH540_H_ACCEL_LIMIT_DEF_G4                 # set accel default for GIM03/GIM04 H motor
        else:
            GIM_MOTION[1]["accel"] = XH540_H_ACCEL_LIMIT_DEF_G1                 # set accel default for GIM01/GIM06 H motor
    GIM_MOTION[1]["anglelim"] = [max(-180, H_ANGLE_HARD_LIMIT[0]), min(180, H_ANGLE_HARD_LIMIT[1])]

    if gim_type == HV:
        if num_motors >= 2:
            GIM_MOTION[2]["vel"] = 0                                            # set default values for V motor
            if num_motors >= 3:
                GIM_MOTION[2]["accel"] = XH540_V_ACCEL_LIMIT_DEF_G4             # set accel default for GIM03/GIM04 V motor
            else:
                GIM_MOTION[2]["accel"] = XH540_V_ACCEL_LIMIT_DEF_G1             # set accel default for GIM01/GIM06 V motor
            GIM_MOTION[2]["anglelim"] = [max(-180, V_ANGLE_HARD_LIMIT[0]), min(180, V_ANGLE_HARD_LIMIT[1])]

        if num_motors >= 4:
            GIM_MOTION[4]["vel"] = 0                                            # set default values for P motor
            GIM_MOTION[4]["accel"] = XH540_P_ACCEL_LIMIT_DEF
            GIM_MOTION[4]["anglelim"] = [max(-180, P_ANGLE_HARD_LIMIT[0]), min(180, P_ANGLE_HARD_LIMIT[1])]

    if gim_type == SPHERICAL:
        if num_motors >= 5:
            GIM_MOTION[5]["vel"] = 0                                            # set default values for T motor
            GIM_MOTION[5]["accel"] = XH540_T_ACCEL_LIMIT_DEF
            GIM_MOTION[5]["anglelim"] = [max(-180, T_ANGLE_HARD_LIMIT[0]), min(180, T_ANGLE_HARD_LIMIT[1])]

        if num_motors >= 6:
            GIM_MOTION[6]["vel"] = 0                                            # set default values for Z motor
            GIM_MOTION[6]["accel"] = XH540_Z_ACCEL_LIMIT_DEF
            GIM_MOTION[6]["anglelim"] = [max(-180, Z_ANGLE_HARD_LIMIT[0]), min(180, Z_ANGLE_HARD_LIMIT[1])]

    write_velocity()                                                            # write to velocity registers
    write_accel()                                                               # write to accel registers

    return


def set_gim_motion(gim_motion):
    """ set the gimbal motion control parameters """
    global GIM_MOTION
    GIM_MOTION = gim_motion                                                     # set global GIM_MOTION structure
    write_velocity()                                                            # write to velocity registers
    write_accel()                                                               # write to accel registers
    return


def get_gim_motion():
    """ return the gimbal motion control parameters """
    global GIM_MOTION
    return GIM_MOTION                                                           # return values from global GIM_MOTION structure


def print_gim_motion():
    """ display the gimbal motion control parameters """
    global GIM_MOTION
    print("****** GIM motion settings ******")
    for x in GIM_MOTION.keys():                                                 # format and print GIM_MOTION structure
        print("%12s = %s" % (str(x), str(GIM_MOTION[x])))
    return


def get_model_number():
    """ read motor model number into GIM_MOTION structure """
    global GIM_MOTION

    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        return

    if gim_type == HV:
        H_model = read2(H, ADD_MODEL_NUMBER)                                    # read the current motor H model number
        print("H model is = %d" % H_model)
        GIM_MOTION[1]["model"] = H_model
        if num_motors >= 2:
            V_model = read2(V, ADD_MODEL_NUMBER)                                # read the current motor V model number
            print("V model is = %d" % V_model)
            GIM_MOTION[2]["model"] = V_model

        if num_motors >= 4:
            P_model = read2(P, ADD_MODEL_NUMBER)                                # read the current motor P model number
            print("P model is = %d" % P_model)
            GIM_MOTION[4]["model"] = P_model

    if gim_type == SPHERICAL:
        H_model = read2(H, ADD_MODEL_NUMBER)                                    # read the current motor H model number
        print("H model is = %d" % H_model)
        GIM_MOTION[1]["model"] = H_model
        if num_motors >= 5:
            T_model = read2(T, ADD_MODEL_NUMBER)                                # read the current motor T model number
            print("T model is = %d" % T_model)
            GIM_MOTION[5]["model"] = T_model

        if num_motors >= 6:
            Z_model = read2(Z, ADD_MODEL_NUMBER)                                # read the current motor Z model number
            print("Z model is = %d" % Z_model)
            GIM_MOTION[6]["model"] = Z_model

    return


def set_accuracy(accuracy):
    """ change accuracy setting for gimbal movement """
    global GIM_MOTION

    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        return

    GIM_MOTION["accuracy"] = accuracy                                           # set accuracy value in GIM_MOTION structure

    return


def set_velocity(vel1=0, vel2=0, vel3=0):
    """ change rotation speed of the motors """

    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        return

    if gim_type == HV:                                                          # set velocities for HV gimbal
        GIM_MOTION[1]["vel"] = vel1

        if num_motors >= 2:
            GIM_MOTION[2]["vel"] = vel2

        if num_motors >= 4:
            GIM_MOTION[4]["vel"] = vel3

    if gim_type == SPHERICAL:                                                   # set velocities for SPHERICAL gimbal
        GIM_MOTION[1]["vel"] = vel1

        if num_motors >= 5:
            GIM_MOTION[5]["vel"] = vel2

        if num_motors >= 6:
            GIM_MOTION[6]["vel"] = vel3

    write_velocity()                                                            # write to velocity registers

    return


def get_velocity(log_to_screen=True):
    """ read motor RPM value """
    global base_ratio
    global ram_offset
    global base_vel_unit
    global V_GOAL_VELOCITY
    global P_GOAL_VELOCITY
    global X_VELOCITY_UNIT
    global GIM_MOTION

    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        return None, None, None

    if gim_type == HV:
        H_velocity = base_vel_unit * read4(H, (ADD_VELOC_PROFILE + ram_offset)) / base_ratio        # read the current motor H velocity setting and convert to rpm
        if log_to_screen:
            print("H velocity is set to = %0.2f rpm" % H_velocity)
        GIM_MOTION[1]["vel"] = H_velocity
        if num_motors >= 2:
            V_velocity = XH540_VELOCITY_UNIT * read4(V, ADD_VELOC_PROFILE)/XH540_V_RATIO            # read the current motor V velocity setting and convert to rpm
            if log_to_screen:
                print("V velocity is set to = %0.2f rpm" % V_velocity)
            GIM_MOTION[2]["vel"] = V_velocity
        else:
            V_velocity = 0

        if num_motors >= 4:
            P_velocity = XH540_VELOCITY_UNIT * read4(P, ADD_VELOC_PROFILE)/XH540_P_RATIO            # read the current motor P velocity setting and convert to rpm
            if log_to_screen:
                print("P velocity is set to = %0.2f rpm" % P_velocity)
            GIM_MOTION[4]["vel"] = P_velocity
        else:
            P_velocity = 0
        vel1 = H_velocity
        vel2 = V_velocity
        vel3 = P_velocity

    if gim_type == SPHERICAL:
        H_velocity = base_vel_unit * read4(H, (ADD_VELOC_PROFILE + ram_offset)) / base_ratio        # read the current motor H velocity setting and convert to rpm
        if log_to_screen:
            print("H velocity is set to = %0.2f rpm" % H_velocity)
        GIM_MOTION[1]["vel"] = H_velocity
        if num_motors >= 5:
            T_velocity = XH540_VELOCITY_UNIT * read4(T, ADD_VELOC_PROFILE)/XH540_T_RATIO            # read the current motor T velocity setting and convert to rpm
            if log_to_screen:
                print("T velocity is set to = %0.2f rpm" % T_velocity)
            GIM_MOTION[5]["vel"] = T_velocity
        else:
            T_velocity = 0

        if num_motors >= 6:
            Z_velocity = XH540_VELOCITY_UNIT * read4(Z, ADD_VELOC_PROFILE)/XH540_Z_RATIO            # read the current motor Z velocity setting and convert to rpm
            if log_to_screen:
                print("Z velocity is set to = %0.2f rpm" % Z_velocity)
            GIM_MOTION[6]["vel"] = Z_velocity
        else:
            Z_velocity = 0
        vel1 = H_velocity
        vel2 = T_velocity
        vel3 = Z_velocity

    return vel1, vel2, vel3


def write_velocity():
    """ write velocity settings to velocity profile registers """
    global GIM_MOTION

    global base_ratio
    global ram_offset
    global base_vel_unit
    global max_H_velocity

    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        return

    if gim_type == HV:                                                          # set velocities for HV gimbal
        h_vel = GIM_MOTION[1]["vel"]
        if h_vel == 0:                                                          # if user set 0 then default max speed is used
            H_vel = max_H_velocity
        else:
            H_vel = int(round(h_vel * base_ratio / base_vel_unit))              # convert H velocity to actual register value
            if H_vel > max_H_velocity:                                          # can't exceed max speed
                H_vel = max_H_velocity
                print("clamping to maximum possible velocity for H motor")
            if H_vel < 1:                                                       # can't be less than min speed
                H_vel = 1
                print("clamping to minimum possible velocity for H motor")
        write4(H, ADD_VELOC_PROFILE + ram_offset, H_vel)                        # program register value in RAM

        if num_motors >= 2:
            v_vel = GIM_MOTION[2]["vel"]
            if v_vel == 0:                                                      # if user set 0 then default max speed is used
                V_vel = XH540_MAX_PROFILE_VELOCITY
            else:
                V_vel = int(round(v_vel*XH540_V_RATIO/XH540_VELOCITY_UNIT))     # convert V velocity to actual register value
                if V_vel > XH540_MAX_PROFILE_VELOCITY:                          # can't exceed max speed
                    V_vel = XH540_MAX_PROFILE_VELOCITY
                    print("clamping to maximum possible velocity for V motor")
                if V_vel < 1:                                                   # can't be less than min speed
                    V_vel = 1
                    print("clamping to minimum possible velocity for V motor")
            write4(V, ADD_VELOC_PROFILE, V_vel)                                 # program register value in RAM

        if num_motors >= 4:
            p_vel = GIM_MOTION[4]["vel"]
            if p_vel == 0:                                                      # if user set 0 then default max speed is used
                P_vel = XH540_MAX_PROFILE_VELOCITY
            else:
                P_vel = int(round(p_vel*XH540_P_RATIO/XH540_VELOCITY_UNIT))     # convert P velocity to actual register value
                if P_vel > XH540_MAX_PROFILE_VELOCITY:                          # can't exceed max speed
                    P_vel = XH540_MAX_PROFILE_VELOCITY
                    print("clamping to maximum possible velocity for P motor")
                if P_vel < 1:                                                   # can't be less than min speed
                    P_vel = 1
                    print("clamping to minimum possible velocity for P motor")
            write4(P, ADD_VELOC_PROFILE, P_vel)

    if gim_type == SPHERICAL:                                                   # set velocities for SPHERICAL gimbal
        h_vel = GIM_MOTION[1]["vel"]
        if h_vel == 0:                                                          # if user set 0 then default max speed is used
            H_vel = max_H_velocity
        else:
            H_vel = int(round(h_vel*base_ratio/base_vel_unit))                  # convert H velocity to actual register value
            if H_vel > max_H_velocity:                                          # can't exceed max speed
                H_vel = max_H_velocity
                print("clamping to maximum possible velocity for H motor")
            if H_vel < 1:                                                       # can't be less than min speed
                H_vel = 1
                print("clamping to minimum possible velocity for H motor")
        write4(H, ADD_VELOC_PROFILE + ram_offset, H_vel)                        # program register value in RAM

        if num_motors >= 5:
            t_vel = GIM_MOTION[5]["vel"]
            if t_vel == 0:                                                      # if user set 0 then default max speed is used
                T_vel = XH540_MAX_PROFILE_VELOCITY
            else:
                T_vel = int(round(t_vel*XH540_T_RATIO/XH540_VELOCITY_UNIT))     # convert T velocity to actual register value
                if T_vel > XH540_MAX_PROFILE_VELOCITY:                          # can't exceed max speed
                    T_vel = XH540_MAX_PROFILE_VELOCITY
                    print("clamping to maximum possible velocity for T motor")
                if T_vel < 1:                                                   # can't be less than min speed
                    T_vel = 1
                    print("clamping to minimum possible velocity for T motor")
            write4(T, ADD_VELOC_PROFILE, T_vel)                                 # program register value in RAM

        if num_motors >= 6:
            z_vel = GIM_MOTION[6]["vel"]
            if z_vel == 0:                                                      # if user set 0 then default max speed is used
                Z_vel = XH540_MAX_PROFILE_VELOCITY
            else:
                Z_vel = int(round(z_vel*XH540_Z_RATIO/XH540_VELOCITY_UNIT))     # convert Z velocity to actual register value
                if Z_vel > XH540_MAX_PROFILE_VELOCITY:                          # can't exceed max speed
                    Z_vel = XH540_MAX_PROFILE_VELOCITY
                    print("clamping to maximum possible velocity for Z motor")
                if Z_vel < 1:                                                   # can't be less than min speed
                    Z_vel = 1
                    print("clamping to minimum possible velocity for Z motor")
            write4(Z, ADD_VELOC_PROFILE, Z_vel)                                 # program register value in RAM

    get_velocity(log_to_screen=False)                                           # check the values by reading back the registers

    return


def set_accel(accel1=0, accel2=0, accel3=0):
    """ change acceleration limit of the motors """

    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        return

    if gim_type == HV:                                                          # set accel limit for HV gimbal
        if GIM_MOTION[1]["model"] >= PH_MOTOR_CLASS:                            # set accel limit for H motor
            accel1_def = PH42_H_ACCEL_LIMIT_DEF                                 # limits for PH42 motor
            accel1_hard_lim = PH42_H_ACCEL_HARD_LIMIT
        else:
            accel1_def = XH540_H_ACCEL_LIMIT_DEF                                # limits for XH540 motor
            accel1_hard_lim = XH540_H_ACCEL_HARD_LIMIT

        if accel1 == 0:
            accel1 = accel1_def                                                 # set to default
        if accel1_hard_lim[0] <= accel1 <= accel1_hard_lim[1]:                  # check against hard limit
            GIM_MOTION[1]["accel"] = accel1
            print("*** H accel limit set ***")
        else:
            print("*** H accel limit = %g out of range %s... SKIPPING. ***" % (accel1, str(accel1_hard_lim)))

        if num_motors >= 2:                                                     # set accel limit for V motor
            if num_motors >= 3:
                accel2_def = XH540_V_ACCEL_LIMIT_DEF_G4                         # limits for XH540 V motor in GIM03/GIM04
                accel2_hard_lim = XH540_V_ACCEL_G4_HARD_LIMIT
            else:
                accel2_def = XH540_V_ACCEL_LIMIT_DEF_G1                         # limits for XH540 V motor in GIM01
                accel2_hard_lim = XH540_V_ACCEL_G1_HARD_LIMIT

            if accel2 == 0:
                accel2 = accel2_def                                             # set to default
            if accel2_hard_lim[0] <= accel2 <= accel2_hard_lim[1]:              # check against hard limit
                GIM_MOTION[2]["accel"] = accel2
                print("*** V accel limit set ***")
            else:
                print("*** V accel limit = %g out of range %s... SKIPPING. ***" % (accel2, str(accel2_hard_lim)))

        if num_motors >= 4:                                                     # set accel limit for P motor
            if accel3 == 0:
                accel3 = XH540_P_ACCEL_LIMIT_DEF                                # set to default
            if XH540_P_ACCEL_HARD_LIMIT[0] <= accel3 <= XH540_P_ACCEL_HARD_LIMIT[1]:    # check against hard limit
                GIM_MOTION[4]["accel"] = accel3
                print("*** P accel limit set ***")
            else:
                print("*** P accel limit = %g out of range %s... SKIPPING. ***" % (accel3, str(XH540_P_ACCEL_HARD_LIMIT)))

    if gim_type == SPHERICAL:                                                   # set accel limit for SPHERICAL gimbal
        if GIM_MOTION[1]["model"] >= PH_MOTOR_CLASS:                            # set accel limit for H motor
            accel1_def = PH42_H_ACCEL_LIMIT_DEF                                 # limits for PH42 motor
            accel1_hard_lim = PH42_H_ACCEL_HARD_LIMIT
        else:
            accel1_def = XH540_H_ACCEL_LIMIT_DEF                                # limits for XH540 motor
            accel1_hard_lim = XH540_H_ACCEL_HARD_LIMIT

        if accel1 == 0:
            accel1 = accel1_def                                                 # set to default
        if accel1_hard_lim[0] <= accel1 <= accel1_hard_lim[1]:                  # check against hard limit
            GIM_MOTION[1]["accel"] = accel1
            print("*** H accel limit set ***")
        else:
            print("*** H accel limit = %g out of range %s... SKIPPING. ***" % (accel1, str(accel1_hard_lim)))

        if num_motors >= 5:                                                     # set accel limit for T motor
            if accel2 == 0:
                accel2 = XH540_T_ACCEL_LIMIT_DEF                                # set to default
            if XH540_T_ACCEL_HARD_LIMIT[0] <= accel2 <= XH540_T_ACCEL_HARD_LIMIT[1]:    # check against hard limit
                GIM_MOTION[5]["accel"] = accel2
                print("*** T accel limit set ***")
            else:
                print("*** T accel limit = %g out of range %s... SKIPPING. ***" % (accel2, str(XH540_T_ACCEL_HARD_LIMIT)))

        if num_motors >= 6:                                                     # set accel limit for Z motor
            if accel3 == 0:
                accel3 = XH540_Z_ACCEL_LIMIT_DEF                                # set to default
            if XH540_Z_ACCEL_HARD_LIMIT[0] <= accel3 <= XH540_Z_ACCEL_HARD_LIMIT[1]:    # check against hard limit
                GIM_MOTION[6]["accel"] = accel3
                print("*** Z accel limit set ***")
            else:
                print("*** Z accel limit = %g out of range %s... SKIPPING. ***" % (accel3, str(XH540_Z_ACCEL_HARD_LIMIT)))

    write_accel()                                                               # write to accel registers

    return


def get_accel(log_to_screen=True):
    """ read motor accel limit value """
    global GIM_MOTION

    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        return None, None, None

    if gim_type == HV:
        H_accel = read4(H, (ADD_ACCEL_PROFILE + ram_offset))                    # read the current motor H accel limit
        if log_to_screen:
            print("H accel is set to = %d" % H_accel)
        GIM_MOTION[1]["accel"] = H_accel
        if num_motors >= 2:
            V_accel = read4(V, ADD_ACCEL_PROFILE)                               # read the current motor V accel limit
            if log_to_screen:
                print("V accel is set to = %d" % V_accel)
            GIM_MOTION[2]["accel"] = V_accel
        else:
            if num_motors >= 3:
                V_accel = XH540_V_ACCEL_LIMIT_DEF_G4
            else:
                V_accel = XH540_V_ACCEL_LIMIT_DEF_G1

        if num_motors >= 4:
            P_accel = read4(P, ADD_ACCEL_PROFILE)                               # read the current motor P accel limit
            if log_to_screen:
                print("P accel is set to = %d" % P_accel)
            GIM_MOTION[4]["accel"] = P_accel
        else:
            P_accel = XH540_P_ACCEL_LIMIT_DEF
        accel1 = H_accel
        accel2 = V_accel
        accel3 = P_accel

    if gim_type == SPHERICAL:
        H_accel = read4(H, (ADD_ACCEL_PROFILE + ram_offset))                    # read the current motor H accel limit
        if log_to_screen:
            print("H accel is set to = %d" % H_accel)
        GIM_MOTION[1]["accel"] = H_accel
        if num_motors >= 5:
            T_accel = read4(T, ADD_ACCEL_PROFILE)                               # read the current motor T accel limit
            if log_to_screen:
                print("T accel is set to = %d" % T_accel)
            GIM_MOTION[5]["accel"] = T_accel
        else:
            T_accel = XH540_T_ACCEL_LIMIT_DEF

        if num_motors >= 6:
            Z_accel = read4(Z, ADD_ACCEL_PROFILE)                               # read the current motor Z accel limit
            if log_to_screen:
                print("Z accel is set to = %d" % Z_accel)
            GIM_MOTION[6]["accel"] = Z_accel
        else:
            Z_accel = XH540_Z_ACCEL_LIMIT_DEF
        accel1 = H_accel
        accel2 = T_accel
        accel3 = Z_accel

    return accel1, accel2, accel3


def write_accel():
    """ write accel limit settings to accel limit registers """
    global GIM_MOTION

    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        return

    if gim_type == HV:                                                          # set accel limit for HV gimbal
        H_accel = GIM_MOTION[1]["accel"]                                        # set accel limit for H motor
        write4(H, ADD_ACCEL_PROFILE + ram_offset, H_accel)                      # program register value in RAM

        if num_motors >= 2:
            V_accel = GIM_MOTION[2]["accel"]                                    # set accel limit for V motor
            write4(V, ADD_ACCEL_PROFILE, V_accel)                               # program register value in RAM

        if num_motors >= 4:
            P_accel = GIM_MOTION[4]["accel"]                                    # set accel limit for P motor
            write4(P, ADD_ACCEL_PROFILE, P_accel)                               # program register value in RAM

    if gim_type == SPHERICAL:                                                   # set accel limit for SPHERICAL gimbal
        H_accel = GIM_MOTION[1]["accel"]                                        # set accel limit for H motor
        write4(H, ADD_ACCEL_PROFILE + ram_offset, H_accel)                      # program register value in RAM

        if num_motors >= 5:
            T_accel = GIM_MOTION[5]["accel"]                                    # set accel limit for T motor
            write4(T, ADD_ACCEL_PROFILE, T_accel)                               # program register value in RAM

        if num_motors >= 6:
            Z_accel = GIM_MOTION[6]["accel"]                                    # set accel limit for Z motor
            write4(Z, ADD_ACCEL_PROFILE, Z_accel)                               # program register value in RAM

    get_accel(log_to_screen=False)                                              # check the values by reading back the registers

    return


def set_anglelim(anglelim1=[0, 0], anglelim2=[0, 0], anglelim3=[0, 0]):
    """ change angle limit of the motors. returns ok=1 if all limits are within bounds. """

    ok = 1
    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        ok = 0
        return ok

    if gim_type == HV:                                                          # set angle limit for H motor
        if ok and (H_ANGLE_HARD_LIMIT[0] <= anglelim1[0] <= 0) and \
                (0 <= anglelim1[1] <= H_ANGLE_HARD_LIMIT[1]):
            GIM_MOTION[1]["anglelim"] = [min(convertpostoangle(H, convertangletopos(H, anglelim1[0])), anglelim1[0]),
                                         max(convertpostoangle(H, convertangletopos(H, anglelim1[1])), anglelim1[1])]
        else:
            ok = 0

        if num_motors >= 2:                                                     # set angle limit for V motor
            if ok and (V_ANGLE_HARD_LIMIT[0] <= anglelim2[0] <= 0) and \
                    (0 <= anglelim2[1] <= V_ANGLE_HARD_LIMIT[1]):
                GIM_MOTION[2]["anglelim"] = [min(convertpostoangle(V, convertangletopos(V, anglelim2[0])), anglelim2[0]),
                                             max(convertpostoangle(V, convertangletopos(V, anglelim2[1])), anglelim2[1])]
            else:
                ok = 0

        if num_motors >= 4:                                                     # set angle limit for P motor
            if ok and (P_ANGLE_HARD_LIMIT[0] <= anglelim3[0] <= 0) and \
                    (0 <= anglelim3[1] <= P_ANGLE_HARD_LIMIT[1]):
                GIM_MOTION[4]["anglelim"] = [min(convertpostoangle(P, convertangletopos(P, anglelim3[0])), anglelim3[0]),
                                             max(convertpostoangle(P, convertangletopos(P, anglelim3[1])), anglelim3[1])]
            else:
                ok = 0

    if gim_type == SPHERICAL:                                                   # set angle limit for H motor
        if ok and (H_ANGLE_HARD_LIMIT[0] <= anglelim1[0] <= 0) and \
                (0 <= anglelim1[1] <= H_ANGLE_HARD_LIMIT[1]):
            GIM_MOTION[1]["anglelim"] = [min(convertpostoangle(H, convertangletopos(H, anglelim1[0])), anglelim1[0]),
                                         max(convertpostoangle(H, convertangletopos(H, anglelim1[1])), anglelim1[1])]
        else:
            ok = 0

        if num_motors >= 5:                                                     # set angle limit for T motor
            if ok and (T_ANGLE_HARD_LIMIT[0] <= anglelim2[0] <= 0) and \
                    (0 <= anglelim2[1] <= T_ANGLE_HARD_LIMIT[1]):
                GIM_MOTION[5]["anglelim"] = [min(convertpostoangle(T, convertangletopos(T, anglelim2[0])), anglelim2[0]),
                                             max(convertpostoangle(T, convertangletopos(T, anglelim2[1])), anglelim2[1])]
            else:
                ok = 0

        if num_motors >= 6:                                                     # set angle limit for Z motor
            if ok and (Z_ANGLE_HARD_LIMIT[0] <= anglelim3[0] <= 0) and \
                    (0 <= anglelim3[1] <= Z_ANGLE_HARD_LIMIT[1]):
                GIM_MOTION[6]["anglelim"] = [min(convertpostoangle(Z, convertangletopos(Z, anglelim3[0])), anglelim3[0]),
                                             max(convertpostoangle(Z, convertangletopos(Z, anglelim3[1])), anglelim3[1])]
            else:
                ok = 0

    return ok


def get_anglelim(log_to_screen=True):
    """ display motor angle limits """
    global GIM_MOTION

    if gim_type != GIM_MOTION["gim_type"] or num_motors != GIM_MOTION["num_motors"]:
        print("gimbal motion parameters mismatch!!")
        print("physical gimbal --> gim_type=%d, num_motors=%d" % (gim_type, num_motors))
        print(" motor settings --> gim_type=%d, num_motors=%d" % (GIM_MOTION["gim_type"], GIM_MOTION["num_motors"]))
        return None, None, None

    if gim_type == HV:
        H_anglelim = GIM_MOTION[1]["anglelim"]                                  # get H angle limit
        if log_to_screen:
            print("H angle limit is set to %s" % str(H_anglelim))
        if num_motors >= 2:
            V_anglelim = GIM_MOTION[2]["anglelim"]                              # get V angle limit
            if log_to_screen:
                print("V angle limit is set to %s" % str(V_anglelim))
        else:
            V_anglelim = [0, 0]

        if num_motors >= 4:
            P_anglelim = GIM_MOTION[4]["anglelim"]                              # get P angle limit
            if log_to_screen:
                print("P angle limit is set to %s" % str(P_anglelim))
        else:
            P_anglelim = [0, 0]
        anglelim1 = H_anglelim
        anglelim2 = V_anglelim
        anglelim3 = P_anglelim

    if gim_type == SPHERICAL:
        H_anglelim = GIM_MOTION[1]["anglelim"]                                  # get H angle limit
        if log_to_screen:
            print("H angle limit is set to %s" % str(H_anglelim))
        if num_motors >= 5:
            T_anglelim = GIM_MOTION[5]["anglelim"]                              # get T angle limit
            if log_to_screen:
                print("T angle limit is set to %s" % str(T_anglelim))
        else:
            T_anglelim = [0, 0]

        if num_motors >= 6:
            Z_anglelim = GIM_MOTION[6]["anglelim"]                              # get Z angle limit
            if log_to_screen:
                print("Z angle limit is set to %s" % str(Z_anglelim))
        else:
            Z_anglelim = [0, 0]
        anglelim1 = H_anglelim
        anglelim2 = T_anglelim
        anglelim3 = Z_anglelim

    return anglelim1, anglelim2, anglelim3


def setoffset(motor, zero=None):
    """ write the current position to motor eeprom to set HOME """

    if zero is None:                                                            # if zero is not explicitly specified, use global value
        if motor == H:
            zero = h_zero
        else:
            zero = XH540_0

    if motor == H:
        offset = read4(H, ADD_HOME_OFFSET)
        print("current offset= %d" % offset)
        offset = offset - current_pos(H, 1) + zero                              # we want Horizontal center at 2048 or 0 (depending on motor type)
        if abs(offset) <= max_offset:
            disable_torque(H)                                                   # access flash
            write4(H, ADD_HOME_OFFSET, offset)
            print("new offset= %d" % offset)
            offset = read4(H, ADD_HOME_OFFSET)
            print("check new offset= %d" % offset)
            enable_torque(H)                                                    # allow motors to move
        else:
            print("offset out of range, reset the offset")
    elif motor <= num_motors:
        offset = read4(motor, ADD_HOME_OFFSET)
        print("current offset= %d" % offset)
        drmode = read1(motor, ADD_DRIVE_MODE)                                   # read drive mode (direction)

        # normal mode:  pos_actual = goalpos - homeoffset
        # reverse mode: pos_actual = goalpos + homeoffset
        if (drmode & 1) == 1:
            offset = offset - (zero - current_pos(motor, 1))                    # reverse drive
        else:
            offset = offset - (current_pos(motor, 1) - zero)                    # normal drive

        if abs(offset) <= XH540_MAX_OFFSET:
            disable_torque(motor)                                               # access flash
            write4(motor, ADD_HOME_OFFSET, offset)
            print("new offset= %d" % offset)
            offset = read4(motor, ADD_HOME_OFFSET)
            print("check new offset= %d" % offset)
            enable_torque(motor)                                                # allow motors to move
        else:
            print("offset out of range, reset the offset")
    else:
        print("ERROR: motor number out of range")
    return


def setoffset_all():
    """ write the current position for all motors to motor eeprom to set HOME """
    if gim_type == HV:
        setoffset(H, h_zero)
        if num_motors >= 2:
            setoffset(V, XH540_0)
            if num_motors >= 4:
                setoffset(P, XH540_0)
    elif gim_type == SPHERICAL:
        setoffset(H, h_zero)
        if num_motors >= 5:
            setoffset(T, XH540_0)
            if num_motors >= 6:
                setoffset(Z, XH540_0)
    return


def print_offset(motor):
    """ reads and prints the LED status, offset, position, and drive mode for the motor """
    if motor == H:
        ram = ram_offset                                                        # set ram offset based on H motor type
    else:
        ram = 0

    led = read1(motor, ADD_LED + ram)                                           # read LED status
    homeoffset = read4(motor, ADD_HOME_OFFSET)                                  # read current offset from flash
    goalpos = read4(motor, ADD_GOAL_POSITION + ram)                             # read current motor position
    drmode = read1(motor, ADD_DRIVE_MODE)                                       # read current drive mode (direction) from flash
    print("motor=%1d   led=%1d   offset=%6d   goalpos=%6d   drmode=%1d" % (motor, led, homeoffset, goalpos, drmode))
    return led, homeoffset, goalpos, drmode


def print_offset_all():
    """ print LED status, offset, position, and drive mode for all motors """
    print("*************************************************************")
    if gim_type == HV:
        print_offset(H)
        if num_motors >= 2:
            print_offset(V)
            if num_motors >= 4:
                print_offset(P)
    elif gim_type == SPHERICAL:
        print_offset(H)
        if num_motors >= 5:
            print_offset(T)
            if num_motors >= 6:
                print_offset(Z)
    print("*************************************************************")
    return


def realign(motor):
    """ re-aligns the motor at boot-up, if needed """
    if motor == H and base_type == 4:
        zero = PH42_0
        resolution = PH42_RES
        max_off = PH42_MAX_OFFSET
        ram = ram_offset
    else:
        zero = XH540_0
        resolution = XH540_RES
        max_off = XH540_MAX_OFFSET
        ram = 0

    led, homeoffset, goalpos, drmode = print_offset(motor)                      # print offset and read LED status - check for power cycle
    if led == 1:
        print("power cycle not detected --> no alignment needed")               # power has been maintained, no need to re-align
    else:
        print("power cycle detected --> re-alignment check needed")             # power has been removed, check for realign

        # normal mode:  pos_actual = goalpos - homeoffset
        # reverse mode: pos_actual = goalpos + homeoffset

        delta = zero - int(resolution / 2)                                      # determine delta to center zero at resolution/2
        pos_valid = (goalpos - delta) % resolution + delta                      # ensure zero-res/2 < pos_valid < zero+res/2
        pos_delta = pos_valid - goalpos                                         # determine change needed to goalpos to make valid
        if (drmode & 1) == 1:                                                   # check bit0 of drive mode
            homeoffset_delta = -1 * pos_delta                                   # reverse drive
        else:
            homeoffset_delta = pos_delta                                        # normal drive

        new_homeoffset = int(homeoffset + homeoffset_delta)                     # compute change needed to offset

        if new_homeoffset > max_off or new_homeoffset < -1*max_off:             # let's check that our offset is not maxed out
            print("WARNING ------ offset overun need to reset offset using menu-------")
        elif homeoffset_delta == 0:                                             # no change is needed
            print("-> no compensation needed for motor %d" % motor)
        else:                                                                   # offset is not maxed out and we need to make a change
            print("-> goal position outside of valid range")
            print("-> updating offset")
            disable_torque(motor)                                               # disable torque opens the motor flash memory for update
            write4(motor, ADD_HOME_OFFSET, new_homeoffset)                      # change offset accordingly
            enable_torque(motor)                                                # close the flash write access
            print_offset(motor)                                                 # display new offset and position

        print("-> motor=%d LED set to 1" % motor)                               # debug
        write1(motor, ADD_LED + ram, 1)                                         # enable LED bit

    return


def resetoffset():
    """ reset the offset position in case it reach the maximum number of turns """
    disable_torque(H)
    offseth = read4(H, ADD_HOME_OFFSET)
    print("current H offset= %d" % offseth)
    write4(H, ADD_HOME_OFFSET, 0)
    enable_torque(H)
    print("*** offset reset done on H")
    if gim_type == HV:
        if num_motors >= 2:
            disable_torque(V)
            offsetv = read4(V, ADD_HOME_OFFSET)
            print("current V offset= %d" % offsetv)
            write4(V, ADD_HOME_OFFSET, 0)
            enable_torque(V)
            print("*** offset reset done on V")
            if num_motors >= 4:
                disable_torque(P)
                offsetp = read4(P, ADD_HOME_OFFSET)
                print("current P offset= %d" % offsetp)
                write4(P, ADD_HOME_OFFSET, 0)
                enable_torque(P)
                print("*** offset reset done on P")
    if gim_type == SPHERICAL:
        if num_motors >= 5:
            disable_torque(T)
            offsett = read4(T, ADD_HOME_OFFSET)
            print("current T offset= %d" % offsett)
            write4(T, ADD_HOME_OFFSET, 0)
            enable_torque(T)
            print("*** offset reset done on T")
            if num_motors >= 6:
                disable_torque(Z)
                offsetz = read4(Z, ADD_HOME_OFFSET)
                print("current Z offset= %d" % offsetz)
                write4(Z, ADD_HOME_OFFSET, 0)
                enable_torque(Z)
                print("*** offset reset done on Z")
    return


def changerate(rate):
    """ change all motors communication baud rate, this is used for MACos which does not support 1Mbps """
    disable_torque(H)
    write1(H, ADD_BAUD_RATE, rate)
    if gim_type == HV:
        if num_motors >= 2:
            disable_torque(V)
            write1(V, ADD_BAUD_RATE, rate)
            if num_motors >= 3:
                disable_torque(R)
                write1(R, ADD_BAUD_RATE, rate)
                if num_motors >= 4:
                    disable_torque(P)
                    write1(P, ADD_BAUD_RATE, rate)
    elif gim_type == SPHERICAL:
        if num_motors >= 5:
            disable_torque(T)
            write1(T, ADD_BAUD_RATE, rate)
            if num_motors >= 6:
                disable_torque(Z)
                write1(Z, ADD_BAUD_RATE, rate)
                write1(Z, ADD_BAUD_RATE, rate)

    print("*** rate changed, port is closed, BAUDRATE memory is saved")
    print("*** restart mbx.py")
    sys.exit()
    return


# ===========================================
# ============= GIMBAL MOVEMENT =============
# ===========================================

def convertangletopos(motor, angle=None):
    """ convert angle in degree to motor position """
    if angle is None:
        pos = None
    else:
        if motor == H:
            pos = int(round((angle*base_res*base_ratio)/360.0))+h_zero          # H motor has an additional 5x gear ratio in gimbal
        elif motor == V:
            pos = int(round((angle*XH540_RES*XH540_V_RATIO)/360.0))+XH540_0     # V motor is in 1x direct drive
        elif motor == P:
            pos = int(round(angle*XH540_RES*XH540_P_RATIO)/360.0)+XH540_0       # P motor has an additional 2.4x (120/50) gear ratio in gimbal
        elif motor == T:
            pos = int(round(angle*XH540_RES*XH540_T_RATIO)/360.0)+XH540_0       # T motor has an additional 2.4x (120/50) gear ratio in gimbal
        elif motor == Z:
            pos = int(round(angle*XH540_RES*XH540_Z_RATIO)/360.0)+XH540_0       # Z motor has an additional 2x gear ratio in gimbal
        elif motor == TH:
            pos = convertangletopos(H, angle)
        elif motor == PH:                                                       # angle = [PHI, DEL_PHI] when specifying PH angle
            pos = [convertangletopos(T, angle[0]), convertangletopos(Z, angle[0]-angle[1])]     # pos = [posT, posZ]
        else:
            print("position error")
            pos = None
    return pos


def convertpostoangle(motor, pos=None):
    """ convert reported motor position to absolute angle """
    if pos is None:
        angle = None
    else:
        if motor == H:
            angle = ((pos-h_zero)*360.0/(base_res*base_ratio))                  # H motor has an additional 5x gear ratio in gimbal
            angle = round(angle, 3)
        elif motor == V:
            angle = ((pos-XH540_0)*360.0/(XH540_RES*XH540_V_RATIO))             # V motor is in 1x direct drive
            angle = round(angle, 3)
        elif motor == P:
            angle = ((pos-XH540_0)*360.0/(XH540_RES*XH540_P_RATIO))             # P motor has an additional 2.4x (120/50) gear ratio in gimbal
            angle = round(angle, 3)
        elif motor == T:
            angle = ((pos-XH540_0)*360.0/(XH540_RES*XH540_T_RATIO))             # T motor has an additional 2.4x (120/50) gear ratio in gimbal
            angle = round(angle, 3)
        elif motor == Z:
            angle = ((pos-XH540_0)*360.0/(XH540_RES*XH540_Z_RATIO))             # Z motor has an additional 2x gear ratio in gimbal
            angle = round(angle, 3)
        elif motor == TH:
            angle = convertpostoangle(H, pos)
            angle = round(angle, 3)
        elif motor == PH:
            Tangle = convertpostoangle(T, pos[0])                               # pos = [posT, posZ]
            Zangle = convertpostoangle(Z, pos[1])
            angle = [Tangle, Tangle - Zangle]                                   # angle = [PHI, DEL_PHI] when specifying PH angle
        else:
            print("position error")
            angle = None
    return angle


def current_pos(motor, log):
    """ read the absolute current position of the motor """
    # this function can be called while the motor is moving
    # if a settled position is desired, use wait_stop_moving() before calling this function
    if motor % 100 <= num_motors:
        if motor == H:
            curpos = read4(motor, ADD_PRESENT_POS + ram_offset)
            offset = read4(motor, ADD_HOME_OFFSET)
        elif motor == TH:
            curpos = read4(H, ADD_PRESENT_POS + ram_offset)
            offset = read4(H, ADD_HOME_OFFSET)
        elif motor == PH:
            curposT = read4(T, ADD_PRESENT_POS)
            offsetT = read4(T, ADD_HOME_OFFSET)
            if num_motors >= 6:
                curposZ = read4(Z, ADD_PRESENT_POS)
                offsetZ = read4(Z, ADD_HOME_OFFSET)
            else:                                                               # special case for GIM05_FIXED
                curposZ = convertangletopos(Z, 0)
                offsetZ = 0
            curpos = [curposT, curposZ]
            offset = [offsetT, offsetZ]
        else:
            curpos = read4(motor, ADD_PRESENT_POS)
            offset = read4(motor, ADD_HOME_OFFSET)
        if log == 0:
            cur_angle = convertpostoangle(motor, curpos)
            print("Current position for motor %d  is =  %s degree, Position is : %s steps,  Offset is : %s" % (motor, str(cur_angle), str(curpos), str(offset)))
    else:
        if motor == TH:
            curpos = convertangletopos(H, 0)
        elif motor == PH:
            curpos = [convertangletopos(T, 0), convertangletopos(Z, 0)]
        else:
            curpos = convertangletopos(motor, 0)
    return curpos


def goal_pos(motor, log):
    """ read the goal position of the motor """
    if motor % 100 <= num_motors:
        if motor == H:
            cur_goal_pos = read4(motor, ADD_GOAL_POSITION + ram_offset)
            offset = read4(motor, ADD_HOME_OFFSET)
        elif motor == TH:
            cur_goal_pos = read4(H, ADD_GOAL_POSITION + ram_offset)
            offset = read4(H, ADD_HOME_OFFSET)
        elif motor == PH:
            cur_goal_posT = read4(T, ADD_GOAL_POSITION)
            offsetT = read4(T, ADD_HOME_OFFSET)
            if num_motors >= 6:
                cur_goal_posZ = read4(Z, ADD_GOAL_POSITION)
                offsetZ = read4(Z, ADD_HOME_OFFSET)
            else:                                                               # special case for GIM05_FIXED
                cur_goal_posZ = convertangletopos(Z, 0)
                offsetZ = 0
            cur_goal_pos = [cur_goal_posT, cur_goal_posZ]
            offset = [offsetT, offsetZ]
        else:
            cur_goal_pos = read4(motor, ADD_GOAL_POSITION)
            offset = read4(motor, ADD_HOME_OFFSET)
    else:
        if motor == TH:
            cur_goal_pos = convertangletopos(H, 0)
            offset = 0
        elif motor == PH:
            cur_goal_posT = convertangletopos(T, 0)
            cur_goal_posZ = convertangletopos(Z, 0)
            cur_goal_pos = [cur_goal_posT, cur_goal_posZ]
            offset = [0, 0]
        else:
            cur_goal_pos = convertangletopos(motor, 0)
            offset = 0

    goal_angle = convertpostoangle(motor, cur_goal_pos)
    if log == 0:
        print("Goal position for motor %d  is =  %s degree, Position is : %s steps,  Offset is : %s" % (motor, str(goal_angle), str(cur_goal_pos), str(offset)))
    return cur_goal_pos


def getposition(log=0):
    """ get absolute position of all motors """
    if gim_type == HV:
        pos1 = current_pos(H, log)
        pos2 = current_pos(V, log)
        pos3 = current_pos(P, log)
    if gim_type == SPHERICAL:
        pos1 = current_pos(TH, log)
        pos2 = current_pos(PH, log)
        pos3 = None
    print_offset_all()

    return pos1, pos2, pos3


def check_is_moving(debug=DEBUG_MOVING):
    """ non-blocking check if all motors are not moving """
    global ram_offset
    global base_pos_acc_thresh
    global XH540_POS_ACC_THRESH

    if debug:
        print("check_moving() called...")

    goalH = read4(H,ADD_GOAL_POSITION + ram_offset)                             # read H goal position
    if gim_type == HV:
        if num_motors >= 2:
            goalV = read4(V,ADD_GOAL_POSITION)                                  # read V goal position
            if num_motors >= 4:
                goalP = read4(P,ADD_GOAL_POSITION)                              # read P goal position
    if gim_type == SPHERICAL:
        if num_motors >= 5:
            goalT = read4(T,ADD_GOAL_POSITION)                                  # read T goal position
            if num_motors >= 6:
                goalZ = read4(Z,ADD_GOAL_POSITION)                              # read Z goal position

    # check if motor H reports not MOVING (based on velocity)
    ismovingH = (read1(H, ADD_MOVING + ram_offset) > 0)                         # check H MOVING register
    if debug:
        print("H is moving = %d" % ismovingH)
    if ismovingH:
        return True                                                             # return if H is moving

    # wait until motor H reaches goal position (based on target and current position)
    # check if abs(pres_pos - goal) <= threshold
    pres_posH = read4(H, ADD_PRESENT_POS + ram_offset)
    if debug:
        print("H goal position / present / error = %d / %d / %d" % (goalH, pres_posH, goalH - pres_posH))
    not_reachedH = (abs(pres_posH - goalH) > base_pos_acc_thresh)
    if not_reachedH:
        return True                                                             # return if H is moving

    if gim_type == HV:
        if num_motors >= 2:                                                     # if we are not GIM1D
            # check if motor V reports not MOVING (based on velocity)
            ismovingV = (read1(V, ADD_MOVING) > 0)                              # check V MOVING register
            if debug:
                print("V is moving = %d" % ismovingV)
            if ismovingV:
                return True                                                     # return if V is moving

            # wait until motor V reaches goal position (based on target and current position)
            # check if abs(pres_pos - goal) <= threshold
            pres_posV = read4(V, ADD_PRESENT_POS)
            if debug:
                print("V goal position / present / error = %d / %d / %d" % (goalV, pres_posV, goalV - pres_posV))
            not_reachedV = (abs(pres_posV - goalV) > XH540_POS_ACC_THRESH)
            if not_reachedV:
                return True                                                     # return if V is moving

        if num_motors >= 4:                                                     # if we are a GIM04
            # wait until motor P reports not MOVING (based on velocity)
            ismovingP = (read1(P, ADD_MOVING) > 0)                              # check P MOVING register
            if debug:
                print("P is moving = %d" % ismovingP)
            if ismovingP:
                return True                                                     # return if P is moving

            # wait until motor P reaches goal position (based on target and current position)
            # check if abs(pres_pos - goal) <= threshold
            pres_posP = read4(P, ADD_PRESENT_POS)
            if debug:
                print("P goal position / present / error = %d / %d / %d" % (goalP, pres_posP, goalP - pres_posP))
            not_reachedP = (abs(pres_posP - goalP) > XH540_POS_ACC_THRESH)
            if not_reachedP:
                return True                                                     # return if P is moving

    if gim_type == SPHERICAL:
        if num_motors >= 5:                                                     # if we are not GIM1D
            # wait until motor T reports not MOVING (based on velocity)
            ismovingT = (read1(T, ADD_MOVING) > 0)                              # check T MOVING register
            if debug:
                print("T is moving = %d" % ismovingT)
            if ismovingT:
                return True                                                     # return if T is moving

            # wait until motor T reaches goal position (based on target and current position)
            # check if abs(pres_pos - goal) <= threshold
            pres_posT = read4(T, ADD_PRESENT_POS)
            if debug:
                print("T goal position / present / error = %d / %d / %d" % (goalT, pres_posT, goalT - pres_posT))
            not_reachedT = (abs(pres_posT - goalT) > XH540_POS_ACC_THRESH)
            if not_reachedT:
                return True                                                     # return if T is moving

        if num_motors >= 6:                                                     # if we are a GIM05
            # wait until motor Z reports not MOVING (based on velocity)
            ismovingZ = (read1(Z, ADD_MOVING) > 0)                              # check Z MOVING register
            if debug:
                print("Z is moving = %d" % ismovingZ)
            if ismovingZ:
                return True                                                     # return if Z is moving

            # wait until motor Z reaches goal position (based on target and current position)
            # check if abs(pres_pos - goal) <= threshold
            pres_posZ = read4(Z, ADD_PRESENT_POS)
            if debug:
                print("Z goal position / present / error = %d / %d / %d" % (goalZ, pres_posZ, goalZ - pres_posZ))
            not_reachedZ = (abs(pres_posZ - goalZ) > XH540_POS_ACC_THRESH)
            if not_reachedZ:
                return True                                                     # return if Z is moving

    return False


def wait_stop_moving(accuracy="HIGH", debug=DEBUG_MOVING):
    """ wait until all motors are not moving """
    global ram_offset
    global base_pos_acc_thresh
    global XH540_POS_ACC_THRESH

    INIT_DELAY = 0.15                                                           # delay before any register is read (to avoid ismoving reporting incorrectly)
    LOOP_DELAY = 0.05                                                           # delay in while loop polling status

    if debug:
        print("wait_stop_moving() called...")

    time.sleep(INIT_DELAY)

    if accuracy == "HIGH" or accuracy == "VERY HIGH":
        goalH = read4(H,ADD_GOAL_POSITION + ram_offset)                         # read H goal position
        if gim_type == HV:
            if num_motors >= 2:
                goalV = read4(V,ADD_GOAL_POSITION)                              # read V goal position
                if num_motors >= 4:
                    goalP = read4(P,ADD_GOAL_POSITION)                          # read P goal position
        if gim_type == SPHERICAL:
            if num_motors >= 5:
                goalT = read4(T,ADD_GOAL_POSITION)                              # read T goal position
                if num_motors >= 6:
                    goalZ = read4(Z,ADD_GOAL_POSITION)                          # read Z goal position

    # wait until motor H reports not MOVING (based on velocity)
    ismovingH = True
    while ismovingH:
        ismovingH = (read1(H, ADD_MOVING + ram_offset) > 0)                     # check H MOVING register
        if debug:
            print("H is moving = %d" % ismovingH)
        if ismovingH:
            time.sleep(LOOP_DELAY)                                              # delay before polling again

    # wait until motor H reaches goal position (based on target and current position)
    if accuracy == "HIGH" or accuracy == "VERY HIGH":
        not_reachedH = True
        while not_reachedH:                                                     # for higher accuracy, loop until (pres_pos - goal) <= threshold
            pres_posH = read4(H, ADD_PRESENT_POS + ram_offset)
            if debug:
                print("H goal position / present / error = %d / %d / %d" % (goalH, pres_posH, goalH - pres_posH))
            not_reachedH = (abs(pres_posH - goalH) > base_pos_acc_thresh)
            if not_reachedH:
                time.sleep(LOOP_DELAY)                                          # delay before polling again

    if gim_type == HV:
        if num_motors >= 2:                                                     # if we are not GIM1D
            # wait until motor V reports not MOVING (based on velocity)
            ismovingV = True
            while ismovingV:
                ismovingV = (read1(V, ADD_MOVING) > 0)                          # check V MOVING register
                if debug:
                    print("V is moving = %d" % ismovingV)
                if ismovingV:
                    time.sleep(LOOP_DELAY)                                      # delay before polling again

            # wait until motor V reaches goal position (based on target and current position)
            if accuracy == "HIGH" or accuracy == "VERY HIGH":
                not_reachedV = True
                while not_reachedV:                                             # for higher accuracy, loop until (pres_pos - goal) <= threshold
                    pres_posV = read4(V, ADD_PRESENT_POS)
                    if debug:
                        print("V goal position / present / error = %d / %d / %d" % (goalV, pres_posV, goalV - pres_posV))
                    not_reachedV = (abs(pres_posV - goalV) > XH540_POS_ACC_THRESH)
                    if not_reachedV:
                        time.sleep(LOOP_DELAY)                                  # delay before polling again

        if num_motors >= 4:                                                     # if we are a GIM04
            # wait until motor P reports not MOVING (based on velocity)
            ismovingP = 1
            while ismovingP:
                ismovingP = (read1(P, ADD_MOVING) > 0)                          # check P MOVING register
                if debug:
                    print("P is moving = %d" % ismovingP)
                if ismovingP:
                    time.sleep(LOOP_DELAY)                                      # delay before polling again

            # wait until motor P reaches goal position (based on target and current position)
            if accuracy == "HIGH" or accuracy == "VERY HIGH":
                not_reachedP = True
                while not_reachedP:                                             # for higher accuracy, loop until (pres_pos - goal) <= threshold
                    pres_posP = read4(P, ADD_PRESENT_POS)
                    if debug:
                        print("P goal position / present / error = %d / %d / %d" % (goalP, pres_posP, goalP - pres_posP))
                    not_reachedP = (abs(pres_posP - goalP) > XH540_POS_ACC_THRESH)
                    if not_reachedP:
                        time.sleep(LOOP_DELAY)                                  # delay before polling again

    if gim_type == SPHERICAL:
        if num_motors >= 5:                                                     # if we are not GIM1D
            # wait until motor T reports not MOVING (based on velocity)
            ismovingT = True
            while ismovingT:
                ismovingT = (read1(T, ADD_MOVING) > 0)                          # check T MOVING register
                if debug:
                    print("T is moving = %d" % ismovingT)
                if ismovingT:
                    time.sleep(LOOP_DELAY)                                      # delay before polling again

            # wait until motor T reaches goal position (based on target and current position)
            if accuracy == "HIGH" or accuracy == "VERY HIGH":
                not_reachedT = True
                while not_reachedT:                                             # for higher accuracy, loop until (pres_pos - goal) <= threshold
                    pres_posT = read4(T, ADD_PRESENT_POS)
                    if debug:
                        print("T goal position / present / error = %d / %d / %d" % (goalT, pres_posT, goalT - pres_posT))
                    not_reachedT = (abs(pres_posT - goalT) > XH540_POS_ACC_THRESH)
                    if not_reachedT:
                        time.sleep(LOOP_DELAY)                                  # delay before polling again

        if num_motors >= 6:                                                     # if we are a GIM05
            # wait until motor Z reports not MOVING (based on velocity)
            ismovingZ = 1
            while ismovingZ:
                ismovingZ = (read1(Z, ADD_MOVING) > 0)                          # check Z MOVING register
                if debug:
                    print("Z is moving = %d" % ismovingZ)
                if ismovingZ:
                    time.sleep(LOOP_DELAY)                                      # delay before polling again

            # wait until motor Z reaches goal position (based on target and current position)
            if accuracy == "HIGH" or accuracy == "VERY HIGH":
                not_reachedZ = True
                while not_reachedZ:                                             # for higher accuracy, loop until (pres_pos - goal) <= threshold
                    pres_posZ = read4(Z, ADD_PRESENT_POS)
                    if debug:
                        print("Z goal position / present / error = %d / %d / %d" % (goalZ, pres_posZ, goalZ - pres_posZ))
                    not_reachedZ = (abs(pres_posZ - goalZ) > XH540_POS_ACC_THRESH)
                    if not_reachedZ:
                        time.sleep(LOOP_DELAY)                                  # delay before polling again

    return


def move_pos(motor, pos):
    """ Makes a single motor move to a given absolute position.
    There is no error checking for valid position.
    Use jump_angle if position validity checking is required.
    Does not wait for motor to stop moving before returning """
    ok = 1
    if motor == H:
        write4(H, ADD_GOAL_POSITION + ram_offset, int(pos))                     # move to H goal position
    elif motor == V:
        if num_motors >= 2:
            write4(V, ADD_GOAL_POSITION, int(pos))                              # move to V goal position if motor exists
        else:
            if convertpostoangle(V, pos) != 0:                                  # if V motor does not exist, and try to move to non-zero angle, print WARNING
                print("WARNING: Trying to move Motor V that does not exist")
                ok = 0
    elif motor == P:
        if num_motors >= 4:
            write4(P, ADD_GOAL_POSITION, int(pos))                              # move to P goal position if motor exists
        else:
            if convertpostoangle(P, pos) != 0:                                  # if P motor does not exist, and try to move to non-zero angle, print WARNING
                print("WARNING: Trying to move Motor P that does not exist")
                ok = 0
    elif motor == T:
        if num_motors >= 5:
            write4(T, ADD_GOAL_POSITION, int(pos))                              # move to T goal position if motor exists
        else:
            if convertpostoangle(T, pos) != 0:                                  # if T motor does not exist, and try to move to non-zero angle, print WARNING
                print("WARNING: Trying to move Motor T that does not exist")
                ok = 0
    elif motor == Z:
        if num_motors >= 6:
            write4(Z, ADD_GOAL_POSITION, int(pos))                              # move to Z goal position if motor exists
        else:
            if convertpostoangle(Z, pos) != 0:                                  # if Z motor does not exist, and try to move to non-zero angle, print WARNING
                print("WARNING: Trying to move Motor Z that does not exist")
                ok = 0
    elif motor == TH:
        ok = move_pos(H, pos)
    elif motor == PH:
        tpos = pos[0]
        zpos = pos[1]
        ok = move_pos(T, tpos) and move_pos(Z, zpos)
    else:
        print("WARNING: Invalid motor")
        ok = 0
    return ok


def move_angle(hang=None, vang=None, pang=None, accuracy="HIGH", thang=None, phang=None, tang=None, zang=None, checkonly=False):
    """ makes all motors move to a given absolute angle for HV or SPHERICAL gimbal """
    global OVERSHOOT_ANG
    global GIM_MOTION

    ok = 1
    if gim_type == HV:
        if hang is not None:
            h_final_pos = convertangletopos(H, hang)                                                # calculate H goal position
        else:
            h_final_pos = goal_pos(H, 1)                                                            # otherwise, do not change H goal position
        overshoot_H = convertangletopos(H, OVERSHOOT_ANG) - convertangletopos(H, 0)                 # convert overshoot angle to motor position step
        h_limit = GIM_MOTION[1]["anglelim"]

        if num_motors >= 2:
            if vang is not None:
                v_final_pos = convertangletopos(V, vang)                                            # calculate V goal position
            else:
                v_final_pos = goal_pos(V, 1)                                                        # otherwise, do not change V goal position
            overshoot_V = convertangletopos(V, OVERSHOOT_ANG) - convertangletopos(V, 0)             # convert overshoot angle to motor position step
            v_limit = GIM_MOTION[2]["anglelim"]
        else:
            v_final_pos = convertangletopos(V, 0)
            v_limit = [0, 0]

        if num_motors >= 4:
            if pang is not None:
                p_final_pos = convertangletopos(P, pang)                                            # calculate P goal position
            else:
                p_final_pos = goal_pos(P, 1)                                                        # otherwise, do not change P goal position
            overshoot_P = convertangletopos(P, OVERSHOOT_ANG) - convertangletopos(P, 0)             # convert overshoot angle to motor position step
            p_limit = GIM_MOTION[4]["anglelim"]
        else:
            p_final_pos = convertangletopos(P, 0)
            p_limit = [0, 0]

        if not(h_limit[0] <= convertpostoangle(H, h_final_pos) <= h_limit[1]):                      # check that H angle is within H angle limits
            ok = False
            print("H motor angle target = %0.4f is out of range %s" % (hang, str(h_limit)))
        if not(v_limit[0] <= convertpostoangle(V, v_final_pos) <= v_limit[1]):                      # check that V angle is within V angle limits
            ok = False
            print("V motor angle target = %0.4f is out of range %s" % (vang, str(v_limit)))
        if not (p_limit[0] <= convertpostoangle(P, p_final_pos) <= p_limit[1]):                     # check that P angle is within P angle limits
            ok = False
            print("P motor angle target = %0.4f is out of range %s" % (pang, str(p_limit)))

        if not checkonly:
            if not ok:
                print("*** Movement cancelled ***")                                                 # do not move if out of range
            else:
                if accuracy == "VERY HIGH":
                    h_cur_goal_pos = goal_pos(H, 1)
                    if h_final_pos != h_cur_goal_pos:                                               # if need to move H
                        ok = ok and move_pos(H, h_final_pos - overshoot_H)                          # for very high accuracy, overshoot H goal position first
                    if num_motors >= 2:
                        v_cur_goal_pos = goal_pos(V, 1)
                        if v_final_pos != v_cur_goal_pos:                                           # if need to move V
                            ok = ok and move_pos(V, v_final_pos - overshoot_V)                      # for very high accuracy, overshoot V goal position first
                        if num_motors >= 4:
                            p_cur_goal_pos = goal_pos(P, 1)
                            if p_final_pos != p_cur_goal_pos:                                       # if need to move P
                                ok = ok and move_pos(P, p_final_pos - overshoot_P)                  # for very high accuracy, overshoot P goal position first
                    wait_stop_moving(accuracy)                                                      # wait for all motors to stop moving

                ok = ok and move_pos(H, h_final_pos)                                                # go to final H position
                if num_motors >= 2:
                    ok = ok and move_pos(V, v_final_pos)                                            # go to final V position
                    if num_motors >= 4:
                        ok = ok and move_pos(P, p_final_pos)                                        # go to final P position
                wait_stop_moving(accuracy)                                                          # wait for all motors to stop moving

    elif gim_type == SPHERICAL:
        if thang is not None:
            th_final_pos = convertangletopos(TH, thang)                                             # calculate TH goal position
        else:
            th_final_pos = goal_pos(TH, 1)                                                          # otherwise, do not change TH goal position
        overshoot_TH = convertangletopos(TH, OVERSHOOT_ANG) - convertangletopos(TH, 0)              # convert overshoot angle to motor position step
        th_limit = GIM_MOTION[1]["anglelim"]

        if num_motors < 6:
            if phang is not None:
                tang = phang[0]
                phang = None
            if zang is not None:
                print("WARNING: Trying to move Motor Z that does not exist")
                zang = None

        if num_motors >= 5:
            t_final_pos = goal_pos(T, 1)
            z_final_pos = goal_pos(Z, 1)

            if tang is not None:
                t_final_pos = convertangletopos(T, tang)                                            # calculate T goal position
                overshoot_T = convertangletopos(T, OVERSHOOT_ANG) - convertangletopos(T, 0)         # convert overshoot angle to motor position step
                overshoot_Z = 0                                                                     # no overshoot on Z

            if zang is not None:
                z_final_pos = convertangletopos(Z, zang)                                            # calculate Z goal position
                overshoot_T = 0                                                                     # no overshoot on T
                overshoot_Z = convertangletopos(Z, OVERSHOOT_ANG) - convertangletopos(Z, 0)         # convert overshoot angle to motor position step

            if phang is not None:
                ph_final_pos = convertangletopos(PH, phang)
                t_final_pos = ph_final_pos[0]
                z_final_pos = ph_final_pos[1]
                overshoot_T = convertangletopos(T, OVERSHOOT_ANG) - convertangletopos(T, 0)         # convert overshoot angle to motor position step
                overshoot_Z = convertangletopos(Z, OVERSHOOT_ANG) - convertangletopos(Z, 0)         # convert overshoot angle to motor position step

            ph_final_pos = [t_final_pos, z_final_pos]

            t_limit = GIM_MOTION[5]["anglelim"]
            if num_motors >= 6:
                z_limit = GIM_MOTION[6]["anglelim"]
            else:
                z_limit = [0, 0]
        else:
            t_final_pos = convertangletopos(T, 0)
            z_final_pos = convertangletopos(Z, 0)
            t_limit = [0, 0]
            z_limit = [0, 0]

        if not(th_limit[0] <= convertpostoangle(TH, th_final_pos) <= th_limit[1]):                  # check that TH angle is within TH angle limits
            ok = False
            print("TH motor angle target = %0.4f is out of range %s" % (thang, str(th_limit)))
        if not(t_limit[0] <= convertpostoangle(T, t_final_pos) <= t_limit[1]):                      # check that T angle is within T angle limits
            ok = False
            print("T motor angle target = %0.4f is out of range %s" % (convertpostoangle(T, t_final_pos), str(t_limit)))
        if not (z_limit[0] <= convertpostoangle(Z, z_final_pos) <= z_limit[1]):                     # check that Z angle is within Z angle limits
            ok = False
            print("Z motor angle target = %0.4f is out of range %s" % (convertpostoangle(Z, z_final_pos), str(z_limit)))

        if not checkonly:
            if not ok:
                print("*** Movement cancelled ***")                                                 # do not move if out of range
            else:
                if accuracy == "VERY HIGH":
                    th_cur_goal_pos = goal_pos(TH, 1)
                    if th_final_pos != th_cur_goal_pos:                                             # if need to move TH
                        ok = ok and move_pos(TH, th_final_pos - overshoot_TH)                       # for very high accuracy, overshoot TH goal position first
                    if num_motors >= 5:
                        t_cur_goal_pos = goal_pos(T, 1)
                        if ph_final_pos[0] != t_cur_goal_pos:                                       # if need to move T
                            ok = ok and move_pos(T, ph_final_pos[0] - overshoot_T)                  # for very high accuracy, overshoot T goal position first
                    if num_motors >= 6:
                        z_cur_goal_pos = goal_pos(Z, 1)
                        if ph_final_pos[1] != z_cur_goal_pos:                                       # if need to move Z
                            ok = ok and move_pos(Z, ph_final_pos[1] - overshoot_Z)                  # for very high accuracy, overshoot Z position first
                    wait_stop_moving(accuracy)                                                      # wait for all motors to stop moving

                ok = ok and move_pos(TH, th_final_pos)                                              # move to final TH position
                if num_motors >= 5:
                    ok = ok and move_pos(PH, ph_final_pos)                                          # move to final [PH, DPH] position
                wait_stop_moving(accuracy)                                                          # wait for all motors to stop moving

    return ok


def move_angle_rel(motor, angstep, accuracy="HIGH"):
    """ Move motor position to a new position relative to current one (and check for boundary limits) """
    ok = 1

    if gim_type == HV:
        step_pos = convertangletopos(motor, angstep) - convertangletopos(motor, 0)                  # convert step angle to motor position step
        cur_goal_pos = goal_pos(motor, 1)                                                           # read the goal position

        if abs(step_pos) < 1:                                                                       # check step size is not lower than resolution
            print("step is too small for this motor, increase step size, to move")
        else:
            newpos = cur_goal_pos + step_pos
            newangle = convertpostoangle(motor, newpos)                                             # calculate new angle
            if motor == H:
                move_angle(hang=newangle, accuracy=accuracy)                                        # move H angle
            elif motor == V:
                if num_motors >= 2:
                    move_angle(vang=newangle, accuracy=accuracy)                                    # move V angle
                else:
                    print("WARNING: Trying to move V motor, but motor not detected")
            elif motor == P:
                if num_motors >= 4:
                    move_angle(pang=newangle, accuracy=accuracy)                                    # move P angle
                else:
                    print("WARNING: Trying to move P motor, but motor not detected")

    elif gim_type == SPHERICAL:
        if motor == PH and num_motors < 6:                                                          # move PH for GIM05_FIXED is same as T motor only
            motor = T

        if motor == PH:
            step_pos_T = convertangletopos(T, angstep) - convertangletopos(T, 0)                    # convert step angle to T motor step position
            cur_goal_pos_T = goal_pos(T, 1)                                                         # read the T goal position

            step_pos_Z = convertangletopos(Z, angstep) - convertangletopos(Z, 0)                    # convert step angle to Z motor step position
            cur_goal_pos_Z = goal_pos(Z, 1)                                                         # read the Z goal position

            if num_motors >= 6:
                if abs(step_pos_T) < 1 or abs(step_pos_Z) < 1:
                    print("step is too small for this motor, increase step size, to move")
                else:
                    newpos_T = cur_goal_pos_T + step_pos_T
                    newpos_Z = cur_goal_pos_Z + step_pos_Z
                    newangle = convertpostoangle(PH, [newpos_T, newpos_Z])
                    move_angle(phang=newangle, accuracy=accuracy)
            else:
                print("WARNING: Trying to move PH motor, but motor not detected")

        else:
            if motor == TH:
                motor_phys = TH
                angstep_phys = angstep
            elif motor == DPH:
                motor_phys = Z
                angstep_phys = -angstep
            elif motor == T:
                motor_phys = T
                angstep_phys = angstep
            else:
                print("WARNING: Invalid SPHERICAL motor type")
                ok = 0
                return ok
            step_pos = convertangletopos(motor_phys, angstep_phys) - convertangletopos(motor_phys, 0)
            cur_goal_pos = goal_pos(motor_phys, 1)

            if abs(step_pos) < 1:                                                                   # check step size is not lower than resolution
                print("step is too small for this motor, increase step size, to move")
            else:
                newpos = cur_goal_pos + step_pos
                newangle = convertpostoangle(motor_phys, newpos)                                    # calculate new angle
                if motor_phys == TH:
                    move_angle(thang=newangle, accuracy=accuracy)                                   # move TH angle
                elif motor_phys == T:
                    if num_motors >= 5:
                        move_angle(tang=newangle, accuracy=accuracy)                                # move T angle
                    else:
                        print("WARNING: Trying to move T motor, but motor not detected")
                elif motor_phys == Z:
                    if num_motors >= 6:
                        move_angle(zang=newangle, accuracy=accuracy)                                # move Z angle
                    else:
                        print("WARNING: Trying to move Z motor, but motor not detected")

    return ok


def gim_move(h_target, v_target, p_target, accuracy="HIGH"):                    # move H and V motors to any H,V position in space
    """ make a direct move and measure the move time """
    t0 = time.time()                                                            # record start time
    if move_angle(hang=h_target, vang=v_target, pang=p_target, accuracy=accuracy):      # do the move
        t1 = time.time()                                                        # record stop time
        travel_time = (t1 - t0)                                                 # measure travel time
        print(" travel time was : %0.3f seconds" % travel_time)                 # print travel time
    else:
        print("*** ERROR: move location out of valid range")
    return


def gim_move_sph(theta_target, phi_target, accuracy="HIGH"):                    # move GIM05 motors to any theta/phi position in space
    """ make a direct move and measure the move time """
    t0 = time.time()                                                            # record start time
    if move_angle(thang=theta_target, phang=phi_target, accuracy=accuracy):
        t1 = time.time()                                                        # record stop time
        travel_time = (t1 - t0)                                                 # measure travel time
        print(" travel time was : %0.3f seconds" % travel_time)                 # print travel time
    else:
        print("*** ERROR: move location out of valid range")
    return


def gotoZERO(accuracy="HIGH"):
    """ makes all motors go home """
    print("going to zero position")
    if gim_type == HV:
        move_angle(hang=0, vang=0, pang=0, accuracy=accuracy)
    elif gim_type == SPHERICAL:
        move_angle(thang=0, phang=[0, 0], accuracy=accuracy)
    return


# ===============================================================
# ============= EQUIPMENT SELECTION AND MEASUREMENT =============
# ===============================================================

def select_meas_mode(cur_mode="UNDEFINED"):
    """ select if using SA, SG+SA, or VNA """
    print("\nCurrent measurement mode = %s\n" % cur_mode)
    print("************* MEASUREMENT SETUP *************")
    print("* press <0> for No Instrument")
    print("* press <1> for Spectrum Analyzer only")
    print("* press <2> for SigGen + Spectrum Analyzer")
    print("* press <3> for VNA (mag)")
    print("* press <4> for VNA (mag/phase)")
    print("* press <ESC> for no change")
    print("*********************************************")
    valid = False
    while not valid:
        pressedkey = ord(getch())
        if pressedkey == ord('0'):
            meas_mode = "NONE"
            print("NO EQUIPMENT mode selected")
            valid = True
        elif pressedkey == ord('1'):
            meas_mode = 'SA'
            print("Spectrum Analyzer mode selected")
            valid = True
        elif pressedkey == ord('2'):
            meas_mode = 'SG+SA'
            print("Sig Gen + Spectrum Analyzer mode selected")
            valid = True
        elif pressedkey == ord('3'):
            meas_mode = 'VNA'
            print("VNA (mag) mode selected")
            valid = True
        elif pressedkey == ord('4'):
            meas_mode = 'VNA_MAGPH'
            print("VNA (mag/phase) mode selected")
            valid = True
        elif pressedkey == 27:
            meas_mode = cur_mode
            if meas_mode != "UNDEFINED":
                valid = True
            else:
                print("Must select a valid measurement mode")
    return meas_mode


def select_visa_addr(orig_addr="SIMULATION"):
    """ displays list of connected VISA instruments and selects one """
    print("")
    print("************** INSTRUMENT LIST **************")
    resources = equip.list_resources()                                          # find list of potential instruments
    resources = [x for x in resources if str(x).find('ASRL') == -1]             # only keep resources without "ASRL" in name
    resources.insert(0, 'MANUAL ENTRY (%s)' % orig_addr)                        # pre-pend "MANUAL ENTRY" to the list - used to type in a socket address
    resources.insert(0, 'SIMULATION')                                           # pre-pend "SIMULATION" to the list - used if no instrument connected

    for x in range(0, len(resources), 1):                                       # list all the resources
        if orig_addr == resources[x]:
            print("  >>> %3d) %s" % (x+1, resources[x]))                        # show which equipment is currently selected
        else:
            print("      %3d) %s" % (x+1, resources[x]))
    print("*********************************************")
    print("")

    done = False
    while not done:
        selection = int(input_num("Select instrument or enter <0> for no change: "))
        if selection in range(len(resources)+1):
            if selection == 0:
                new_addr = orig_addr
                done = True
            elif selection == 2:                                                # manual entry
                new_addr = str(six.moves.input("Enter equipment VISA address: "))
                done = True
            else:
                new_addr = str(resources[selection-1])                          # set the name of the GPIB resource, convert from unicode to string
                done = True
        else:
            print("Invalid selection. Please try again")

    print ("")
    print("Measurement instrument selected (addr): %s" % (new_addr))

    return new_addr


def visa(orig_meas_mode, inst):
    """ list all potential VISA instruments connected, select and initialize for measurement """

    orig_addr = inst.addr
    inst.close_instrument()

    meas_mode = select_meas_mode(orig_meas_mode)

    # Spectrum Analyzer mode
    if meas_mode == "SA":
        if orig_meas_mode != meas_mode:
            orig_addr = ["SIMULATION"]                                          # if previous was not SA, set default to SIMULATION

        print("Select Spectrum Analyzer VISA address")
        new_addr = [select_visa_addr(orig_addr[0])]                             # select Spectrum Analyzer

        inst = equip.inst_setup(meas_mode, new_addr)                            # initialize equipment

    # SigGen + SpecAnalyzer mode
    elif meas_mode == "SG+SA":
        if orig_meas_mode != meas_mode:
            orig_addr = ["SIMULATION", "SIMULATION"]                            # if previous was not SG+SA, set default to SIMULATION

        print("\n\nSelect SIG GEN VISA address")
        sg_addr = select_visa_addr(orig_addr[0])                                # select Sig Gen
        print("\n\nSelect SPECTRUM ANALYZER VISA address")
        sa_addr = select_visa_addr(orig_addr[1])                                # select Spectrum Analyzer
        new_addr = [sg_addr, sa_addr]

        inst = equip.inst_setup(meas_mode, new_addr)                            # initialize equipment

    # VNA mode
    elif meas_mode == "VNA" or meas_mode == "VNA_MAGPH":
        if orig_meas_mode != meas_mode:
            orig_addr = ["SIMULATION"]                                          # if previous was not VNA, set default to SIMULATION

        print("Select VNA VISA address")
        new_addr = [select_visa_addr(orig_addr[0])]                             # select VNA

        inst = equip.inst_setup(meas_mode, new_addr)                            # initialize equipment

    # NONE or undefined mode
    else:
        inst = equip.inst_setup(meas_mode, ["SIMULATION"])                      # initialize SIMULATION mode

    if inst.port_open:                                                          # if instrument is open
        print("initializing equipment")
        inst.init_meas()                                                        # initialize instrument for measurement

    return meas_mode, inst


def get_power(inst, raw_data=False):
    """ return measured power at a given gimbal (H,V) position or compute value if no instrument connected """

    val = []
    freq = []

    # readback power from instrument
    if inst.port_open:
        if inst.inst_type.find("SA") > -1:
            try:
                inst.single_trigger()                                           # single trigger and hold if it has been implemented
            except:
                pass
            val = [round(float(inst.get_marker(1)),2)]                          # readback marker 1 if connected to an instrument
            freq = [float(inst.get_marker_freq(1))]                             # readback marker 1 freq

            if not raw_data:
                val = proc.corr_power(freq, val)                                # apply correction factor to measured power

        elif inst.inst_type.find("VNA") > -1:
            try:
                inst.single_trigger()                                           # single trigger and hold if it has been implemented
            except:
                pass
            db, phase = inst.get_s_dbphase()                                    # readback S21
            freq_list = inst.get_freq_list()                                    # readback frequency list

            if not raw_data:
                db_corr = proc.corr_power(freq_list, db)                        # apply correction factor to measured power
            else:
                db_corr = db

            val = []                                                            # init val list
            freq = []                                                           # init freq list
            for i in range(len(db_corr)):
                val.append(db_corr[i])                                          # add dB (corrected) value
                freq.append(freq_list[i])                                       # add freq
                if inst.inst_type.find("MAGPH") > -1:
                    val.append(phase[i])                                        # add phase value
                    freq.append(freq_list[i])                                   # add freq

    # compute simulated power level based on (H,V) position
    else:                                                                       # compute DUMMY value based on gimbal position if not connected to instrument
        # print("type = %s" % inst.inst_type)

        offset_db = 0.0
        if gim_type == HV:
            hori_ang = convertpostoangle(H,current_pos(H,1))
            vert_ang = convertpostoangle(V,current_pos(V,1))
            offset_db = ((vert_ang ** 2 + 0.6 * hori_ang ** 2) * (-1 / 300.0))  # DUMMY val, when instrument is not connected
        elif gim_type == SPHERICAL:
            theta_ang = convertpostoangle(TH,current_pos(TH,1))
            phi_ang, dphi_ang = convertpostoangle(PH,current_pos(PH,1))
            offset_db = ((theta_ang ** 2 * (2.0 - np.sin(phi_ang*np.pi/180)**2)) * (-1 / 300.0))    # DUMMY val, when instrument is not connected

        if inst.inst_type.find("VNA") > -1:
            db, phase = inst.get_s_dbphase()                                    # readback S21
            freq_list = inst.get_freq_list()                                    # readback frequency list

            db = np.array(db) + offset_db                                       # add simulated value to db array

            if not raw_data:
                db_corr = proc.corr_power(freq_list, db)                        # apply correction factor to measured power
            else:
                db_corr = db

            val = []                                                            # init val list
            freq = []                                                           # init freq list
            for i in range(len(db_corr)):
                val.append(db_corr[i])                                          # add dB (corrected) value
                freq.append(freq_list[i])                                       # add freq
                if inst.inst_type.find("MAGPH") > -1:
                    val.append(phase[i])                                        # add phase value
                    freq.append(freq_list[i])                                   # add freq

        else:
            val = [offset_db]                                                   # default DUMMY SA data
            freq = [28.0e9]

            if not raw_data:
                val = proc.corr_power(freq, val)                                # apply correction factor to measured power

    val = [round(x,2) for x in val]                                             # round to 2 digits
    return val, freq


def meas_to_file(inst):
    """ save measurement of freq/power to CSV file """
    raw_data = False
    if proc.is_corr_on():                                                       # determine whether to save Raw or Corrected data
        print("Save <C>orrected data or <R>aw data?")
        valid = False
        while not valid:
            pressedkey = chr(ord(getch().upper()))
            if pressedkey == "C":
                raw_data = False
                valid = True
            elif pressedkey == "R":
                raw_data = True
                valid = True
        print("")

    num = int(np.round(input_num("Number of consecutive measurements: ")))
    if num > 1:
        delay = float(input_num("Delay between measurements [sec]: "))
        # save_stats = False
        # print("Save statistics to file? [Y/N]")
        # key = None                                                              # ask if user wants to save stats (mean, stdev, etc.)
        # while key not in ['Y', 'N']:
        #     key = chr(ord(getch().upper()))
        #     if key == 'Y':
        #         save_stats = True
    else:
        delay = 0

    data = []
    for k in range(num):
        print("Measuring point %d of %d" % (k+1, num))

        freqList = np.array([28.0e9])                                               # default to 28GHz if simulation mode or no instrument mode
        if inst.inst_type.find("SA") > -1:
            if inst.port_open:                                                      # if connected to an instrument, get freq
                min_f = float(input_num("Enter your start freq [GHz]: "))           # frequency sweep
                max_f = float(input_num("Enter your end freq [GHz]: "))
                step_f = float(input_num("Enter your step freq [GHz]: "))
                if max_f > min_f:
                    num = np.floor((max_f - min_f) / step_f)                        # freq sweep if max_f > min_f
                else:
                    num = 0                                                         # fixed freq if max_f <= min_f
                freqList = np.linspace(min_f, min_f + num * step_f, num + 1)        # [min:step:max] with endpoints inclusive
                print("")
            val = []
            freq = []
            for f in freqList:
                inst.set_freq(f)                                                    # for SG+SA mode, set SG output freq and SA meas freq
                val0, freq0 = get_power(inst, raw_data=raw_data)
                val.append(val0)                                                    # append measured power
                freq.append(freq0)                                                  # append measured freq

        elif inst.inst_type.find("VNA") > -1:
            val, freq = get_power(inst, raw_data=raw_data)

        else:
            val, freq = get_power(inst, raw_data=raw_data)

        # print("k = " + str(k))
        # print("freq = " + str(freq))
        # print("val = " + str(val))
        # print("data = " + str(data))

        # data = np.transpose(np.array([freq, val]))
        if len(data) == 0:
            # print("num = " + str(num))
            # print("freq = " + str(freq))
            data = np.array([[np.nan for x in range(num+1)] for y in range(len(freq))])

            # print("data = " + str(data))
            data[:, 0] = np.array(freq)
            # print("data = " + str(data))
            data[:, k+1] = np.array(val)
            # print("data = " + str(data))
        else:
            # print("data = " + str(data))
            data[:, k+1] = np.array(val)
            # print("data = " + str(data))

        time.sleep(delay)

    try:
        inst.cont_trigger()                                                     # set to cont sweep after power measurement
    except:
        pass

    print("")
    fname = six.moves.input("Filename to save or [ENTER] to cancel: ")
    print("")

    if fname != "":
        (mypath, myfile) = os.path.split(fname)
        (myname, myext) = os.path.splitext(myfile)
        if myext != ".csv":
            myfile = myfile + ".csv"                                            # for any filename without .csv extension, append .csv
        outdir = os.path.join('..', '..', 'MilliBox_plot_data', mypath)         # outdir starts at ..\..\MilliBox_plot_data
        if not os.path.isdir(outdir):                                           # check if directory exists
            print("*** Creating output directory %s ***" % outdir)
            try:
                os.mkdir(outdir)                                                # create directory if it doesn't exist
            except:
                print("")
                print("*** WARNING: Unable to create directory %s" % outdir)    # could not create directory
                print("*** Save cancelled ***")
                return
        fname = os.path.join(outdir, myfile)
        if os.path.isfile(fname):
            print("Overwrite file %s? [Y/N]" % fname)
            keypressed = None
            while keypressed not in ['Y', 'N']:
                keypressed = chr(ord(getch().upper()))
            if keypressed == 'N':
                print("*** Save cancelled ***")
                return

        csvfile = open(fname, "w", buffering=1)
        writer = csv.writer(csvfile, lineterminator="\n")
        writer.writerows(data)                                                  # save the CSV file
        csvfile.close()
        print("SAVED to %s" % fname)

    return


# =================================================
# ============= SWEEP CHECK FUNCTIONS =============
# =================================================

def input_num(prompt, default=None):
    """ prompt for a number and wait until a valid number is returned """
    ok = False
    while not ok:
        s = six.moves.input(prompt)                                             # display prompt and wait for input, does not crash on empty string
        if len(s) == 0:                                                         # six make it 2.x and 3.x compatible
            s = str(default)
        try:
            x = float(s)
            ok = True
        except ValueError:
            print("\n*** ERROR: Please enter a valid number ***\n")
    return x


def check_plot_1d(dir, minh, maxh, minv, maxv, step, pola=None):
    """ check that user plot values are valid for 1D sweep """
    global GIM_MOTION
    ok = 1
    if gim_type == HV:
        h_lim = GIM_MOTION[1]["anglelim"]
        if num_motors >= 2:
            v_lim = GIM_MOTION[2]["anglelim"]
        else:
            v_lim = [0, 0]
        if num_motors >= 4:
            p_lim = GIM_MOTION[4]["anglelim"]
        else:
            p_lim = [0, 0]

        if (minh < h_lim[0]) or (minv < v_lim[0]):
            ok = 0
        if (maxh > h_lim[1]) or (maxv > v_lim[1]):
            ok = 0
        if (maxh-minh) < 0:
            ok = 0
        if (maxv-minv) < 0:
            ok = 0
        if dir == "H" and step > (maxh-minh):
            ok = 0
        if dir == "V" and step > (maxv-minv):
            ok = 0
        if step <= 0:
            ok = 0
        if pola is not None:
            if (pola > p_lim[1]) or (pola < p_lim[0]):
                ok = 0
    else:
        ok = 0
    return ok


def check_plot_1d_sph(dir, minth, maxth, minph, maxph, step, dphi=None):
    """ check that user plot values are valid for 1D sweep """
    global GIM_MOTION
    ok = 1
    if gim_type == SPHERICAL:
        h_lim = GIM_MOTION[1]["anglelim"]
        if num_motors >= 5:
            t_lim = GIM_MOTION[5]["anglelim"]
        else:
            t_lim = [0, 0]
        if num_motors >= 6:
            z_lim = GIM_MOTION[6]["anglelim"]
        else:
            z_lim = [0, 0]

        if (minth < h_lim[0]) or (minph < t_lim[0]):
            ok = 0
        if (maxth > h_lim[1]) or (maxph > t_lim[1]):
            ok = 0
        if (maxth-minth) < 0:
            ok = 0
        if (maxph-minph) < 0:
            ok = 0
        if dir == "T" and step > (maxth-minth):
            ok = 0
        if dir == "P" and step > (maxph-minph):
            ok = 0
        if step <= 0:
            ok = 0
        if dphi is not None:
            if (maxph - dphi > z_lim[1]) or (minph - dphi < z_lim[0]):
                ok = 0
    else:
        ok = 0
    return ok


def check_plot(minh, maxh, minv, maxv, step, pola=None):
    """ check that user plot values are valid """
    global GIM_MOTION
    ok = 1
    if gim_type == HV:
        h_lim = GIM_MOTION[1]["anglelim"]
        if num_motors >= 2:
            v_lim = GIM_MOTION[2]["anglelim"]
        else:
            v_lim = [0, 0]
        if num_motors >= 4:
            p_lim = GIM_MOTION[4]["anglelim"]
        else:
            p_lim = [0, 0]

        if (minh < h_lim[0]) or (minv < v_lim[0]):
            ok = 0
        if (maxh > h_lim[1]) or (maxv > v_lim[1]):
            ok = 0
        if (maxh-minh) < 0:
            ok = 0
        if (maxv-minv) < 0:
            ok = 0
        if step > (maxh-minh):
            ok = 0
        if step > (maxv-minv):
            ok = 0
        if step <= 0:
            ok = 0
        if pola is not None:
            if (pola > p_lim[1]) or (pola < p_lim[0]):
                ok = 0
    else:
        ok = 0
    return ok


def check_plot_sph(minth, maxth, minph, maxph, step, dphi=None):
    """ check that user plot values are valid """
    global GIM_MOTION
    ok = 1
    if gim_type == SPHERICAL:
        h_lim = GIM_MOTION[1]["anglelim"]
        if num_motors >= 5:
            t_lim = GIM_MOTION[5]["anglelim"]
        else:
            t_lim = [0, 0]
        if num_motors >= 6:
            z_lim = GIM_MOTION[6]["anglelim"]
        else:
            z_lim = [0, 0]

        if (minth < h_lim[0]) or (minph < t_lim[0]):
            ok = 0
        if (maxth > h_lim[1]) or (maxph > t_lim[1]):
            ok = 0
        if (maxth-minth) < 0:
            ok = 0
        if (maxph-minph) < 0:
            ok = 0
        if step > (maxth-minth):
            ok = 0
        if step > (maxph-minph):
            ok = 0
        if step <= 0:
            ok = 0
        if dphi is not None:
            if (maxph-dphi > z_lim[1]) or (minph-dphi < z_lim[0]):
                ok = 0
    else:
        ok = 0
    return ok


def check_abort():
    """ check if <ESC> was pressed to abort measurement sweep """
    keypressed = chr(ord(getch()))
    if keypressed == chr(27):
        print("")
        print("Are you sure you want to ABORT? [Y/N]")
        while keypressed not in ['Y', 'N']:
            keypressed = chr(ord(getch().upper()))
        if keypressed == 'Y':
            print("*** ABORTING ***")
            print("")
    else:
        keypressed = ''
    return keypressed == 'Y'


# =================================================
# ================ SWEEP FUNCTIONS ================
# =================================================

def millibox_1dsweep(dir, minh, maxh, minv, maxv, step, pangle, plot, tag, inst, accuracy="HIGH", meas_delay=0, plot_freq=0, validonly=True):
    """ 1D sweep - capture, plot and save the data """

    t0 = time.time()                                                            # get the start time for routine
    timeStr = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())                # get day and time to build unique file names
    outdir = os.path.join('..', '..', 'MilliBox_plot_data')                     # outdir is ..\..\MilliBox_plot_data
    if not os.path.isdir(outdir):                                               # check if directory exists
        print("*** Creating output directory MilliBox_plot_data ***")
        os.mkdir(outdir)                                                        # create directory if it doesn't exist
    basename = os.path.join(outdir, 'mbx_capture_'+timeStr+'_1d_'+dir+'_'+tag)  # format base filename
    filename = ("%s.csv" % basename)                                            # format CSV filename
    print(" Plot data is saved in file : %s" % filename)                        # tell user filename
    csvplot = open(filename, 'w', buffering=1)                                  # open CSV file for write
    capture = csv.writer(csvplot, lineterminator='\n')                          # set line terminator to newline only (no carraige return)

    if proc.is_corr_on() and proc.get_corr_write():                             # check if corr factor ON and write ON
        corrname = ("%s_corr.csv" % basename)                                   # append _corr to filename
        print(" Corr factor data is saved in file : %s" % corrname)             # tell user filename
        proc.save_corr_file(corrname)                                           # save correction file

    val, freq = get_power(inst)                                                 # query the frequency points

    if num_motors >= 4:
        capture.writerow(['V', 'actual_V', 'H', 'actual_H', 'P', 'actual_P'] + freq)    # write the column headers to file (include pol)
    else:
        capture.writerow(['V', 'actual_V', 'H', 'actual_H'] + freq)             # write the column headers to file

    freqIdx = np.abs(np.array(freq) - plot_freq).argmin()                       # find index for value that is closest to plot_freq
    print("\n**** Plotting frequency = %0.3fGHz ****\n" % (freq[freqIdx]/1.0e9))

    num = int(np.floor((maxv-minv)/step))                                       # map of vertical angle iteration
    Vangle = np.linspace(minv,minv+num*step,num+1)                              # [min:step:max] with endpoints inclusive
    num = int(np.floor((maxh-minh)/step))                                       # map of horizontal angle iteration
    Hangle = np.linspace(minh,minh+num*step,num+1)                              # [min:step:max] with endpoints inclusive

    if validonly:                                                               # filter points within angle limits
        gim_motion = get_gim_motion()
        Hlim = gim_motion[1]["anglelim"]
        Hangle = Hangle[np.intersect1d(np.nonzero(Hangle >= Hlim[0]), np.nonzero(Hangle <= Hlim[1]))]
        if num_motors >= 2:
            Vlim = gim_motion[2]["anglelim"]
            Vangle = Vangle[np.intersect1d(np.nonzero(Vangle >= Vlim[0]), np.nonzero(Vangle <= Vlim[1]))]

    print(" V range is = " + str(Vangle))                                       # log tracker
    print(" H range is = " + str(Hangle))                                       # log tracker
    if num_motors >= 4:
        print(" Polarization position is = " + str(pangle))
        move_angle(pang=pangle, accuracy=accuracy)

    inst.fix_status()                                                           # check and run calibration, if needed

    if dir == "H":
        heatmap = [np.nan for x in Hangle]                                      # Initialize heatmap array with all point = NaN
        i = 0
        total = len(Hangle)                                                     # total number of measurement points
        vert = min(Vangle)                                                      # vert angle is fixed during sweep
        move_angle(vang=vert, accuracy=accuracy)                                # jump to vert angle
        for hori in Hangle:                                                     # loop for horizontal motion

            if kbhit():                                                         # check if key pressed
                if check_abort():                                               # check if <ESC> pressed
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)                                          # go to home and abort
                    csvplot.close()
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return

            move_angle(hang=hori, accuracy=accuracy)                            # move to H position

            time.sleep(meas_delay)                                              # optional delay after movement before measuring
            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            actual_H = convertpostoangle(H, current_pos(H, 1))                  # record actual absolute position moto H reached
            actual_V = convertpostoangle(V, current_pos(V, 1))                  # record actual absolute position moto V reached

            if num_motors >= 4:
                actual_P = convertpostoangle(P, current_pos(P, 1))
                if len(val) == 1:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))
                else:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))

                entry = [vert, actual_V, hori, actual_H, pangle, actual_P] + val                     # record a new plot entry

            else:
                if len(val) == 1:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,val[freqIdx]))
                else:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,val[freqIdx]))

                entry = [vert, actual_V,  hori, actual_H] + val                 # record a new plot entry

            capture.writerow(entry)                                             # commit to CSV file
            heatmap[i] = val[freqIdx]                                           # append heatmap with actual captured val
            i += 1                                                              # update counter

            n = i                                                               # compute iterations completed
            t1 = time.time()                                                    # get the current time
            elapsed = t1 - t0                                                   # compute elapsed time
            total_time = elapsed / n * total                                    # estimate total time
            remain = total_time - elapsed                                       # compute remaining time
            print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
                  (100.0 * n / total,
                   datetime.timedelta(seconds=int(total_time)),
                   datetime.timedelta(seconds=int(elapsed)),
                   datetime.timedelta(seconds=int(remain))))                    # print % complete and time remaining

            if hori == Hangle[-1]:                                              # last point
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to (0,0)
                csvplot.close()                                                 # close the CSV file
                print("## THE PLOT WAS SAVED IN FILE :  " + str(filename) + "    ##")  # tell user where to find CSV file
                t1 = time.time()
                print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))   # display elapsed time
            if plot == 1:
                display_1dplot(dir,Vangle,Hangle,heatmap,vert,hori,plot_freq=freq[freqIdx],pangle=pangle)   # update the line plot after each data point

    elif dir == "V":
        heatmap = [np.nan for x in Vangle]                                      # Initialize heatmap array with all point = NaN
        i = 0
        total = len(Vangle)                                                     # total number of measurement points
        hori = min(Hangle)                                                      # hori angle is fixed during sweep
        move_angle(hang=hori, accuracy=accuracy)                                # jump to hori angle
        for vert in Vangle:                                                     # loop for vertical motion

            if kbhit():                                                         # check if key pressed
                if check_abort():                                               # check if <ESC> pressed
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)                                          # go to home and abort
                    csvplot.close()
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return

            move_angle(vang=vert, accuracy=accuracy)                            # move to V position

            time.sleep(meas_delay)                                              # optional delay after movement before measuring
            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            actual_H = convertpostoangle(H, current_pos(H, 1))                  # record actual absolute position moto H reached
            actual_V = convertpostoangle(V, current_pos(V, 1))                  # record actual absolute position moto V reached

            if num_motors >= 4:
                actual_P = convertpostoangle(P, current_pos(P, 1))
                if len(val) == 1:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))
                else:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))

                entry = [vert, actual_V, hori, actual_H, pangle, actual_P] + val                     # record a new plot entry

            else:
                if len(val) == 1:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,val[freqIdx]))
                else:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,val[freqIdx]))

                entry = [vert, actual_V,  hori, actual_H] + val                     # record a new plot entry

            capture.writerow(entry)                                             # commit to CSV file
            heatmap[i] = val[freqIdx]                                           # append heatmap with actual captured val
            i += 1                                                              # update counter

            n = i                                                               # compute iterations completed
            t1 = time.time()                                                    # get the current time
            elapsed = t1 - t0                                                   # compute elapsed time
            total_time = elapsed / n * total                                    # estimate total time
            remain = total_time - elapsed                                       # compute remaining time
            print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
                  (100.0 * n / total,
                   datetime.timedelta(seconds=int(total_time)),
                   datetime.timedelta(seconds=int(elapsed)),
                   datetime.timedelta(seconds=int(remain))))                    # print % complete and time remaining

            if vert == Vangle[-1]:                                              # last point
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to (0,0)
                csvplot.close()                                                 # close the CSV file
                print("## THE PLOT WAS SAVED IN FILE :  " + str(filename) + "    ##")  # tell user where to find CSV file
                t1 = time.time()
                print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))   # display elapsed time

            if plot == 1:
                display_1dplot(dir,Vangle,Hangle,heatmap,vert,hori,plot_freq=freq[freqIdx],pangle=pangle)     # update the line plot after each data point

    return


def millibox_1dsweep_sph(dir, minth, maxth, minph, maxph, step, dphi, plot, tag, inst, accuracy="HIGH", meas_delay=0, plot_freq=0, validonly=True):
    """ 1D sweep (spherical coords) - capture, plot and save the data """

    if num_motors < 6:                                                          # for GIM05_FIXED target DPHI=nan (undefined)
        dphi = float("nan")

    t0 = time.time()                                                            # get the start time for routine
    timeStr = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())                # get day and time to build unique file names
    outdir = os.path.join('..', '..', 'MilliBox_plot_data')                     # outdir is ..\..\MilliBox_plot_data
    if not os.path.isdir(outdir):                                               # check if directory exists
        print("*** Creating output directory MilliBox_plot_data ***")
        os.mkdir(outdir)                                                        # create directory if it doesn't exist
    basename = os.path.join(outdir, 'mbx_capture_'+timeStr+'_1d_sph_'+dir+'_'+tag)  # format base filename
    filename = ("%s.csv" % basename)                                            # format CSV filename
    print(" Plot data is saved in file : %s" % filename)                        # tell user filename
    csvplot = open(filename, 'w', buffering=1)                                  # open CSV file for write
    capture = csv.writer(csvplot, lineterminator='\n')                          # set line terminator to newline only (no carraige return)

    if proc.is_corr_on() and proc.get_corr_write():                             # check if corr factor ON and write ON
        corrname = ("%s_corr.csv" % basename)                                   # append _corr to filename
        print(" Corr factor data is saved in file : %s" % corrname)             # tell user filename
        proc.save_corr_file(corrname)                                           # save correction file

    val, freq = get_power(inst)                                                 # query the frequency points

    capture.writerow(['PHI', 'actual_PHI', 'THETA', 'actual_THETA', 'DPHI', 'actual_DPHI'] + freq)  # write the column headers to file (include pol)

    freqIdx = np.abs(np.array(freq) - plot_freq).argmin()                       # find index for value that is closest to plot_freq
    print("\n**** Plotting frequency = %0.3fGHz ****\n" % (freq[freqIdx]/1.0e9))

    num = int(np.floor((maxph-minph)/step))                                     # map of PHI angle iteration
    PHangle = np.linspace(minph,minph+num*step,num+1)                           # [min:step:max] with endpoints inclusive
    num = int(np.floor((maxth-minth)/step))                                     # map of THETA angle iteration
    THangle = np.linspace(minth,minth+num*step,num+1)                           # [min:step:max] with endpoints inclusive

    if validonly:                                                               # filter points within angle limits
        gim_motion = get_gim_motion()
        THlim = gim_motion[1]["anglelim"]
        THangle = THangle[np.intersect1d(np.nonzero(THangle >= THlim[0]), np.nonzero(THangle <= THlim[1]))]
        if num_motors >= 6:
            Tlim = gim_motion[5]["anglelim"]
            Zlim = gim_motion[6]["anglelim"]
            ph_idx = np.intersect1d(np.nonzero(PHangle >= Tlim[0]), np.nonzero(PHangle <= Tlim[1]))
            dph_idx = np.intersect1d(np.nonzero(PHangle - dphi >= Zlim[0]), np.nonzero(PHangle - dphi <= Zlim[1]))
            PHangle = PHangle[np.intersect1d(ph_idx, dph_idx)]
        elif num_motors >= 5:
            Tlim = gim_motion[5]["anglelim"]
            ph_idx = np.intersect1d(np.nonzero(PHangle >= Tlim[0]), np.nonzero(PHangle <= Tlim[1]))
            PHangle = PHangle[ph_idx]

    print(" PHI range is = " + str(PHangle))                                    # log tracker
    print(" THETA range is = " + str(THangle))                                  # log tracker
    if num_motors >= 6:
        print(" DPHI position is = " + str(dphi))

    inst.fix_status()                                                           # check and run calibration, if needed

    if dir == "T":
        heatmap = [np.nan for x in THangle]                                     # Initialize heatmap array with all point = NaN
        i = 0
        total = len(THangle)                                                    # total number of measurement points
        phi = min(PHangle)                                                      # PHI angle is fixed during sweep
        move_angle(phang=[phi, dphi], accuracy=accuracy)                        # jump to PHI angle
        for theta in THangle:                                                   # loop for THETA motion

            if kbhit():                                                         # check if key pressed
                if check_abort():                                               # check if <ESC> pressed
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)                                          # go to home and abort
                    csvplot.close()
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return

            move_angle(thang=theta, accuracy=accuracy)                          # move to THETA position

            time.sleep(meas_delay)                                              # optional delay after movement before measuring
            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            actual_TH = convertpostoangle(TH, current_pos(TH, 1))               # record actual absolute position motor TH reached
            actual_PH, actual_DPH = convertpostoangle(PH, current_pos(PH, 1))   # record actual absolute position motor PH reached

            if len(val) == 1:
                print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=%0.2f" % (phi,actual_PH,theta,actual_TH,dphi,actual_DPH,val[freqIdx]))
            else:
                print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=[ ... %0.2f ... ]" % (phi,actual_PH,theta,actual_TH,dphi,actual_DPH,val[freqIdx]))

            entry = [phi, actual_PH, theta, actual_TH, dphi, actual_DPH] + val  # record a new plot entry

            capture.writerow(entry)                                             # commit to CSV file
            heatmap[i] = val[freqIdx]                                           # append heatmap with actual captured val
            i += 1                                                              # update counter

            n = i                                                               # compute iterations completed
            t1 = time.time()                                                    # get the current time
            elapsed = t1 - t0                                                   # compute elapsed time
            total_time = elapsed / n * total                                    # estimate total time
            remain = total_time - elapsed                                       # compute remaining time
            print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
                  (100.0 * n / total,
                   datetime.timedelta(seconds=int(total_time)),
                   datetime.timedelta(seconds=int(elapsed)),
                   datetime.timedelta(seconds=int(remain))))                    # print % complete and time remaining

            if theta == THangle[-1]:                                            # last point
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to (0,0)
                csvplot.close()                                                 # close the CSV file
                print("## THE PLOT WAS SAVED IN FILE :  " + str(filename) + "    ##")  # tell user where to find CSV file
                t1 = time.time()
                print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))   # display elapsed time
            if plot == 1:
                display_1dplot_sph(dir, PHangle, THangle, heatmap, phi, theta, plot_freq=freq[freqIdx], dphi=dphi)  # update the line plot after each data point

    elif dir == "P":
        heatmap = [np.nan for x in PHangle]                                     # Initialize heatmap array with all point = NaN
        i = 0
        total = len(PHangle)                                                    # total number of measurement points
        theta = min(THangle)                                                    # THETA angle is fixed during sweep
        move_angle(thang=theta, accuracy=accuracy)                              # jump to THETA angle
        for phi in PHangle:                                                     # loop for PHI motion

            if kbhit():                                                         # check if key pressed
                if check_abort():                                               # check if <ESC> pressed
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)                                          # go to home and abort
                    csvplot.close()
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return

            move_angle(phang=[phi, dphi], accuracy=accuracy)                    # move to PH position

            time.sleep(meas_delay)                                              # optional delay after movement before measuring
            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            actual_TH = convertpostoangle(TH, current_pos(TH, 1))               # record actual absolute position motor TH reached
            actual_PH, actual_DPH = convertpostoangle(PH, current_pos(PH, 1))   # record actual absolute position motor PH reached

            if len(val) == 1:
                print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=%0.2f" % (phi,actual_PH,theta,actual_TH,dphi,actual_DPH,val[freqIdx]))
            else:
                print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=[ ... %0.2f ... ]" % (phi,actual_PH,theta,actual_TH,dphi,actual_DPH,val[freqIdx]))

            entry = [phi, actual_PH, theta, actual_TH, dphi, actual_DPH] + val          # record a new plot entry

            capture.writerow(entry)                                             # commit to CSV file
            heatmap[i] = val[freqIdx]                                           # append heatmap with actual captured val
            i += 1                                                              # update counter

            n = i                                                               # compute iterations completed
            t1 = time.time()                                                    # get the current time
            elapsed = t1 - t0                                                   # compute elapsed time
            total_time = elapsed / n * total                                    # estimate total time
            remain = total_time - elapsed                                       # compute remaining time
            print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
                  (100.0 * n / total,
                   datetime.timedelta(seconds=int(total_time)),
                   datetime.timedelta(seconds=int(elapsed)),
                   datetime.timedelta(seconds=int(remain))))                    # print % complete and time remaining

            if phi == PHangle[-1]:                                              # last point
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to (0,0)
                csvplot.close()                                                 # close the CSV file
                print("## THE PLOT WAS SAVED IN FILE :  " + str(filename) + "    ##")  # tell user where to find CSV file
                t1 = time.time()
                print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))   # display elapsed time
            if plot == 1:
                display_1dplot_sph(dir, PHangle, THangle, heatmap, phi, theta, plot_freq=freq[freqIdx], dphi=dphi)  # update the line plot after each data point

    return


def millibox_hvsweep(min, max, step, pangle, plot, tag, inst, accuracy="HIGH", meas_delay=0, plot_freq=0, validonly=True):
    """ 1D sweep in H and V planes - capture, plot and save the data """

    t0 = time.time()                                                            # get the start time for routine
    timeStr = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())                # get day and time to build unique file names
    outdir = os.path.join('..', '..', 'MilliBox_plot_data')                     # outdir is ..\..\MilliBox_plot_data
    if not os.path.isdir(outdir):                                               # check if directory exists
        print("*** Creating output directory MilliBox_plot_data ***")
        os.mkdir(outdir)                                                        # create directory if it doesn't exist
    basename = os.path.join(outdir, 'mbx_capture_'+timeStr+'_hv_'+tag)          # format base filename
    filename = ("%s.csv" % basename)                                            # format CSV filename
    print(" Plot data is saved in file : %s" % filename)                        # tell user filename
    csvplot = open(filename, 'w', buffering=1)                                  # open CSV file for write
    capture = csv.writer(csvplot, lineterminator='\n')                          # set line terminator to newline only (no carraige return)

    if proc.is_corr_on() and proc.get_corr_write():                             # check if corr factor ON and write ON
        corrname = ("%s_corr.csv" % basename)                                   # append _corr to filename
        print(" Corr factor data is saved in file : %s" % corrname)             # tell user filename
        proc.save_corr_file(corrname)                                           # save correction file

    val, freq = get_power(inst)                                                 # query the frequency points

    if num_motors >= 4:
        capture.writerow(['V', 'actual_V', 'H', 'actual_H', 'P', 'actual_P'] + freq)    # write the column headers to file (include pol)
    else:
        capture.writerow(['V', 'actual_V', 'H', 'actual_H'] + freq)             # write the column headers to file

    freqIdx = np.abs(np.array(freq) - plot_freq).argmin()                       # find index for value that is closest to plot_freq
    print("\n**** Plotting frequency = %0.3fGHz ****\n" % (freq[freqIdx]/1.0e9))

    num = int(np.floor((max-min)/step))                                         # map of vertical angle iteration
    Vangle = np.linspace(min,min+num*step,num+1)                                # [min:step:max] with endpoints inclusive
    num = int(np.floor((max-min)/step))                                         # map of horizontal angle iteration
    Hangle = np.linspace(min,min+num*step,num+1)                                # [min:step:max] with endpoints inclusive

    if validonly:                                                               # filter points within angle limits
        gim_motion = get_gim_motion()
        Hlim = gim_motion[1]["anglelim"]
        Hangle = Hangle[np.intersect1d(np.nonzero(Hangle >= Hlim[0]), np.nonzero(Hangle <= Hlim[1]))]
        if num_motors >= 2:
            Vlim = gim_motion[2]["anglelim"]
            Vangle = Vangle[np.intersect1d(np.nonzero(Vangle >= Vlim[0]), np.nonzero(Vangle <= Vlim[1]))]

    print(" V range is = " + str(Vangle))                                       # log tracker
    print(" H range is = " + str(Hangle))                                       # log tracker
    if num_motors >= 4:
        print(" Polarization position is = " + str(pangle))
        move_angle(pang=pangle, accuracy=accuracy)                              # make the polarization move

    heatmapV = [np.nan for x in Vangle]                                         # Initialize heatmap array with all point = NaN
    heatmapH = [np.nan for x in Hangle]                                         # Initialize heatmap array with all point = NaN

    i = 0
    total = len(Hangle) + len(Vangle)                                           # total number of points
    vert = 0                                                                    # Hsweep with vert=0
    move_angle(vang=vert, accuracy=accuracy)                                    # jump to vert angle

    inst.fix_status()                                                           # check and run calibration, if needed

    for hori in Hangle:                                                         # loop for horizontal motion

        if kbhit():                                                             # check if key pressed
            if check_abort():                                                   # check if <ESC> pressed
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to home and abort
                csvplot.close()
                if six.PY2:
                    plt.close('all')                                            # automatically close plot for Py2.x
                else:
                    print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                    plt.ioff()
                    plt.show(block=True)                                        # manually close plot for Py3.x
                return

        move_angle(hang=hori, accuracy=accuracy)                                # make the move

        time.sleep(meas_delay)                                                  # optional delay after movement before measuring
        val, freq = get_power(inst)                                             # #####################  this is where you get the value from measurement ####################

        actual_H = convertpostoangle(H, current_pos(H, 1))                      # record actual absolute position motor H reached
        actual_V = convertpostoangle(V, current_pos(V, 1))                      # record actual absolute position motor V reached
        if num_motors >= 4:
            actual_P = convertpostoangle(P, current_pos(P, 1))
            if len(val) == 1:
                print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))
            else:
                print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))

            entry = [vert, actual_V,  hori, actual_H, pangle, actual_P] + val   # record a new plot entry

        else:
            if len(val) == 1:
                print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,val[freqIdx]))
            else:
                print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,val[freqIdx]))

            entry = [vert, actual_V,  hori, actual_H] + val                     # record a new plot entry

        capture.writerow(entry)                                                 # commit to CSV file

        heatmapH[i] = val[freqIdx]                                              # append heatmap with actual captured val
        i += 1                                                                  # update counter

        n = i                                                                   # compute iterations completed
        t1 = time.time()                                                        # get the current time
        elapsed = t1 - t0                                                       # compute elapsed time
        total_time = elapsed / n * total                                        # estimate total time
        remain = total_time - elapsed                                           # compute remaining time
        print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
              (100.0 * n / total,
               datetime.timedelta(seconds=int(total_time)),
               datetime.timedelta(seconds=int(elapsed)),
               datetime.timedelta(seconds=int(remain))))                        # print % complete and time remaining

        if hori == Hangle[-1]:
            if num_motors >= 4:
                move_angle(0, 0, pangle, accuracy)                              # go to (0,0,pol) after last point
            else:
                gotoZERO(accuracy)                                              # go to (0,0) after last point

        if plot == 1:
            blocking = 0                                                        # set interactive (non-blocking) plot
            display_hvplot(Vangle,Hangle,heatmapV,heatmapH,blocking,plot_freq=freq[freqIdx],pangle=pangle)    # update the line plot after each data point

    i = 0
    hori = 0                                                                    # Vsweep with hori=0
    move_angle(hang=hori, accuracy=accuracy)                                    # jump to hori angle

    inst.fix_status()                                                           # check and run calibration, if needed

    for vert in Vangle:                                                         # loop for vertical motion

        if kbhit():                                                             # check if key pressed
            if check_abort():                                                   # check if <ESC> pressed
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to home and abort
                csvplot.close()
                if six.PY2:
                    plt.close('all')                                            # automatically close plot for Py2.x
                else:
                    print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                    plt.ioff()
                    plt.show(block=True)                                        # manually close plot for Py3.x
                return

        move_angle(vang=vert, accuracy=accuracy)                                # make the move

        time.sleep(meas_delay)                                                  # optional delay after movement before measuring
        val, freq = get_power(inst)                                             # #####################  this is where you get the value from measurement ####################

        actual_H = convertpostoangle(H, current_pos(H, 1))                      # record actual absolute position moto H reached
        actual_V = convertpostoangle(V, current_pos(V, 1))                      # record actual absolute position moto V reached
        if num_motors >= 4:
            actual_P = convertpostoangle(P, current_pos(P, 1))
            if len(val) == 1:
                print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))
            else:
                print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))

            entry = [vert, actual_V,  hori, actual_H, pangle, actual_P] + val   # record a new plot entry

        else:
            if len(val) == 1:
                print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,val[freqIdx]))
            else:
                print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,val[freqIdx]))

            entry = [vert, actual_V,  hori, actual_H] + val                     # record a new plot entry
        capture.writerow(entry)                                                 # commit to CSV file

        heatmapV[i] = val[freqIdx]                                              # append heatmap with actual captured val
        i += 1                                                                  # update counter

        n = i + len(Hangle)                                                     # compute iterations completed
        t1 = time.time()                                                        # get the current time
        elapsed = t1 - t0                                                       # compute elapsed time
        total_time = elapsed / n * total                                        # estimate total time
        remain = total_time - elapsed                                           # compute remaining time
        print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
              (100.0 * n / total,
               datetime.timedelta(seconds=int(total_time)),
               datetime.timedelta(seconds=int(elapsed)),
               datetime.timedelta(seconds=int(remain))))                        # print % complete and time remaining

        if vert == Vangle[-1]:                                                  # last point
            try:
                inst.cont_trigger()                                             # enable cont_trigger if it has been implemented
            except:
                pass
            gotoZERO(accuracy)                                                  # go to (0,0)
            t1 = time.time()
            print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))  # display elapsed time
        if plot == 1:
            blocking = 0                                                        # set interactive (non-blocking) plot
            display_hvplot(Vangle,Hangle,heatmapV,heatmapH,blocking,plot_freq=freq[freqIdx],pangle=pangle)    # update the line plot after each data point

    csvplot.close()                                                             # close the CSV file
    print("## THE PLOT WAS SAVED IN FILE :  " +str(filename) + "    ##")        # tell user where to find CSV file

    if plot == 1:
        blocking = 1
        display_hvplot(Vangle,Hangle,heatmapV,heatmapH,blocking,plot_freq=freq[freqIdx],pangle=pangle)        # re-plot data as blocking

    return


def millibox_hvsweep_sph(min, max, step, dphi, plot, tag, inst, accuracy="HIGH", meas_delay=0, plot_freq=0, validonly=True):
    """ 1D sweep in Phi=0 and Phi=90 planes - capture, plot and save the data """

    if num_motors < 6:                                                          # for GIM05_FIXED target DPHI=nan (undefined)
        dphi = float("nan")

    t0 = time.time()                                                            # get the start time for routine
    timeStr = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())                # get day and time to build unique file names
    outdir = os.path.join('..', '..', 'MilliBox_plot_data')                     # outdir is ..\..\MilliBox_plot_data
    if not os.path.isdir(outdir):                                               # check if directory exists
        print("*** Creating output directory MilliBox_plot_data ***")
        os.mkdir(outdir)                                                        # create directory if it doesn't exist
    basename = os.path.join(outdir, 'mbx_capture_'+timeStr+'_hv_sph_'+tag)      # format base filename
    filename = ("%s.csv" % basename)                                            # format CSV filename
    print(" Plot data is saved in file : %s" % filename)                        # tell user filename
    csvplot = open(filename, 'w', buffering=1)                                  # open CSV file for write
    capture = csv.writer(csvplot, lineterminator='\n')                          # set line terminator to newline only (no carraige return)

    if proc.is_corr_on() and proc.get_corr_write():                             # check if corr factor ON and write ON
        corrname = ("%s_corr.csv" % basename)                                   # append _corr to filename
        print(" Corr factor data is saved in file : %s" % corrname)             # tell user filename
        proc.save_corr_file(corrname)                                           # save correction file

    val, freq = get_power(inst)                                                 # query the frequency points

    capture.writerow(['PHI', 'actual_PHI', 'THETA', 'actual_THETA', 'DPHI', 'actual_DPHI'] + freq)  # write the column headers to file (include pol)

    freqIdx = np.abs(np.array(freq) - plot_freq).argmin()                       # find index for value that is closest to plot_freq
    print("\n**** Plotting frequency = %0.3fGHz ****\n" % (freq[freqIdx]/1.0e9))

    num = int(np.floor((max-min)/step))                                         # map of THETA angle iteration
    THangle = np.linspace(min,min+num*step,num+1)                               # [min:step:max] with endpoints inclusive
    PHangle = np.array([0.0, 90.0])

    if validonly:                                                               # filter points within angle limits
        gim_motion = get_gim_motion()
        THlim = gim_motion[1]["anglelim"]
        THangle = THangle[np.intersect1d(np.nonzero(THangle >= THlim[0]), np.nonzero(THangle <= THlim[1]))]

    print(" PH range is = " + str(PHangle))                                     # log tracker
    print(" TH range is = " + str(THangle))                                     # log tracker
    if num_motors >= 6:
        print(" DPHI position is = " + str(dphi))

    heatmapPH00 = [np.nan for x in THangle]                                     # Initialize heatmap array with all point = NaN
    heatmapPH90 = [np.nan for x in THangle]                                     # Initialize heatmap array with all point = NaN

    i = 0
    total = len(THangle) * len(PHangle)                                         # total number of points
    phi = 0                                                                     # THETA sweep with PHI=0
    move_angle(phang=[phi, dphi], accuracy=accuracy)                            # jump to PHI angle

    inst.fix_status()                                                           # check and run calibration, if needed

    for theta in THangle:                                                       # loop for THETA motion

        if kbhit():                                                             # check if key pressed
            if check_abort():                                                   # check if <ESC> pressed
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to home and abort
                csvplot.close()
                if six.PY2:
                    plt.close('all')                                            # automatically close plot for Py2.x
                else:
                    print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                    plt.ioff()
                    plt.show(block=True)                                        # manually close plot for Py3.x
                return

        move_angle(thang=theta, accuracy=accuracy)                              # make the move

        time.sleep(meas_delay)                                                  # optional delay after movement before measuring
        val, freq = get_power(inst)                                             # #####################  this is where you get the value from measurement ####################

        actual_TH = convertpostoangle(TH, current_pos(TH, 1))                   # record actual absolute position motor TH reached
        actual_PH, actual_DPH = convertpostoangle(PH, current_pos(PH, 1))       # record actual absolute position motor PH reached

        if len(val) == 1:
            print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=%0.2f" % (phi,actual_PH,theta,actual_TH,dphi,actual_DPH,val[freqIdx]))
        else:
            print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=[ ... %0.2f ... ]" % (phi,actual_PH,theta,actual_TH,dphi,actual_DPH,val[freqIdx]))

        entry = [phi, actual_PH, theta, actual_TH, dphi, actual_DPH] + val  # record a new plot entry

        capture.writerow(entry)                                                 # commit to CSV file

        heatmapPH00[i] = val[freqIdx]                                           # append heatmap with actual captured val
        i += 1                                                                  # update counter

        n = i                                                                   # compute iterations completed
        t1 = time.time()                                                        # get the current time
        elapsed = t1 - t0                                                       # compute elapsed time
        total_time = elapsed / n * total                                        # estimate total time
        remain = total_time - elapsed                                           # compute remaining time
        print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
              (100.0 * n / total,
               datetime.timedelta(seconds=int(total_time)),
               datetime.timedelta(seconds=int(elapsed)),
               datetime.timedelta(seconds=int(remain))))                        # print % complete and time remaining

        if theta == THangle[-1]:
            if num_motors >= 6:
                move_angle(thang=0, phang=[0, dphi], accuracy=accuracy)         # go to (0,0,dphi) after last point
            else:
                gotoZERO(accuracy)                                              # go to (0,0) after last point

        if plot == 1:
            blocking = 0                                                        # set interactive (non-blocking) plot
            display_hvplot_sph(PHangle,THangle,heatmapPH00,heatmapPH90,blocking,plot_freq=freq[freqIdx],dphi=dphi)    # update the line plot after each data point

    i = 0
    phi = 90                                                                    # THETA sweep with PHI=90
    move_angle(phang=[phi, dphi], accuracy=accuracy)                            # jump to PHI angle

    inst.fix_status()                                                           # check and run calibration, if needed

    for theta in THangle:                                                       # loop for THETA motion

        if kbhit():                                                             # check if key pressed
            if check_abort():                                                   # check if <ESC> pressed
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to home and abort
                csvplot.close()
                if six.PY2:
                    plt.close('all')                                            # automatically close plot for Py2.x
                else:
                    print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                    plt.ioff()
                    plt.show(block=True)                                        # manually close plot for Py3.x
                return

        move_angle(thang=theta, accuracy=accuracy)                              # make the move

        time.sleep(meas_delay)                                                  # optional delay after movement before measuring
        val, freq = get_power(inst)                                             # #####################  this is where you get the value from measurement ####################

        actual_TH = convertpostoangle(TH, current_pos(TH, 1))                   # record actual absolute position motor TH reached
        actual_PH, actual_DPH = convertpostoangle(PH, current_pos(PH, 1))       # record actual absolute position motor PH reached

        if len(val) == 1:
            print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=%0.2f" % (phi,actual_PH,theta,actual_TH,dphi,actual_DPH,val[freqIdx]))
        else:
            print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=[ ... %0.2f ... ]" % (phi,actual_PH,theta,actual_TH,dphi,actual_DPH,val[freqIdx]))

        entry = [phi, actual_PH, theta, actual_TH, dphi, actual_DPH] + val  # record a new plot entry

        capture.writerow(entry)                                                 # commit to CSV file

        heatmapPH90[i] = val[freqIdx]                                           # append heatmap with actual captured val
        i += 1                                                                  # update counter

        n = i + len(THangle)                                                    # compute iterations completed
        t1 = time.time()                                                        # get the current time
        elapsed = t1 - t0                                                       # compute elapsed time
        total_time = elapsed / n * total                                        # estimate total time
        remain = total_time - elapsed                                           # compute remaining time
        print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
              (100.0 * n / total,
               datetime.timedelta(seconds=int(total_time)),
               datetime.timedelta(seconds=int(elapsed)),
               datetime.timedelta(seconds=int(remain))))                        # print % complete and time remaining

        if theta == THangle[-1]:                                                # last point
            try:
                inst.cont_trigger()                                             # enable cont_trigger if it has been implemented
            except:
                pass
            gotoZERO(accuracy)                                                  # go to (0,0)
            t1 = time.time()
            print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))  # display elapsed time
        if plot == 1:
            blocking = 0                                                        # set interactive (non-blocking) plot
            display_hvplot_sph(PHangle,THangle,heatmapPH00,heatmapPH90,blocking,plot_freq=freq[freqIdx],dphi=dphi)    # update the line plot after each data point

    csvplot.close()                                                             # close the CSV file
    print("## THE PLOT WAS SAVED IN FILE :  " +str(filename) + "    ##")        # tell user where to find CSV file

    if plot == 1:
        blocking = 1
        display_hvplot_sph(PHangle,THangle,heatmapPH00,heatmapPH90,blocking,plot_freq=freq[freqIdx],dphi=dphi)        # re-plot data as blocking

    return


def millibox_2dsweep(minh, maxh, minv, maxv, step, pangle, plot, tag, inst, accuracy="HIGH", meas_delay=0, plot_freq=0, zigzag=False, validonly=True):
    """ 2D sweep - capture, plot and save the data - HV gimbal """

    t0 = time.time()                                                            # get the start time for routine
    timeStr = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())                # get day and time to build unique file names
    outdir = os.path.join('..', '..', 'MilliBox_plot_data')                     # outdir is ..\..\MilliBox_plot_data
    if not os.path.isdir(outdir):                                               # check if directory exists
        print("*** Creating output directory MilliBox_plot_data ***")
        os.mkdir(outdir)                                                        # create directory if it doesn't exist
    basename = os.path.join(outdir, 'mbx_capture_'+timeStr+'_2d_'+tag)          # format base filename
    filename = ("%s.csv" % basename)                                            # format CSV filename
    print(" Plot data is saved in file : %s" % filename)                        # tell user filename
    csvplot = open(filename, 'w', buffering=1)                                  # open CSV file for write
    capture = csv.writer(csvplot, lineterminator='\n')                          # set line terminator to newline only (no carraige return)

    if proc.is_corr_on() and proc.get_corr_write():                             # check if corr factor ON and write ON
        corrname = ("%s_corr.csv" % basename)                                   # append _corr to filename
        print(" Corr factor data is saved in file : %s" % corrname)             # tell user filename
        proc.save_corr_file(corrname)                                           # save correction file

    val, freq = get_power(inst)                                                 # query the frequency points

    if num_motors >= 4:
        capture.writerow(['V', 'actual_V', 'H', 'actual_H', 'P', 'actual_P'] + freq)    # write the column headers to file (include pol)
    else:
        capture.writerow(['V', 'actual_V', 'H', 'actual_H'] + freq)             # write the column headers to file
    freqIdx = np.abs(np.array(freq) - plot_freq).argmin()                       # find index for value that is closest to plot_freq
    print("\n**** Plotting frequency = %0.3fGHz ****\n" % (freq[freqIdx]/1.0e9))

    num = int(np.floor((maxv-minv)/step))                                       # map of vertical angle iteration
    Vangle = np.linspace(minv, minv+num*step, num+1)                            # [min:step:max] with endpoints inclusive
    num = int(np.floor((maxh-minh)/step))                                       # map of horizontal angle iteration
    Hangle = np.linspace(minh, minh+num*step, num+1)                            # [min:step:max] with endpoints inclusive

    if validonly:                                                               # filter points within angle limits
        gim_motion = get_gim_motion()
        Hlim = gim_motion[1]["anglelim"]
        Hangle = Hangle[np.intersect1d(np.nonzero(Hangle >= Hlim[0]), np.nonzero(Hangle <= Hlim[1]))]
        if num_motors >= 2:
            Vlim = gim_motion[2]["anglelim"]
            Vangle = Vangle[np.intersect1d(np.nonzero(Vangle >= Vlim[0]), np.nonzero(Vangle <= Vlim[1]))]

    heatmap = [[np.nan for x in Vangle] for y in Hangle]                        # Initialize heamap array with x is V, y is H with all point = NaN
    print(" V range is = " + str(Vangle))                                       # log tracker
    print(" H range is = " + str(Hangle))                                       # log tracker
    i = j = 0                                                                   # init loop counters
    total = len(Hangle) * len(Vangle)                                           # number of measurement points
    direction = 1                                                               # init direction for H movement
    blocking = False                                                            # only set final plot to blocking

    if num_motors >= 4:                                                         # if GIM04
        move_angle(pang=pangle, accuracy=accuracy)                              # move P to target position
    for vert in Vangle:                                                         # loop for vertical motion
        move_angle(vang=vert, accuracy=accuracy)                                # make the vertical move

        inst.fix_status()                                                       # check and run calibration, if needed

        entry_H = []                                                            # store all H sweep in single list before dumping to CSV
        if direction == 1:
            Hindex = range(0, len(Hangle), 1)                                   # set index to ascending
        else:
            Hindex = range(len(Hangle)-1, -1, -1)                               # set index to descending

        for hi in Hindex:                                                       # loop for horizontal motion
            hori = Hangle[hi]                                                   # H angle at current index

            if kbhit():                                                         # check for abort
                if check_abort():
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)
                    csvplot.close()
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return

            move_angle(hang=hori, accuracy=accuracy)                            # make the horizontal move

            time.sleep(meas_delay)                                              # optional delay after movement before measuring
            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            actual_H = convertpostoangle(H, current_pos(H, 1))                  # record actual absolute position motor H reached
            actual_V = convertpostoangle(V, current_pos(V, 1))                  # record actual absolute position motor V reached
            if num_motors >= 4:
                actual_P = convertpostoangle(P, current_pos(P, 1))              # record actual absolute position motor V reached
                if len(val) == 1:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))
                else:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,pangle,actual_P,val[freqIdx]))
                entry = [vert, actual_V,  hori, actual_H, pangle, actual_P] + val       # record a new plot entry
            else:
                if len(val) == 1:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=%0.2f" % (vert,actual_V,hori,actual_H,val[freqIdx]))
                else:
                    print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=[ ... %0.2f ... ]" % (vert,actual_V,hori,actual_H,val[freqIdx]))

                entry = [vert, actual_V,  hori, actual_H] + val                 # record a new plot entry

            if direction == 1:
                entry_H = entry_H + [entry]                                     # append entry to data if normal direction
            else:
                entry_H = [entry] + entry_H                                     # prepend entry to data if reverse direction

            heatmap[hi][j] = val[freqIdx]                                       # append heatmap with actual captured val

            i += 1                                                              # update H counter
            n = i + j * len(Hangle)                                             # compute iterations completed
            t1 = time.time()                                                    # get the current time
            elapsed = t1 - t0                                                   # compute elapsed time
            total_time = elapsed / n * total                                    # estimate total time
            remain = total_time - elapsed                                       # compute remaining time
            print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
                  (100.0 * n / total,
                   datetime.timedelta(seconds=int(total_time)),
                   datetime.timedelta(seconds=int(elapsed)),
                   datetime.timedelta(seconds=int(remain))))                    # print % complete and time remaining

            if vert == Vangle[-1] and hori == Hangle[Hindex[-1]]:               # last point
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to (0,0)
                print("## THE PLOT WAS SAVED IN FILE :  " + str(filename) + "    ##")  # tell user where to find CSV file
                t1 = time.time()
                print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))   # display elapsed time
                blocking = True                                                 # set last plot to blocking

            if plot == 3:                                                       # plot multi-line plot after each data point
                display_multilineplot(Vangle, Hangle, heatmap, vert, hori, plot_freq=freq[freqIdx], pangle=pangle, blocking=blocking)

        capture.writerows(entry_H)                                              # commit all H data to CSV file
        if zigzag:
            direction = -1 * direction                                          # switch direction if zigzag

        j += 1                                                                  # update V counter
        i = 0                                                                   # reset H counter

        if plot == 1:                                                           # interactive plot is not recommended for large plot, as it takes too much CPU resource
            display_surfplot(Vangle, Hangle, heatmap, vert, hori, plot_freq=freq[freqIdx], pangle=pangle, blocking=blocking)  # pass all values for interactive plot, last slice call to plot will be blocking here

        if plot == 2:                                                           # interactive plot is not recommended for large plot, as it takes too much CPU resource
            display_heatmap(Vangle, Hangle, heatmap, vert, hori, plot_freq=freq[freqIdx], pangle=pangle, blocking=blocking)  # pass all values for interactive plot, last slice call to plot will be blocking here

        print("")

    csvplot.close()                                                             # close the CSV file

    if plot > 0:
        display_millibox3d_ant_pattern(Vangle, Hangle, heatmap, vert, hori, step, plot_freq=freq[freqIdx], pangle=pangle, blocking=blocking)  # display 3D radiation pattern plot

    return


def millibox_2dsweep_sph(minth, maxth, minph, maxph, step, dphi, plot, tag, inst, accuracy="HIGH", meas_delay=0, plot_freq=0, zigzag=False, validonly=True):
    """ 2D sweep - capture, plot and save the data - SPHERICAL gimbal """

    if num_motors < 6:                                                          # for GIM05_FIXED target DPHI=nan (undefined)
        dphi = float("nan")

    t0 = time.time()                                                            # get the start time for routine
    timeStr = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())                # get day and time to build unique file names
    outdir = os.path.join('..', '..', 'MilliBox_plot_data')                     # outdir is ..\..\MilliBox_plot_data
    if not os.path.isdir(outdir):                                               # check if directory exists
        print("*** Creating output directory MilliBox_plot_data ***")
        os.mkdir(outdir)                                                        # create directory if it doesn't exist
    basename = os.path.join(outdir, 'mbx_capture_'+timeStr+'_2d_sph_'+tag)      # format base filename
    filename = ("%s.csv" % basename)                                            # format CSV filename
    print(" Plot data is saved in file : %s" % filename)                        # tell user filename
    csvplot = open(filename, 'w', buffering=1)                                  # open CSV file for write
    capture = csv.writer(csvplot, lineterminator='\n')                          # set line terminator to newline only (no carraige return)

    if proc.is_corr_on() and proc.get_corr_write():                             # check if corr factor ON and write ON
        corrname = ("%s_corr.csv" % basename)                                   # append _corr to filename
        print(" Corr factor data is saved in file : %s" % corrname)             # tell user filename
        proc.save_corr_file(corrname)                                           # save correction file

    val, freq = get_power(inst)                                                 # query the frequency points

    capture.writerow(['PHI', 'actual_PHI', 'THETA', 'actual_THETA', 'DPHI', 'actual_DPHI'] + freq)    # write the column headers to file

    freqIdx = np.abs(np.array(freq) - plot_freq).argmin()                       # find index for value that is closest to plot_freq
    print("\n**** Plotting frequency = %0.3fGHz ****\n" % (freq[freqIdx]/1.0e9))

    num = int(np.floor((maxph-minph)/step))                                     # map of PHI angle iteration
    PHangle = np.linspace(minph, minph+num*step, num+1)                         # [min:step:max] with endpoints inclusive
    num = int(np.floor((maxth-minth)/step))                                     # map of THETA angle iteration
    THangle = np.linspace(minth, minth+num*step, num+1)                         # [min:step:max] with endpoints inclusive

    if validonly:                                                               # filter points within angle limits
        gim_motion = get_gim_motion()
        THlim = gim_motion[1]["anglelim"]
        THangle = THangle[np.intersect1d(np.nonzero(THangle >= THlim[0]), np.nonzero(THangle <= THlim[1]))]
        if num_motors >= 6:
            Tlim = gim_motion[5]["anglelim"]
            Zlim = gim_motion[6]["anglelim"]
            ph_idx = np.intersect1d(np.nonzero(PHangle >= Tlim[0]), np.nonzero(PHangle <= Tlim[1]))
            dph_idx = np.intersect1d(np.nonzero(PHangle - dphi >= Zlim[0]), np.nonzero(PHangle - dphi <= Zlim[1]))
            PHangle = PHangle[np.intersect1d(ph_idx, dph_idx)]
        elif num_motors >= 5:
            Tlim = gim_motion[5]["anglelim"]
            ph_idx = np.intersect1d(np.nonzero(PHangle >= Tlim[0]), np.nonzero(PHangle <= Tlim[1]))
            PHangle = PHangle[ph_idx]

    print(" PH range is = " + str(PHangle))                                     # log tracker
    print(" TH range is = " + str(THangle))                                     # log tracker
    if num_motors >= 6:
        print(" DPHI position is = " + str(dphi))

    heatmap = [[np.nan for x in PHangle] for y in THangle]                      # Initialize heamap array with x is PH, y is TH with all point = NaN
    i = j = 0                                                                   # init loop counters
    total = len(THangle) * len(PHangle)                                         # number of measurement points
    direction = 1                                                               # init direction for H movement
    blocking = False                                                            # only set final plot to blocking

    for phi in PHangle:                                                         # loop for PHI motion
        move_angle(phang=[phi, dphi], accuracy=accuracy)                        # make the PHI move

        inst.fix_status()                                                       # check and run calibration, if needed

        entry_TH = []                                                           # store all THETA sweep in single list before dumping to CSV
        if direction == 1:
            THindex = range(0, len(THangle), 1)                                 # set index to ascending
        else:
            THindex = range(len(THangle)-1, -1, -1)                             # set index to descending

        for thi in THindex:                                                     # loop for THETA motion
            theta = THangle[thi]                                                # THETA angle at current index

            if kbhit():                                                         # check for abort
                if check_abort():
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)
                    csvplot.close()
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return

            move_angle(thang=theta, accuracy=accuracy)                          # make the THETA move

            time.sleep(meas_delay)                                              # optional delay after movement before measuring
            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            actual_TH = convertpostoangle(TH, current_pos(TH, 1))               # record actual absolute position motor TH reached
            actual_PH, actual_DPH = convertpostoangle(PH, current_pos(PH, 1))   # record actual absolute position motor PH reached

            if len(val) == 1:
                print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=%0.2f" % (phi, actual_PH, theta, actual_TH, dphi, actual_DPH, val[freqIdx]))
            else:
                print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=[ ... %0.2f ... ]" % (phi, actual_PH, theta, actual_TH, dphi, actual_DPH, val[freqIdx]))

            entry = [phi, actual_PH, theta, actual_TH, dphi, actual_DPH] + val  # record a new plot entry

            if direction == 1:
                entry_TH = entry_TH + [entry]                                   # append entry to data if normal direction
            else:
                entry_TH = [entry] + entry_TH                                   # prepend entry to data if reverse direction

            heatmap[thi][j] = val[freqIdx]                                      # append heatmap with actual captured val

            i += 1                                                              # update THETA counter
            n = i + j * len(THangle)                                            # compute iterations completed
            t1 = time.time()                                                    # get the current time
            elapsed = t1 - t0                                                   # compute elapsed time
            total_time = elapsed / n * total                                    # estimate total time
            remain = total_time - elapsed                                       # compute remaining time
            print("%5.1f %% complete     %s total est     %s elapsed     %s remaining" %
                  (100.0 * n / total,
                   datetime.timedelta(seconds=int(total_time)),
                   datetime.timedelta(seconds=int(elapsed)),
                   datetime.timedelta(seconds=int(remain))))                    # print % complete and time remaining

            if phi == PHangle[-1] and theta == THangle[THindex[-1]]:            # last point
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                gotoZERO(accuracy)                                              # go to (0,0)
                print("## THE PLOT WAS SAVED IN FILE :  " + str(filename) + "    ##")  # tell user where to find CSV file
                t1 = time.time()
                print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))   # display elapsed time
                blocking = True                                                 # set last plot to blocking

        capture.writerows(entry_TH)                                             # commit all THETA data to CSV file
        if zigzag:
            direction = -1 * direction                                          # switch direction if zigzag

        j += 1                                                                  # update PHI counter
        i = 0                                                                   # reset THETA counter

        if plot == 1:                                                           # interactive plot is not recommended for large plot, as it takes too much CPU resource
            display_dir_cosine_sph(PHangle, THangle, heatmap, phi, theta, plot_freq=freq[freqIdx], dphi=dphi, blocking=blocking)  # pass all values for interactive plot, last slice call to plot will be blocking here
        if plot == 2:                                                           # interactive plot is not recommended for large plot, as it takes too much CPU resource
            display_polar_sph(PHangle, THangle, heatmap, phi, theta, plot_freq=freq[freqIdx], dphi=dphi, blocking=blocking)  # pass all values for interactive plot, last slice call to plot will be blocking here

        print("")

    csvplot.close()                                                             # close the CSV file

    if plot > 0:
        display_millibox3d_ant_pattern_sph(PHangle, THangle, heatmap, phi, theta, step, plot_freq=freq[freqIdx], dphi=dphi, blocking=blocking)  # display 3D radiation pattern plot

    return


def millibox_pat_sweep(pat_file, tag, inst, accuracy="HIGH", meas_delay=0, plot_freq=0):
    """ CSV-file defined pattern sweep - capture and save the data """

    patDir1 = os.path.join('..', '..', '_Internal', 'Patterns')                 # pattern search directory is ..\..\_Internal\Patterns
    patDir2 = os.path.join('.', 'patterns')                                     # pattern search directory is .\patterns
    path = ['', patDir1, patDir2]                                               # search path order

    filefound = False
    for pathdir in path:                                                        # search through the path in order
        if not filefound:                                                       # if file hasn't been found
            fullfile = os.path.join(pathdir, pat_file)                          # append the path to the filename
            if os.path.isfile(fullfile):
                filefound = True                                                # found = True
                pat_file = fullfile                                             # set pat_file with absolute path

    if not filefound:                                                           # if no file found
        if pat_file != '':
            print("*** ERROR: Pattern file %s does not exist! ***" % pat_file)  # if file does not exist, exit routine
        return
    else:
        print(" Pattern file: %s" % pat_file)                                   # display filename with full path

    csvin = open(pat_file, 'r')                                                 # open CSV file for read
    pattern = csv.reader(csvin, lineterminator='\n')                            # set line terminator to newline only (no carriage return)

    header = next(pattern)                                                      # check if pattern file type matches gimbal type
    if gim_type == HV:
        if header[0] != "H" or header[1] != "V" or header[2] != "P":
            print("*** ERROR: Pattern file does not match HV gimbal (H,V,P) ***")
            return
    if gim_type == SPHERICAL:
        if header[0] != "THETA" or header[1] != "PHI" or header[2] != "DPHI":
            print("*** ERROR: Pattern file does not match SPHERICAL gimbal (THETA,PHI,DPHI) ***")
            return

    t0 = time.time()                                                            # get the start time for routine
    timeStr = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())                # get day and time to build unique file names
    outdir = os.path.join('..', '..', 'MilliBox_plot_data')                     # outdir is ..\..\MilliBox_plot_data
    if not os.path.isdir(outdir):                                               # check if directory exists
        print("*** Creating output directory MilliBox_plot_data ***")
        os.mkdir(outdir)                                                        # create directory if it doesn't exist
    basename = os.path.join(outdir, 'mbx_capture_'+timeStr+'_pat_'+tag)         # format base filename
    filename = ("%s.csv" % basename)                                            # format CSV filename
    print(" Plot data is saved in file : %s" % filename)                        # tell user filename
    csvplot = open(filename, 'w', buffering=1)                                  # open CSV file for write
    capture = csv.writer(csvplot, lineterminator='\n')                          # set line terminator to newline only (no carraige return)

    if proc.is_corr_on() and proc.get_corr_write():                             # check if corr factor ON and write ON
        corrname = ("%s_corr.csv" % basename)                                   # append _corr to filename
        print(" Corr factor data is saved in file : %s" % corrname)             # tell user filename
        proc.save_corr_file(corrname)                                           # save correction file

    val, freq = get_power(inst)                                                 # query the frequency points

    if gim_type == HV:
        if num_motors >= 4:
            capture.writerow(['V', 'actual_V', 'H', 'actual_H', 'P', 'actual_P'] + freq)  # write the column headers to file (include pol)
        else:
            capture.writerow(['V', 'actual_V', 'H', 'actual_H'] + freq)         # write the column headers to file
    elif gim_type == SPHERICAL:
        if num_motors >= 6:
            capture.writerow(['PHI', 'actual_PHI', 'THETA', 'actual_THETA', 'DPHI', 'actual_DPHI'] + freq)    # write the column headers to file (include pol)

    freqIdx = np.abs(np.array(freq) - plot_freq).argmin()                       # find index for value that is closest to plot_freq
    print("\n**** Plotting frequency = %0.3fGHz ****\n" % (freq[freqIdx]/1.0e9))

    for angles in pattern:
        if gim_type == HV:
            hori = float(angles[0])
            vert = float(angles[1])
            pangle = float(angles[2])
            valid_move = move_angle(hang=hori, vang=vert, pang=pangle, checkonly=True)
        elif gim_type == SPHERICAL:
            theta = float(angles[0])
            phi = float(angles[1])
            dphi = float(angles[2])
            valid_move = move_angle(thang=theta, phang=[phi, dphi], checkonly=True)

        if valid_move:

            inst.fix_status()                                                   # check and run calibration, if needed

            if kbhit():                                                         # check for abort
                if check_abort():
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)
                    csvin.close()
                    csvplot.close()
                    return

            if gim_type == HV:
                move_angle(hang=hori, vang=vert, pang=pangle, accuracy=accuracy)    # make the move

                time.sleep(meas_delay)                                          # optional delay after movement before measuring
                val, freq = get_power(inst)                                     # #####################  this is where you get the value from measurement ####################

                actual_H = convertpostoangle(H, current_pos(H, 1))              # record actual absolute position motor H reached
                actual_V = convertpostoangle(V, current_pos(V, 1))              # record actual absolute position motor V reached
                if num_motors >= 4:
                    actual_P = convertpostoangle(P, current_pos(P, 1))          # record actual absolute position motor P reached
                    if len(val) == 1:
                        print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=%0.2f" %
                              (vert, actual_V, hori, actual_H, pangle, actual_P, val[freqIdx]))
                    else:
                        print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| P=%+8.3f| actual_P=%+8.3f| VALUE=[ ... %0.2f ... ]" %
                              (vert, actual_V, hori, actual_H, pangle, actual_P, val[freqIdx]))
                    entry = [vert, actual_V, hori, actual_H, pangle, actual_P] + val  # record a new plot entry
                    capture.writerow(entry)                                     # commit to CSV file
                else:
                    if len(val) == 1:
                        print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=%0.2f" %
                              (vert, actual_V, hori, actual_H, val[freqIdx]))
                    else:
                        print("capture: V=%+8.3f| actual_V=%+8.3f| H=%+8.3f| actual_H=%+8.3f| VALUE=[ ... %0.2f ... ]" %
                              (vert, actual_V, hori, actual_H, val[freqIdx]))

                    entry = [vert, actual_V, hori, actual_H] + val              # record a new plot entry
                    capture.writerow(entry)                                     # commit to CSV file

            elif gim_type == SPHERICAL:
                move_angle(thang=theta, phang=[phi, dphi], accuracy=accuracy)       # make the move

                time.sleep(meas_delay)                                              # optional delay after movement before measuring
                val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

                actual_TH = convertpostoangle(TH, current_pos(TH, 1))               # record actual absolute position motor TH reached
                actual_PH, actual_DPH = convertpostoangle(PH, current_pos(PH, 1))   # record actual absolute position motor PH reached

                if num_motors >= 6:
                    if len(val) == 1:
                        print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=%0.2f" % (phi, actual_PH, theta, actual_TH, dphi, actual_DPH, val[freqIdx]))
                    else:
                        print("capture: PH=%+8.3f| actual_PH=%+8.3f| TH=%+8.3f| actual_TH=%+8.3f| DPH=%+8.3f| actual_DPH=%+8.3f| VALUE=[ ... %0.2f ... ]" % (phi, actual_PH, theta, actual_TH, dphi, actual_DPH, val[freqIdx]))

                    entry = [phi, actual_PH, theta, actual_TH, dphi, actual_DPH] + val  # record a new plot entry

                    capture.writerow(entry)                                     # commit to CSV file
        else:
            if gim_type == HV:
                print("*** Position (H,V,P) = (%0.2f, %0.2f, %0.2f) out of range .... SKIPPING ***" % (hori, vert, pangle))
            elif gim_type == SPHERICAL:
                print("*** Position (TH,PH,DPH) = (%0.2f, %0.2f, %0.2f) out of range .... SKIPPING ***" % (theta, phi, dphi))

    try:
        inst.cont_trigger()                                                     # enable cont_trigger if it has been implemented
    except:
        pass
    gotoZERO(accuracy)                                                          # go to (0,0)
    print("## THE SWEEP WAS SAVED IN FILE :  " + str(filename) + "    ##")      # tell user where to find CSV file
    t1 = time.time()
    print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1 - t0))))  # display elapsed time

    csvin.close()                                                               # close the CSV pattern file
    csvplot.close()                                                             # close the CSV output file

    return


def milliboxacc(minh, maxh, minv, maxv, step, accuracy="HIGH", zigzag=False, validonly=True):
    """ capture position accuracy data, and save the data - HV gimbal """

    t0 = time.time()                                                            # get the start time for routine
    timeStr = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())                # get day and time to build unique file names
    outdir = os.path.join('..', '..', 'MilliBox_plot_data')                     # outdir is ..\..\MilliBox_plot_data
    if not os.path.isdir(outdir):                                               # check if directory exists
        print("*** Creating output directory MilliBox_plot_data ***")
        os.mkdir(outdir)                                                        # create directory if it doesn't exist
    basename = os.path.join(outdir, 'mbx_accuracy_'+timeStr)                    # format base filename
    filename = ("%s.csv" % basename)                                            # format CSV filename
    print(" accuracy data is saved in file : %s" % filename)                    # tell user filename
    csvplot = open(filename, 'w', buffering=1)                                  # open CSV file for write
    capture = csv.writer(csvplot, lineterminator='\n')                          # set line terminator to newline only (no carraige return)

    capture.writerow(['V', 'Vquant', 'actual_V', 'H', 'Hquant', 'actual_H', 'Verr', 'Herr', 'Vtoterr', 'Htoterr'])        # write the column headers to file

    num = int(np.floor((maxv-minv)/step))                                       # map of vertical angle iteration
    Vangle = np.linspace(minv,minv+num*step,num+1)                              # [min:step:max] with endpoints inclusive
    num = int(np.floor((maxh-minh)/step))                                       # map of horizontal angle iteration
    Hangle = np.linspace(minh,minh+num*step,num+1)                              # [min:step:max] with endpoints inclusive

    if validonly:                                                               # filter points within angle limits
        gim_motion = get_gim_motion()
        Hlim = gim_motion[1]["anglelim"]
        Hangle = Hangle[np.intersect1d(np.nonzero(Hangle >= Hlim[0]), np.nonzero(Hangle <= Hlim[1]))]
        if num_motors >= 2:
            Vlim = gim_motion[2]["anglelim"]
            Vangle = Vangle[np.intersect1d(np.nonzero(Vangle >= Vlim[0]), np.nonzero(Vangle <= Vlim[1]))]

    print(" x: V range is = " + str(Vangle))                                    # log tracker
    print(" y: H range is = " + str(Hangle))                                    # log tracker
    direction = 1                                                               # init direction for H movement

    for vert in Vangle:                                                         # loop for vertical motion
        print("")
        move_angle(vang=vert, accuracy=accuracy)                                # jump to vert position

        entry_H = []                                                            # store all H sweep in single list before dumping to CSV
        if direction == 1:
            Hindex = range(0, len(Hangle), 1)                                   # set index to ascending
        else:
            Hindex = range(len(Hangle)-1, -1, -1)                               # set index to descending

        for hi in Hindex:                                                       # loop for horizontal motion
            hori = Hangle[hi]                                                   # H angle at current index

            if kbhit():
                if check_abort():
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)
                    csvplot.close()
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return

            move_angle(hang=hori, accuracy=accuracy)                            # make the move
            hquant = convertpostoangle(H,round(convertangletopos(H,hori)))      # compute the quantized H target angle
            vquant = convertpostoangle(V,round(convertangletopos(V,vert)))      # compute the quantized V target angle
            actual_H = convertpostoangle(H, current_pos(H, 1))
            actual_V = convertpostoangle(V, current_pos(V, 1))
            herr = actual_H - hquant                                            # error between actual and quantized
            verr = actual_V - vquant
            htoterr = actual_H - hori                                           # error between actual and target
            vtoterr = actual_V - vert
            entry = (vert, vquant, actual_V, hori, hquant, actual_H, verr, herr, vtoterr, htoterr)  # record a new plot entry
            print("capture: V=%+7.2f|V_quant=%+7.2f|actual_V=%+7.2f|H=%+7.2f|Hquant=%+7.2f|actual_H=%+7.2f|verr=%+7.2f|herr=%+7.2f|vtoterr=%+7.2f|htoterr=%+7.2f" % entry)

            if direction == 1:
                entry_H = entry_H + [entry]                                     # append entry to data if normal direction
            else:
                entry_H = [entry] + entry_H                                     # prepend entry to data if reverse direction

        capture.writerows(entry_H)                                              # commit all H data to CSV file
        if zigzag:
            direction = -1 * direction                                          # switch direction if zigzag

    csvplot.close()
    print("## THE PLOT WAS SAVED IN FILE :  " +str(filename) + "    ##")        # tell user where to find CSV file
    gotoZERO(accuracy)                                                          # always return to 0,0 when plot is done
    t1 = time.time()
    print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1 - t0))))  # display elapsed time

    return


def milliboxacc_sph(minth, maxth, minph, maxph, step, accuracy="HIGH", zigzag=False, validonly=True):
    """ capture position accuracy data, and save the data - SPHERICAL gimbal """

    t0 = time.time()                                                            # get the start time for routine
    timeStr = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())                # get day and time to build unique file names
    outdir = os.path.join('..', '..', 'MilliBox_plot_data')                     # outdir is ..\..\MilliBox_plot_data
    if not os.path.isdir(outdir):                                               # check if directory exists
        print("*** Creating output directory MilliBox_plot_data ***")
        os.mkdir(outdir)                                                        # create directory if it doesn't exist
    basename = os.path.join(outdir, 'mbx_accuracy_sph_'+timeStr)                # format base filename
    filename = ("%s.csv" % basename)                                            # format CSV filename
    print(" accuracy data is saved in file : " +str(filename))                  # tell user filename

    csvplot = open(filename, 'w', buffering=1)                                  # open CSV file for write
    capture = csv.writer(csvplot, lineterminator='\n')                          # set line terminator to newline only (no carraige return)
    capture.writerow(['PH', 'PHquant', 'actual_PH', 'TH', 'THquant', 'actual_TH', 'PHerr', 'THerr', 'PHtoterr', 'THtoterr'])        # write the column headers to file

    num = int(np.floor((maxph-minph)/step))                                     # map of PHI angle iteration
    PHangle = np.linspace(minph,minph+num*step,num+1)                           # [min:step:max] with endpoints inclusive
    num = int(np.floor((maxth-minth)/step))                                     # map of THETA angle iteration
    THangle = np.linspace(minth,minth+num*step,num+1)                           # [min:step:max] with endpoints inclusive

    if validonly:                                                               # filter points within angle limits
        dphi = 0
        gim_motion = get_gim_motion()
        THlim = gim_motion[1]["anglelim"]
        THangle = THangle[np.intersect1d(np.nonzero(THangle >= THlim[0]), np.nonzero(THangle <= THlim[1]))]
        if num_motors >= 6:
            Tlim = gim_motion[5]["anglelim"]
            Zlim = gim_motion[6]["anglelim"]
            ph_idx = np.intersect1d(np.nonzero(PHangle >= Tlim[0]), np.nonzero(PHangle <= Tlim[1]))
            dph_idx = np.intersect1d(np.nonzero(PHangle - dphi >= Zlim[0]), np.nonzero(PHangle - dphi <= Zlim[1]))
            PHangle = PHangle[np.intersect1d(ph_idx, dph_idx)]
        elif num_motors >= 5:
            Tlim = gim_motion[5]["anglelim"]
            ph_idx = np.intersect1d(np.nonzero(PHangle >= Tlim[0]), np.nonzero(PHangle <= Tlim[1]))
            PHangle = PHangle[ph_idx]

    print(" PH range is = " + str(PHangle))                                     # log tracker
    print(" TH range is = " + str(THangle))                                     # log tracker
    direction = 1                                                               # init direction for THETA movement

    for phi in PHangle:                                                         # loop for PHI motion
        print("")
        move_angle(phang=[phi, 0], accuracy=accuracy)                           # jump to PHI position

        entry_TH = []                                                           # store all THETA sweep in single list before dumping to CSV
        if direction == 1:
            THindex = range(0, len(THangle), 1)                                 # set index to ascending
        else:
            THindex = range(len(THangle)-1, -1, -1)                             # set index to descending

        for thi in THindex:                                                     # loop for THETA motion
            theta = THangle[thi]                                                # THETA angle at current index

            if kbhit():
                if check_abort():
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)
                    csvplot.close()
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return

            move_angle(thang=theta, accuracy=accuracy)                          # make the move
            thquant = convertpostoangle(TH,round(convertangletopos(TH,theta)))          # compute the quantized TH target angle
            phquant, dphquant = convertpostoangle(PH,convertangletopos(PH,[phi,0]))     # compute the quantized PH target angle
            actual_TH = convertpostoangle(TH, current_pos(TH, 1))
            actual_PH, actual_DPH = convertpostoangle(PH, current_pos(PH, 1))
            therr = actual_TH - thquant                                         # error between actual and quantized
            pherr = actual_PH - phquant
            thtoterr = actual_TH - theta                                        # error between actual and target
            phtoterr = actual_PH - phi
            entry = (phi, phquant, actual_PH, theta, thquant, actual_TH, pherr, therr, phtoterr, thtoterr)  # record a new plot entry
            print("capture: PH=%+7.2f|PH_quant=%+7.2f|actual_PH=%+7.2f|TH=%+7.2f|THquant=%+7.2f|actual_TH=%+7.2f|pherr=%+7.2f|therr=%+7.2f|phtoterr=%+7.2f|thtoterr=%+7.2f" % entry)

            if direction == 1:
                entry_TH = entry_TH + [entry]                                   # append entry to data if normal direction
            else:
                entry_TH = [entry] + entry_TH                                   # prepend entry to data if reverse direction

        capture.writerows(entry_TH)                                             # commit all THETA data to CSV file
        if zigzag:
            direction = -1 * direction                                          # switch direction if zigzag

    csvplot.close()
    print("## THE PLOT WAS SAVED IN FILE :  " +str(filename) + "    ##")        # tell user where to find CSV file
    gotoZERO(accuracy)                                                          # always return to 0,0 when plot is done
    t1 = time.time()
    print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1 - t0))))  # display elapsed time

    return


def center_of_mass(pos, val):
    """ compute center of mass for position/mass vectors """
    center_pos = np.sum(np.array(pos)*np.array(val)*1.0)/np.sum(np.array(val))
    return center_pos


def beam_align_hv_single(inst, minh=-90.0, maxh=90.0, minv=-90.0, maxv=90.0, step=1.0, pangle=0.0, vert0=0.0, accuracy="VERY HIGH", keepplot=False, validonly=True):
    """ electronic alignment of beam peak for HV Gimbal """

    H_off = None
    V_off = None

    if gim_type == HV:
        t0 = time.time()                                                        # get the start time for routine

        plot = 1

        num = int(np.floor((maxv-minv)/step))                                   # map of vertical angle iteration
        Vangle = np.linspace(minv,minv+num*step,num+1)                          # [minv:step:maxv] with endpoints inclusive
        num = int(np.floor((maxh-minh)/step))                                   # map of horizontal angle iteration
        Hangle = np.linspace(minh,minh+num*step,num+1)                          # [minh:step:maxh] with endpoints inclusive

        if validonly:                                                           # filter points within angle limits
            gim_motion = get_gim_motion()
            Hlim = gim_motion[1]["anglelim"]
            Hangle = Hangle[np.intersect1d(np.nonzero(Hangle >= Hlim[0]), np.nonzero(Hangle <= Hlim[1]))]
            if num_motors >= 2:
                Vlim = gim_motion[2]["anglelim"]
                Vangle = Vangle[np.intersect1d(np.nonzero(Vangle >= Vlim[0]), np.nonzero(Vangle <= Vlim[1]))]

        print(" x: V range is = " + str(Vangle))                                # log tracker
        print(" y: H range is = " + str(Hangle))                                # log tracker
        if num_motors >= 4:
            print(" p: Polarization position is = " + str(pangle))
            move_angle(pang=pangle, accuracy=accuracy)                          # make the polarization move

        heatmapV = [np.nan for x in Vangle]                                     # Initialize heatmap array with all point = NaN
        heatmapH = [np.nan for x in Hangle]                                     # Initialize heatmap array with all point = NaN

        print("H alignment sweep")

        i = 0
        print("Moving to V=%0.3f" % vert0)
        vert = vert0                                                            # Hsweep with vert=vert0
        move_angle(vang=vert, accuracy=accuracy)                                # jump to vert angle

        inst.fix_status()                                                       # check and run calibration, if needed

        for hori in Hangle:                                                     # loop for horizontal motion

            if kbhit():                                                         # check if key pressed
                if check_abort():                                               # check if <ESC> pressed
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)                                          # go to home and abort
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return H_off, V_off

            move_angle(hang=hori, accuracy=accuracy)                            # make the move

            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            freqIdx = int(len(val)/2)                                           # choose midpoint index

            heatmapH[i] = val[freqIdx]                                          # append heatmap with actual captured val
            i += 1                                                              # update counter

            if hori == Hangle[-1]:
                if num_motors >= 4:
                    move_angle(0, 0, pangle, accuracy)                          # go to (0,0,pol) after last point
                else:
                    move_angle(0, 0, 0, accuracy)                               # go to (0,0) after last point

            if plot == 1:
                blocking = 0                                                    # set interactive (non-blocking) plot
                display_hvplot(Vangle,Hangle,heatmapV,heatmapH,blocking,plot_freq=freq[freqIdx],pangle=pangle)    # update the line plot after each data point

        heatmap = np.array(heatmapH)*1.0
        heatmap_rel = heatmap - heatmap.max()                                   # compute power relative to peak
        heatmap_mass = 10**(heatmap_rel/5.0)                                    # empirically use 10^(P_rel/5.0)

        H_off = center_of_mass(Hangle, heatmap_mass)                            # calculate center of mass
        print("H center = %0.3f" % H_off)

        print("V alignment sweep")

        i = 0
        print("Moving to H=%0.3f" % H_off)
        hori = H_off                                                            # Vsweep with hori=H_off
        move_angle(hang=hori, accuracy=accuracy)                                # jump to hori angle

        inst.fix_status()                                                       # check and run calibration, if needed

        for vert in Vangle:                                                     # loop for vertical motion

            if kbhit():                                                         # check if key pressed
                if check_abort():                                               # check if <ESC> pressed
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)                                          # go to home and abort
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return H_off, V_off

            move_angle(vang=vert, accuracy=accuracy)                            # make the move

            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            freqIdx = int(len(val)/2)                                           # choose midpoint index

            heatmapV[i] = val[freqIdx]                                          # append heatmap with actual captured val
            i += 1                                                              # update counter

            if vert == Vangle[-1]:                                              # last point
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                move_angle(0, 0, pangle, accuracy)                              # go to (0,0,pol) after last point
                t1 = time.time()
                print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))  # display elapsed time
            if plot == 1:
                blocking = 0                                                    # set interactive (non-blocking) plot
                block_final = False
                display_hvplot(Vangle,Hangle,heatmapV,heatmapH,blocking,plot_freq=freq[freqIdx],pangle=pangle,block_final=block_final)    # update the line plot after each data point

        heatmap = np.array(heatmapV)*1.0
        heatmap_rel = heatmap - heatmap.max()                                   # compute power relative to peak
        heatmap_mass = 10**(heatmap_rel/5.0)                                    # empirically use 10^(P_rel/5.0)

        V_off = center_of_mass(Vangle, heatmap_mass)                            # calculate center of mass
        print("V center = %0.3f" % V_off)

        vert = V_off
        move_angle(vang=vert, accuracy=accuracy)                                # jump to V_off angle

        if plot == 1:
            blocking = 1
            block_final = keepplot                                              # only block last point if final pass
            display_hvplot(Vangle,Hangle,heatmapV,heatmapH,blocking,plot_freq=freq[freqIdx],pangle=pangle,block_final=block_final)        # re-plot data as blocking

        move_angle(hang=H_off, accuracy=accuracy)                               # jump to H_off angle
        move_angle(vang=V_off, accuracy=accuracy)                               # jump to V_off angle

    else:
        print("*** ERROR: Incorrect gimbal type. Cannot run HV electronic alignment")

    return H_off, V_off


def beam_align_hv(inst, pangle=0.0, accuracy="VERY HIGH"):
    """ 2-pass electronic alignment of beam peak of HV Gimbal """

    H1 = None
    V1 = None

    if gim_type == HV:
        t0 = time.time()                                                        # get the start time for routine

        print("Coarse alignment search")                                        # coarse search with -90 90 -90 90 5
        H0, V0 = beam_align_hv_single(inst, -90, 90, -90, 90, 5, pangle, vert0=0.0, accuracy=accuracy, keepplot=False)
        if H0 is None or V0 is None:
            return H1, V1
        print("Coarse alignment center (H,V) = (%0.3f, %0.3f)" % (H0, V0))

        print("Fine alignment search")                                          # fine alignment search with +/-40 from (H0,V0)
        H0x = np.round(H0, 0)
        V0x = np.round(V0, 0)
        H1, V1 = beam_align_hv_single(inst, H0x-40, H0x+40, V0x-40, V0x+40, 1, pangle, vert0=V0, accuracy=accuracy, keepplot=True)
        if H1 is None or V1 is None:
            return H1, V1
        print("Fine alignment center (H,V) = (%0.3f, %0.3f)" % (H1, V1))

        move_angle(hang=H1, vang=V1, accuracy=accuracy)                         # move to found center (H1,V1)

        # t1 = time.time()
        # print("*** Total elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1 - t0))))     # display elapsed time

        print("")
        print("")
        print("******************************************")
        print("**** ELECTRONIC BEAM ALIGNMENT RESULT ****")
        print("******************************************")
        print("")
        print("         (H,V) = (%0.3f, %0.3f)" % (H1, V1))
        print("")

    return H1, V1


def beam_align_sph_single(inst, minth=-90.0, maxth=90.0, minph=-90.0, maxph=90.0, step=1.0, phi0=0.0, accuracy="VERY HIGH", keepplot=False, validonly=True):
    """ electronic alignment of beam peak of Spherical Gimbal """

    TH_off = None
    PH_off = None

    if gim_type == SPHERICAL:
        t0 = time.time()                                                        # get the start time for routine

        plot = 1

        num = int(np.floor((maxph-minph)/step))                                 # map of phi angle iteration
        PHangle = np.linspace(minph,minph+num*step,num+1)                       # [minph:step:maxph] with endpoints inclusive
        num = int(np.floor((maxth-minth)/step))                                 # map of theta angle iteration
        THangle = np.linspace(minth,minth+num*step,num+1)                       # [minth:step:maxth] with endpoints inclusive

        if validonly:                                                           # filter points within angle limits
            gim_motion = get_gim_motion()
            THlim = gim_motion[1]["anglelim"]
            THangle = THangle[np.intersect1d(np.nonzero(THangle >= THlim[0]), np.nonzero(THangle <= THlim[1]))]
            if num_motors >= 5:
                Tlim = gim_motion[5]["anglelim"]
                ph_idx = np.intersect1d(np.nonzero(PHangle >= Tlim[0]), np.nonzero(PHangle <= Tlim[1]))
                PHangle = PHangle[ph_idx]

        print(" x: PH range is = " + str(PHangle))                              # log tracker
        print(" y: TH range is = " + str(THangle))                              # log tracker

        heatmapPH = [np.nan for x in PHangle]                                   # Initialize heatmap array with all point = NaN
        heatmapTH = [np.nan for x in THangle]                                   # Initialize heatmap array with all point = NaN

        print("TH alignment sweep")

        i = 0
        print("Moving to PH=%0.3f" % phi0)
        phi = [phi0, phi0]                                                      # THsweep with PH=[phi0, phi0]
        move_angle(phang=phi, accuracy=accuracy)                                # jump to phi angle

        inst.fix_status()                                                       # check and run calibration, if needed

        for theta in THangle:                                                   # loop for theta motion

            if kbhit():                                                         # check if key pressed
                if check_abort():                                               # check if <ESC> pressed
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)                                          # go to home and abort
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return TH_off, PH_off

            move_angle(thang=theta, accuracy=accuracy)                          # make the move

            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            freqIdx = int(len(val)/2)                                           # choose midpoint index
            # print("%+8.3f, %+8.3f" % (hori, val[freqIdx]))

            heatmapTH[i] = val[freqIdx]                                         # append heatmap with actual captured val
            i += 1                                                              # update counter

            if theta == THangle[-1]:
                # jump_angle_sph(0, [0, 0], accuracy)                             # go to (0,[0,0]) after last point
                move_angle(thang=0, phang=[0, 0], accuracy=accuracy)            # go to (0,[0,0]) after last point

            if plot == 1:
                blocking = 0                                                    # set interactive (non-blocking) plot
                display_hvplot(PHangle,THangle,heatmapPH,heatmapTH,blocking,plot_freq=freq[freqIdx],
                               legend=['TH sweep','PH sweep'])                  # update the line plot after each data point

        heatmap = np.array(heatmapTH)*1.0
        heatmap_rel = heatmap - heatmap.max()                                   # compute power relative to peak
        heatmap_mass = 10**(heatmap_rel/5.0)                                    # empirically use 10^(P_rel/5.0)

        TH_off = center_of_mass(THangle, heatmap_mass)                          # calculate center of mass
        print("TH center = %0.3f" % TH_off)

        print("PH alignment sweep")

        i = 0
        print("Moving to TH=%0.3f" % TH_off)
        theta = TH_off                                                          # PHsweep with theta=TH_off
        move_angle(thang=theta, accuracy=accuracy)                              # jump to theta angle

        inst.fix_status()                                                       # check and run calibration, if needed

        for phi in PHangle:                                                     # loop for phi motion

            if kbhit():                                                         # check if key pressed
                if check_abort():                                               # check if <ESC> pressed
                    try:
                        inst.cont_trigger()                                     # enable cont_trigger if it has been implemented
                    except:
                        pass
                    gotoZERO(accuracy)                                          # go to home and abort
                    if six.PY2:
                        plt.close('all')                                        # automatically close plot for Py2.x
                    else:
                        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
                        plt.ioff()
                        plt.show(block=True)                                    # manually close plot for Py3.x
                    return TH_off, PH_off

            move_angle(phang=[phi, phi], accuracy=accuracy)                     # make the move

            val, freq = get_power(inst)                                         # #####################  this is where you get the value from measurement ####################

            freqIdx = int(len(val)/2)                                           # choose midpoint index

            heatmapPH[i] = val[freqIdx]                                         # append heatmap with actual captured val
            i += 1                                                              # update counter

            if phi == PHangle[-1]:                                              # last point
                try:
                    inst.cont_trigger()                                         # enable cont_trigger if it has been implemented
                except:
                    pass
                move_angle(thang=0, phang=[0, 0], accuracy=accuracy)            # go to (0,[0,0]) after last point
                t1 = time.time()
                print("*** Elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1-t0))))  # display elapsed time
            if plot == 1:
                blocking = 0                                                    # set interactive (non-blocking) plot
                block_final = False
                display_hvplot(PHangle,THangle,heatmapPH,heatmapTH,blocking,plot_freq=freq[freqIdx],block_final=block_final,
                               legend=['TH sweep','PH sweep'])                  # update the line plot after each data point

        heatmap = np.array(heatmapPH)*1.0
        heatmap_rel = heatmap - heatmap.max()                                   # compute power relative to peak
        heatmap_mass = 10**(heatmap_rel/5.0)                                    # empirically use 10^(P_rel/5.0)

        phi = center_of_mass(PHangle, heatmap_mass)                             # calculate center of mass
        print("PH center = %0.3f" % phi)

        PH_off = [phi, phi]
        move_angle(phang=PH_off, accuracy=accuracy)                             # jump to PH_off angle

        if plot == 1:
            blocking = 1
            block_final = keepplot                                              # only block last point if final pass
            display_hvplot(PHangle,THangle,heatmapPH,heatmapTH,blocking,plot_freq=freq[freqIdx],block_final=block_final,
                           legend=['TH sweep','PH sweep'])                      # re-plot data as blocking

        move_angle(thang=TH_off, phang=PH_off, accuracy=accuracy)               # jump to (TH_off,PH_off) angle

    else:
        print("*** ERROR: Incorrect gimbal type. Cannot run SPHERICAL electronic alignment")

    return TH_off, PH_off


def beam_align_sph(inst, accuracy="VERY HIGH"):
    """ 2-pass electronic alignment of beam peak of Spherical Gimbal """

    TH1 = None
    PH1 = None

    if gim_type == SPHERICAL:
        # t0 = time.time()                                                        # get the start time for routine

        print("Coarse alignment search")                                        # coarse search with -90 90 -90 90 5
        TH0, PH0 = beam_align_sph_single(inst, -90, 90, -90, 90, 5, phi0=0.0, accuracy=accuracy, keepplot=False)
        if TH0 is None or PH0 is None:
            return TH1, PH1
        print("Coarse alignment center (TH,PH,DPH) = (%0.3f, %0.3f, %0.3f)" % (TH0, PH0[0], PH0[1]))

        print("Fine alignment search")                                          # fine alignment search with +/-40 from (TH0,PH0)
        TH0x = np.round(TH0, 0)
        PH0x = [np.round(PH0[0], 0), np.round(PH0[1], 0)]
        TH1, PH1 = beam_align_sph_single(inst, TH0x-40, TH0x+40, PH0x[0]-40, PH0x[0]+40, 1, phi0=PH0[0], accuracy=accuracy, keepplot=True)
        if TH1 is None or PH1 is None:
            return TH1, PH1

        move_angle(thang=TH1, phang=PH1, accuracy=accuracy)                     # move to found center (TH1,PH1)

        # t1 = time.time()
        # print("*** Total elapsed time = %s ***" % (datetime.timedelta(seconds=int(t1 - t0))))     # display elapsed time

        print("")
        print("")
        print("******************************************")
        print("**** ELECTRONIC BEAM ALIGNMENT RESULT ****")
        print("******************************************")
        print("")
        print("   (TH,PH,DPH) = (%0.3f, %0.3f, %0.3f)" % (TH1, PH1[0], PH1[1]))
        print("")

    return TH1, PH1


# ================================================
# ============= DEPRECATED FUNCTIONS =============
# ================================================

def deprecated_note(old_name, new_name):
    global DEPRECATED_WARNING
    if DEPRECATED_WARNING:
        print("*** WARNING: function ""%s"" is deprecated. It may be removed in a future release." % old_name)
        print("*** Please use function ""%s"" for future compatibility." % new_name)
    return


def jump_H(hpos):
    """ makes H motor move to a given absolute position
    does not wait for motor to stop moving before returning """
    deprecated_note("jump_H", "move_pos")
    move_pos(H, hpos)
    return


def jump_V(vpos):
    """ makes V motor move to a given absolute position
    does not wait for motor to stop moving before returning """
    deprecated_note("jump_V", "move_pos")
    move_pos(V, vpos)
    return


def jump_P(ppos):
    """ makes P motor move to a given absolute position
    does not wait for motor to stop moving before returning """
    deprecated_note("jump_P", "move_pos")
    move_pos(P, ppos)
    return


def jump_T(tpos):
    """ makes T motor move to a given absolute position
    does not wait for motor to stop moving before returning """
    deprecated_note("jump_T", "move_pos")
    move_pos(T, tpos)
    return


def jump_Z(zpos):
    """ makes Z motor move to a given absolute position
    does not wait for motor to stop moving before returning """
    deprecated_note("jump_Z", "move_pos")
    move_pos(Z, zpos)
    return


def jump_TH(thpos):
    """ makes TH motor move to a given absolute position
    does not wait for motor to stop moving before returning """
    deprecated_note("jump_TH", "move_pos")
    move_pos(TH, thpos)
    return


def jump_PH(pos):
    """ makes PH motor move to a given absolute position
    does not wait for motor to stop moving before returning """
    deprecated_note("jump_PH", "move_pos")
    move_pos(PH, pos)
    return


def jump_angle_H(hang, accuracy="HIGH"):
    """ makes H motor move to a given absolute angle """
    deprecated_note("jump_angle_H", "move_angle")
    move_angle(hang=hang, accuracy=accuracy)
    return


def jump_angle_V(vang, accuracy="HIGH"):
    """ makes V motor move to a given absolute angle """
    deprecated_note("jump_angle_V", "move_angle")
    move_angle(vang=vang, accuracy=accuracy)
    return


def jump_angle_P(pang, accuracy="HIGH"):
    """ makes P motor move to a given absolute angle """
    deprecated_note("jump_angle_P", "move_angle")
    move_angle(pang=pang, accuracy=accuracy)
    return


def jump_angle_TH(thang, accuracy="HIGH"):
    """ makes TH motor move to a given absolute angle """
    deprecated_note("jump_angle_TH", "move_angle")
    move_angle(thang=thang, accuracy=accuracy)
    return


def jump_angle_PH(phang, accuracy="HIGH"):
    """ makes PH motor move to a given absolute angle """
    deprecated_note("jump_angle_PH", "move_angle")
    move_angle(phang=phang, accuracy=accuracy)
    return


def jump_angle_single(motor, ang, accuracy="HIGH"):
    """ Makes a single motor move to a given absolute angle """
    deprecated_note("jump_angle_single", "move_angle")
    ok = 1
    if motor == H:
        ok = move_angle(hang=ang, accuracy=accuracy)
    elif motor == V:
        ok = move_angle(vang=ang, accuracy=accuracy)
    elif motor == P:
        ok = move_angle(pang=ang, accuracy=accuracy)
    elif motor == TH:
        ok = move_angle(thang=ang, accuracy=accuracy)
    elif motor == PH:
        ok = move_angle(phang=ang, accuracy=accuracy)
    return ok


def jump_angle(hang, vang, pang, accuracy="HIGH"):
    """ makes all motors move to a given absolute angle for HV gimbal """
    deprecated_note("jump_angle", "move_angle")
    ok = move_angle(hang=hang, vang=vang, pang=pang, accuracy=accuracy)
    return ok


def jump_angle_sph(thang=None, phang=None, accuracy="HIGH"):
    """ makes all motors move to a given absolute angle for SPHERICAL gimbal """
    deprecated_note("jump_angle_sph", "move_angle")
    ok = move_angle(thang=thang, phang=phang, accuracy=accuracy)
    return ok


def move(motor, step, accuracy="HIGH"):
    """ Move motor position to a new position relative to current one (and check for boundary limits) - HV gimbal """
    deprecated_note("move", "move_angle_rel")
    ok = move_angle_rel(motor=motor, angstep=step, accuracy=accuracy)
    return


def move_sph(motor, step, accuracy="HIGH"):
    """ Move motor position to a new position relative to current one (and check for boundary limits) - SPHERICAL gimbal """
    deprecated_note("move_sph", "move_angle_rel")
    ok = move_angle_rel(motor=motor, angstep=step, accuracy=accuracy)
    return


def check_move(h_target, v_target, p_target):                                   # check that the move values are in range
    """ check the move is doable """
    deprecated_note("check_move", "move_angle")
    ok = move_angle(hang=h_target, vang=v_target, pang=p_target, checkonly=True)
    return ok


def check_move_sph(theta_target, phi_target):                                   # check that the move values are in range
    """ check the move is doable """
    deprecated_note("check_move_sph", "move_angle")
    ok = move_angle(thang=theta_target, phang=phi_target, checkonly=True)
    return ok
