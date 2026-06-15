#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################################################################
# Copyright 2021-2024 MILLIWAVE SILICON SOLUTIONS, inc.
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


import six
import sys
import time
import mbx_scpi_connection as scpi


connected = False
visa_add = ""
port = []
cmd = ""
logfile = "visa_log.txt"


class Logger:
    """ mirrors STDOUT to screen and log file """

    def __init__(self, stdout, filename):
        self.stdout = stdout
        self.logfile = open(filename, 'a', buffering=1)

    def write(self, text):
        self.stdout.write(text)
        self.stdout.flush()
        self.logfile.write(text)

    def close(self):
        self.stdout.close()
        self.logfile.close()

    def flush(self):
        # this flush method is needed for python 3 compatibility.
        # this handles the flush command by doing nothing.
        # you might want to specify some extra behavior here
        pass


def connect_instr():
    """ select instrument from list or manual entry and connect """

    conn = False
    addr = ''
    p = scpi.visa_connection()

    print("")
    print("************** INSTRUMENT LIST **************")
    resources = scpi.list_resources()                                       # find list of potential instruments
    resources = list(resources)
    resources.insert(0, 'MANUAL ENTRY')                                     # pre-pend "MANUAL ENTRY" to the list - used to type in a socket address

    for y in range(len(resources)):                                         # list all the resources
        print("    %3d) %s" % (y, resources[y]))
    print("*********************************************")
    print("")

    done = False
    while not done:
        sel = six.moves.input("Select instrument: ")
        print(sel)

        try:
            sel_num = int(sel)                                              # try to convert selection to number
        except ValueError:
            sel_num = -1                                                    # non-integer input

        if sel_num in range(len(resources)):                                # valid selection
            if sel_num == 0:
                addr = str(six.moves.input("Enter equipment VISA address: "))
                conn = p.open_resource(addr)
                done = True
            else:
                addr = resources[sel_num]                                   # set the address of the instrument
                conn = p.open_resource(addr)                                # open new instrument port
                conn = True
                done = True

        if not done:
            print("Invalid selection. Please try again\n")

    print ("")
    print("Measurement instrument selected: %s" % addr)
    print ("")

    return p, addr, conn


print("")
print("****************** MilliBox VISA Shell Debugger ******************")
print(" This program allows connection to an instrument and interactive")
print(" debugging of the SCPI commands using basic write/read/query.")
print("")

while cmd not in ["Y", "N"]:
    cmd = six.moves.input("Save session to '%s'? [Y/N] " % logfile)
    cmd = cmd.upper()
    print("")

if cmd == "Y":
    # add header to log file
    f = open(logfile, 'a', buffering=1)
    f.write("\n\n\n\n")
    f.write("*****************************************************************************\n")
    f.write("*********     MilliBox VISA Shell Session - %s      ********\n" % time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
    f.write("*****************************************************************************\n")
    f.write("\n\n\n")
    f.close()

    # send output to stdout and append to logfile visa_log.txt
    writer = Logger(sys.stdout, "visa_log.txt")
    sys.stdout = writer

while cmd != "X":
    if not connected:
        cmd = six.moves.input("NOT CONNECTED :: <C>onnect, E<X>it --> ")
        cmd = cmd.upper()
        print(cmd)

        if cmd == "C":
            (port, visa_add, connected) = connect_instr()

    else:
        cmd = six.moves.input("CONNECTED to %s :: <W>rite, <Q>uery, <R>ead, <T>imeout, <C>onnect, E<X>it --> " % visa_add)
        cmd = cmd.upper()
        print(cmd)

        if cmd == "W":
            x = six.moves.input("Write --> ")
            print(x)
            port.write(x)
            print("")

        if cmd == "Q":
            x = six.moves.input("Query --> ")
            print(x)
            try:
                print("Reply --> '%s'" % port.query(x))
            except:
                print("\n!! ERROR in Query. Please check command. !!\n")

        if cmd == "R":
            try:
                print("Read --> '%s'" % port.read())
            except:
                print("\n!! ERROR in Read. Please check command. !!\n")

        if cmd == "T":
            x = six.moves.input("Timeout (ms) [current = %g] --> " % (port.get_timeout()*1000.0))
            print(x)
            port.set_timeout(float(x)/1000.0)
            print("")

        if cmd == "C":
            (port, visa_add, connected) = connect_instr()
