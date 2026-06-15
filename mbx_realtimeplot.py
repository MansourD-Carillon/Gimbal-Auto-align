#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################################################################
# Copyright 2018-2024 MILLIWAVE SILICON SOLUTIONS, inc.
# Author: Chinh Doan, Jeanmarc Laurent - Milliwave Silicon Solutions

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


# IMPORTS
import numpy as np                                                              # matplotlib needs numpy fucntions
import matplotlib.pyplot as plt                                                 # matplotlib for 3D display
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import time
import six
import os
# from mbx_functions import *

from scipy.spatial import Delaunay                                              # libraries needed to save to STL
import surf2stl

# matplotlib specific plotting delays - different for Python2.x vs. Python3.x
if six.PY2:
    # print("Setting plt_pause = 1e-3")
    plt_pause = 1e-3                                                            # matplotlib: Py2.x - delay=1e-3
else:
    # print("Setting plt_pause = 2e-1")
    plt_pause = 2e-1                                                            # matplotlib: Py3.x - delay=2e-1


def display_xyplot(x, y, sTitle=""):
    """ basic xy rectangular plot """

    # plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    # plt.plot(x, y, color='0.6', marker='.', linestyle='-')                      # plot the curve in GRAY
    plt.plot(x, y, color='r', marker='.', linestyle='-')                        # plot the curve in RED
    plt.xlabel('Freq (GHz)')
    plt.ylabel('Power (dB)')
    if sTitle != "":
        plt.title(sTitle)

    plt.grid(True)                                                              # turn grid on
    plt.draw()                                                                  # draw the surface on figure 1
    plt.pause(plt_pause)                                                        # allow time for the drawing to show on screen
    time.sleep(0.01)

    print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")

    # plt.ioff()
    plt.show()                                                                  # closing the plot unblock the function and go to menu

    return


def display_1dplot(dir, Vang, Hang, data, vert, hori, plot_freq=0, block_final=True, pangle=None, blocking=None):
    """ single line plot with last plot iteration blocking - HV gimbal """

    plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    curVIdx = max(np.where(Vang == vert)[0])                                    # find the index for the current vertical angle
    curHIdx = max(np.where(Hang == hori)[0])                                    # find the index for the current horizontal angle

    Z = np.array(data)                                                          # parse captured array

    if dir == "H":
        plt.plot(Hang,Z,color='0.6',marker='.',linestyle='-')                   # plot the curve in GRAY
        plt.xlabel('Horizontal angle')
        if pangle is None:
            if plot_freq == 0:
                plt.title("(H,V)=(%g,%g):   Power = %0.2f dBm" % (hori, vert, Z[curHIdx]))                              # display current point in title
            else:
                plt.title("%0.2fGHz\n(H,V)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, hori, vert, Z[curHIdx]))     # display current point in title
        else:
            if plot_freq == 0:
                plt.title("Pol=%g\n(H,V)=(%g,%g):   Power = %0.2f dBm" % (pangle, hori, vert, Z[curHIdx]))              # display current point in title
            else:
                plt.title("%0.2fGHz -- Pol=%g\n(H,V)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, pangle, hori, vert, Z[curHIdx]))   # display current point in title

    elif dir == "V":
        plt.plot(Vang,Z,color='0.6',marker='.',linestyle='-')                   # plot the curve in GRAY
        plt.xlabel('Vertical angle')
        if pangle is None:
            if plot_freq == 0:
                plt.title("(H,V)=(%g,%g):   Power = %0.2f dBm" % (hori, vert, Z[curVIdx]))                              # display current point in title
            else:
                plt.title("%0.2fGHz\n(H,V)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, hori, vert, Z[curVIdx]))     # display current point in title
        else:
            if plot_freq == 0:
                plt.title("Pol=%g\n(H,V)=(%g,%g):   Power = %0.2f dBm" % (pangle, hori, vert, Z[curVIdx]))              # display current point in title
            else:
                plt.title("%0.2fGHz -- Pol=%g\n(H,V)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, pangle, hori, vert, Z[curVIdx]))   # display current point in title

    plt.ylabel("Power (dB)")
    plt.grid(True)                                                              # turn grid on
    plt.draw()                                                                  # draw the surface on figure 1
    plt.pause(plt_pause)                                                        # allow time for the drawing to show on screen
    time.sleep(0.01)

    if blocking is None:
        intermediate = vert < Vang[-1] or hori < Hang[-1]
    else:
        intermediate = not blocking

    if intermediate:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        hmaxidx = 0
        vmaxidx = 0
        maxidx = np.where(Z == np.amax(Z))[0][0]                                # find angle for peak power
        if dir == "H":
            hmaxidx = maxidx
            plt.plot(Hang[hmaxidx], Z[hmaxidx], 'ro')                           # plot the location of the peak
        elif dir == "V":
            vmaxidx = maxidx
            plt.plot(Vang[vmaxidx], Z[vmaxidx], 'ro')                           # plot the location of the peak

        if pangle is None:
            if plot_freq == 0:
                if dir == 'H' or dir == 'V':
                    plt.title("Peak @ (H,V)=(%g,%g):   Power = %0.2f dBm" % (Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))       # print the peak power and location
                else:
                    plt.title("Peak @ (TH,PH)=(%g,%g):   Power = %0.2f dBm" % (Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))     # print the peak power and location
            else:
                if dir == 'H' or dir == 'V':
                    plt.title("%0.2fGHz\nPeak @ (H,V)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))      # print the peak power and location
                else:
                    plt.title("%0.2fGHz\nPeak @ (TH,PH)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))    # print the peak power and location
        else:
            if plot_freq == 0:
                if dir == 'H' or dir == 'V':
                    plt.title("Pol=%g\nPeak @ (H,V)=(%g,%g):   Power = %0.2f dBm" % (pangle,Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))            # print the peak power and location
                else:
                    plt.title("Delta_Phi=%g\nPeak @ (TH,PH)=(%g,%g):   Power = %0.2f dBm" % (pangle,Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))    # print the peak power and location
            else:
                if dir == 'H' or dir == 'V':
                    plt.title("%0.2fGHz -- Pol=%g\nPeak @ (H,V)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, pangle, Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))            # print the peak power and location
                else:
                    plt.title("%0.2fGHz -- Delta_Phi=%g\nPeak @ (TH,PH)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, pangle, Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))    # print the peak power and location

        if block_final:
            print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")

            plt.ioff()
            plt.show()                                                              # closing the plot unblock the function and go to menu

    return


def display_1dplot_sph(dir, PHang, THang, data, phi, theta, plot_freq=0, block_final=True, dphi=None, blocking=None):
    """ single line plot with last plot iteration blocking - SPHERICAL gimbal """

    plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    curPHIdx = max(np.where(PHang == phi)[0])                                   # find the index for the current PHI angle
    curTHIdx = max(np.where(THang == theta)[0])                                 # find the index for the current THETA angle

    Z = np.array(data)                                                          # parse captured array

    if dir == "T":
        plt.plot(THang,Z,color='0.6',marker='.',linestyle='-')                  # plot the curve in GRAY
        plt.xlabel('Theta angle')
        if dphi is None or np.isnan(dphi):
            if plot_freq == 0:
                plt.title("(TH,PH)=(%g,%g):   Power = %0.2f dBm" % (theta, phi, Z[curTHIdx]))                           # display current point in title
            else:
                plt.title("%0.2fGHz\n(TH,PH)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, theta, phi, Z[curTHIdx]))  # display current point in title
        else:
            if plot_freq == 0:
                plt.title("Delta_Phi=%g\n(TH,PH)=(%g,%g):   Power = %0.2f dBm" % (dphi, theta, phi, Z[curTHIdx]))                               # display current point in title
            else:
                plt.title("%0.2fGHz -- Delta_Phi=%g\n(TH,PH)=(%g,%g):   Power = %0.2f dBm" % (plot_freq / 1e9, dphi, theta, phi, Z[curTHIdx]))  # display current point in title
    elif dir == "P":
        plt.plot(PHang,Z,color='0.6',marker='.',linestyle='-')                  # plot the curve in GRAY
        plt.xlabel('Phi angle')
        if dphi is None or np.isnan(dphi):
            if plot_freq == 0:
                plt.title("(TH,PH)=(%g,%g):   Power = %0.2f dBm" % (theta, phi, Z[curPHIdx]))                           # display current point in title
            else:
                plt.title("%0.2fGHz\n(TH,PH)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, theta, phi, Z[curPHIdx]))  # display current point in title
        else:
            if plot_freq == 0:
                plt.title("Delta_Phi=%g\n(TH,PH)=(%g,%g):   Power = %0.2f dBm" % (dphi, theta, phi, Z[curPHIdx]))                               # display current point in title
            else:
                plt.title("%0.2fGHz -- Delta_Phi=%g\n(TH,PH)=(%g,%g):   Power = %0.2f dBm" % (plot_freq / 1e9, dphi, theta, phi, Z[curPHIdx]))  # display current point in title

    plt.ylabel("Power (dB)")
    plt.grid(True)                                                              # turn grid on
    plt.draw()                                                                  # draw the surface on figure 1
    plt.pause(plt_pause)                                                        # allow time for the drawing to show on screen
    time.sleep(0.01)

    if blocking is None:
        intermediate = phi < PHang[-1] or theta < THang[-1]
    else:
        intermediate = not blocking

    if intermediate:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        thmaxidx = 0
        phmaxidx = 0
        maxidx = np.where(Z == np.amax(Z))[0][0]                                # find angle for peak power
        if dir == "T":
            thmaxidx = maxidx
            plt.plot(THang[thmaxidx], Z[thmaxidx], 'ro')                        # plot the location of the peak
        elif dir == "P":
            phmaxidx = maxidx
            plt.plot(PHang[phmaxidx], Z[phmaxidx], 'ro')                        # plot the location of the peak

        if dphi is None or np.isnan(dphi):
            if plot_freq == 0:
                plt.title("Peak @ (TH,PH)=(%g,%g):   Power = %0.2f dBm" % (THang[thmaxidx],PHang[phmaxidx],np.amax(Z)))  # print the peak power and location
            else:
                plt.title("%0.2fGHz\nPeak @ (TH,PH)=(%g,%g):   Power = %0.2f dBm" % (plot_freq/1e9, THang[thmaxidx],PHang[phmaxidx],np.amax(Z)))  # print the peak power and location
        else:
            if plot_freq == 0:
                plt.title("Delta_Phi=%g\nPeak @ (TH,PH)=(%g,%g):   Power = %0.2f dBm" % (dphi, THang[thmaxidx], PHang[phmaxidx], np.amax(Z)))  # print the peak power and location
            else:
                plt.title("%0.2fGHz -- Delta_Phi=%g\nPeak @ (TH,PH)=(%g,%g):   Power = %0.2f dBm" % (plot_freq / 1e9, dphi, THang[thmaxidx], PHang[phmaxidx], np.amax(Z)))  # print the peak power and location

        if block_final:
            print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")

            plt.ioff()
            plt.show()                                                              # closing the plot unblock the function and go to menu

    return


def display_hvplot(Vang, Hang, dataV, dataH, blocking, plot_freq=0, block_final=True, pangle=None, legend=None, query_scale=False):
    """ line plot for E- and H-plane with last plot iteration blocking - HV gimbal """

    if legend is None:
        legend = ['H sweep', 'V sweep']
    plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    ZV = np.array(dataV)                                                        # parse captured array
    ZH = np.array(dataH)                                                        # parse captured array

    plt.plot(Hang,ZH,color='0.6',marker='.',linestyle='-')                      # plot H sweep in GRAY
    plt.plot(Vang,ZV,color='r',marker='.',linestyle='-')                        # plot V sweep in RED
    plt.xlabel('angle [deg]')
    plt.legend(legend)
    if pangle is None:
        if plot_freq != 0:
            plt.title("%0.2fGHz" % (plot_freq/1e9))
    else:
        if plot_freq != 0:
            plt.title("%0.2fGHz -- Pol=%g" % (plot_freq/1e9,pangle))
        else:
            plt.title("Pol=%g" % (pangle))

    plt.ylabel("Power (dB)")
    plt.grid(True)                                                              # turn on grid
    plt.draw()                                                                  # draw the surface on figure 1
    plt.pause(plt_pause)                                                        # allow time for the drawing to show on screen
    time.sleep(0.01)

    # if query_scale:
    #     print("Specify X or Y scale? [Y/N]")                                    # option to override autoscale
    #     set_scale = 0
    #     while chr(set_scale) not in ['y', 'n']:
    #         set_scale = ord(getch().lower())
    #     if set_scale == ord('y'):
    #
    #         xlim_min = #12345
    #
    # plt.axis([None, None, -50, 5])
    # if query_scale:
    #     #12345
    #     plt.axis

    if not blocking:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        if block_final:
            print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")

            plt.ioff()
            plt.show()                                                          # closing the plot unblock the function and go to menu

    return


def display_hvplot_sph(PHang, THang, dataPH00, dataPH90, blocking, plot_freq=0, block_final=True, dphi=None):
    """ line plot for E- and H-plane with last plot iteration blocking - SPHERICAL gimbal """

    plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    ZPH00 = np.array(dataPH00)                                                  # parse captured array
    ZPH90 = np.array(dataPH90)                                                  # parse captured array

    plt.plot(THang,ZPH00,color='0.6',marker='.',linestyle='-')                  # plot PHI=0 sweep in GRAY
    plt.plot(THang,ZPH90,color='r',marker='.',linestyle='-')                    # plot PHI=90 sweep in RED
    plt.xlabel('Theta angle [deg]')
    plt.legend(['Phi=0 (azimuth)','Phi=90 (elevation)'])
    if dphi is None or np.isnan(dphi):
        if plot_freq != 0:
            plt.title("%0.2fGHz" % (plot_freq/1e9))
    else:
        if plot_freq != 0:
            plt.title("%0.2fGHz -- Delta_Phi=%g" % (plot_freq/1e9,dphi))
        else:
            plt.title("Delta_Phi=%g" % (dphi))

    plt.ylabel("Power (dB)")
    plt.grid(True)                                                              # turn on grid
    plt.draw()                                                                  # draw the surface on figure 1
    plt.pause(plt_pause)                                                        # allow time for the drawing to show on screen
    time.sleep(0.01)

    if not blocking:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        if block_final:
            print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")

            plt.ioff()
            plt.show()                                                              # closing the plot unblock the function and go to menu

    return


def display_surfplot(Vang, Hang, data, vert, hori, plot_freq=0, pangle=None, blocking=None):
    """ 3D surface plot with last plot iteration blocking - HV gimbal """

    plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    ax = plt.axes(projection='3d')                                              # 3D projection
    ax.set_xlabel('Vertical angle')                                             # label X axis as Vertical
    ax.set_ylabel('Horizontal angle')                                           # label Y axis as Horizontal
    ax.set_zlabel("Power (dB)")                                                 # label z axis as captured data
    X, Y = np.meshgrid(Vang , Hang)                                             # define X Y grid
    # print("array data is: " +str(data))                                       # for debug print captured array
    zs = np.array(data)                                                         # parse captured array
    # print("Shape X is" +str(X.shape))                                         # for debug get X Y shape
    Z = zs.reshape(X.shape)                                                     # reshape the array to X Y shape
    Z[np.isnan(Z)] = np.nanmin(Z)                                               # set all NaN values to lowest value (ignoring NaN)
    ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.jet, linewidth=0, antialiased=False, alpha=0.5)
    if pangle is None:
        if plot_freq != 0:
            plt.title("%0.2fGHz" % (plot_freq/1e9))
    else:
        if plot_freq != 0:
            plt.title("%0.2fGHz -- Pol=%g" % (plot_freq/1e9,pangle))
        else:
            plt.title("Pol=%g" % pangle)

    plt.draw()                                                                  # draw the surface on figure 1
    plt.pause(plt_pause)                                                        # allow time for the drawing to show on screen
    time.sleep(0.01)

    if blocking is None:
        intermediate = vert < Vang[-1] or hori < Hang[-1]
    else:
        intermediate = not blocking

    if intermediate:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
        plt.ioff()
        plt.show()                                                              # closing the plot unblock the function and go to menu
    return


def display_heatmap(Vang, Hang, data, vert, hori, plot_freq=0, pangle=None, blocking=None):
    """ 2D heatmap plot with last plot iteration blocking - HV gimbal """

    plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    ax = plt.axes()
    plt.xlabel('Horizontal angle')                                              # label X axis as Horizontal
    plt.ylabel("Vertical angle")                                                # label Y axis as Vertical

    X, Y = np.meshgrid(Vang , Hang)                                             # define X Y grid
    # print("array data is: " +str(data))                                       # for debug print captured array
    zs = np.array(data)                                                         # parse captured array
    # print("Shape X is" +str(X.shape))                                         # for debug get X Y shape
    Z = zs.reshape(X.shape)                                                     # reshape the array to X Y shape
    Z[np.isnan(Z)] = np.nanmin(Z)                                               # set all NaN values to lowest value (ignoring NaN)

    plt.rcParams['contour.negative_linestyle'] = 'solid'                        # set negative value contours to solid lines
    ax.contourf(Y, X, Z, 10, cmap=cm.jet)                                       # filled contours, swap X/Y to align with Hang/Vang
    ax.contour(Y, X, Z, 10, colors='gray', linewidths=0.5)                      # contour lines, swap X/Y to align with Hang/Vang
    sm = plt.cm.ScalarMappable(cmap="jet", norm=plt.Normalize(vmin=Z.min(), vmax=Z.max()))
    sm.set_array([])
    plt.colorbar(sm, ax=ax)                                                     # show jet colorbar
    plt.grid(visible=True, which='major', color='0.2', linestyle=':')           # plot major gridlines - gray dotted lines

    if pangle is None:
        if plot_freq != 0:
            plt.title("%0.2fGHz" % (plot_freq/1e9))
    else:
        if plot_freq != 0:
            plt.title("%0.2fGHz -- Pol=%g" % (plot_freq/1e9,pangle))
        else:
            plt.title("Pol=%g" % pangle)

    plt.draw()                                                                  # draw the surface on figure 1
    plt.pause(plt_pause)                                                        # allow time for the drawing to show on screen
    time.sleep(0.01)

    if blocking is None:
        intermediate = vert < Vang[-1] or hori < Hang[-1]
    else:
        intermediate = not blocking

    if intermediate:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
        plt.ioff()
        plt.show()                                                              # closing the plot unblock the function and go to menu
    return


def display_multilineplot(Vang, Hang, data, vert, hori, plot_freq=0, pangle=None, blocking=None):
    """ multiple line plot with last plot iteration blocking - HV gimbal """

    plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    curVIdx = max(np.where(Vang == vert)[0])                                    # find the index for the current vertical angle
    curHIdx = max(np.where(Hang == hori)[0])                                    # find the index for the current horizontal angle

    X = Hang                                                                    # plot individual slices vs. H angle
    zs = np.array(data)                                                         # parse captured array
    Z = zs.reshape(len(Hang),len(Vang))                                         # reshape the array to X Y shape

    plt.clf()                                                                   # clear current figure; replot from scratch
    plt.plot(X,Z,color='0.6',marker='.',linestyle='-')                          # plot all of the curves in GRAY
    plt.xlabel('Horizontal angle')
    plt.ylabel("Power (dB)")
    if pangle is None:
        if plot_freq == 0:
            plt.title("(H,V)=(%g,%g):   Power = %0.2fdBm" % (hori,vert,Z[curHIdx][curVIdx]))                            # display current point in title
        else:
            plt.title("%0.2fGHz\n(H,V)=(%g,%g):   Power = %0.2fdBm" % (plot_freq/1e9,hori,vert,Z[curHIdx][curVIdx]))    # display current point in title
    else:
        if plot_freq == 0:
            plt.title("Pol=%g\n(H,V)=(%g,%g):   Power = %0.2fdBm" % (pangle,hori,vert,Z[curHIdx][curVIdx]))             # display current point in title
        else:
            plt.title("%0.2fGHz -- Pol=%g\n(H,V)=(%g,%g):   Power = %0.2fdBm" % (plot_freq/1e9,pangle,hori,vert,Z[curHIdx][curVIdx]))  # display current point in title
    plt.grid(True)                                                              # turn on grid

    if blocking is None:
        intermediate = vert < Vang[-1] or hori < Hang[-1]
    else:
        intermediate = not blocking

    if intermediate:                                                            # intermediate plot
        plt.plot(X, Z[:, curVIdx], 'r.-',linewidth=3)                           # plot the current curve in RED
        plt.draw()                                                              # draw the plot
        plt.pause(plt_pause)                                                    # allow time for the drawing to show on screen
        time.sleep(0.01)
        plt.show()

    else:                                                                       # last plot becomes blocking
        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")

        hmaxidx = np.where(Z == np.amax(Z))[0][0]                               # find H angle for peak power
        vmaxidx = np.where(Z == np.amax(Z))[1][0]                               # find V angle for peak power
        if pangle is None:
            if plot_freq == 0:
                plt.title("Peak @ (H,V)=(%g,%g):   Power = %0.2fdBm" % (Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))  # print the peak power and location
            else:
                plt.title("%0.2fGHz\nPeak @ (H,V)=(%g,%g):   Power = %0.2fdBm" % (plot_freq/1e9,Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))  # print the peak power and location
        else:
            if plot_freq == 0:
                plt.title("Pol=%g\nPeak @ (H,V,P)=(%g,%g):   Power = %0.2fdBm" % (pangle,Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))  # print the peak power and location
            else:
                plt.title("%0.2fGHz -- Pol=%g\nPeak @ (H,V)=(%g,%g):   Power = %0.2fdBm" % (plot_freq/1e9,pangle,Hang[hmaxidx],Vang[vmaxidx],np.amax(Z)))  # print the peak power and location
        plt.plot(Hang[hmaxidx],Z[hmaxidx][vmaxidx],'ro')                        # plot the location of the peak
        plt.draw()                                                              # draw the plot
        plt.pause(plt_pause)                                                    # allow time for the drawing to show on screen
        time.sleep(0.01)

        plt.ioff()
        plt.show()                                                              # closing the plot unblock the function and go to menu

    return


def display_dir_cosine_sph(PHang, THang, data, phi, theta, plot_freq=0, dphi=None, blocking=None):
    """ 2D direction cosine heatmap plot with last plot iteration blocking - SPHERICAL gimbal """

    plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    ax = plt.axes()
    ax.set_aspect('equal')
    plt.xlabel('u = sin(theta)*cos(phi)')                                       # label X axis
    plt.ylabel('v = sin(theta)*sin(phi)')                                       # label Y axis

    PHang_mesh, THang_mesh = np.meshgrid(PHang, THang)                          # define X Y grid
    u = np.sin(THang_mesh*np.pi/180) * np.cos(PHang_mesh*np.pi/180)             # transform grid
    v = np.sin(THang_mesh*np.pi/180) * np.sin(PHang_mesh*np.pi/180)

    # print("array data is: " +str(data))                                       # for debug print captured array
    zs = np.array(data)                                                         # parse captured array
    # print("Shape X is" +str(X.shape))                                         # for debug get X Y shape
    Z = zs.reshape(PHang_mesh.shape)                                            # reshape the array to X Y shape
    Z[np.isnan(Z)] = np.nanmin(Z)                                               # set all NaN values to lowest value (ignoring NaN)

    plt.rcParams['contour.negative_linestyle'] = 'solid'                        # set negative value contours to solid lines
    ax.contourf(u, v, Z, 10, cmap=cm.jet)                                       # filled contours, swap X/Y to align with Hang/Vang
    ax.contour(u, v, Z, 10, colors='gray', linewidths=0.5)                      # contour lines, swap X/Y to align with Hang/Vang
    sm = plt.cm.ScalarMappable(cmap="jet", norm=plt.Normalize(vmin=Z.min(), vmax=Z.max()))
    sm.set_array([])
    plt.colorbar(sm, ax=ax)                                                     # show jet colorbar
    # plt.grid(visible=True, which='major', color='0.2', linestyle=':')           # plot major gridlines - gray dotted lines
    ax.plot(np.cos(np.linspace(0,2*np.pi,100)), np.sin(np.linspace(0,2*np.pi,100)), color='lightgray', linestyle='-')   # plot unit circle
    plt.axis([-1.1, 1.1, -1.1, 1.1])

    if dphi is None or np.isnan(dphi):
        if plot_freq != 0:
            plt.title("%0.2fGHz" % (plot_freq/1e9))
    else:
        if plot_freq != 0:
            plt.title("%0.2fGHz -- Delta_Phi=%g" % (plot_freq/1e9,dphi))
        else:
            plt.title("Delta_Phi=%g" % dphi)

    plt.draw()                                                                  # draw the surface on figure 1
    plt.pause(plt_pause)                                                        # allow time for the drawing to show on screen
    time.sleep(0.01)

    if blocking is None:
        intermediate = phi < PHang[-1] or theta < THang[-1]
    else:
        intermediate = not blocking

    if intermediate:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
        plt.ioff()
        plt.show()                                                              # closing the plot unblock the function and go to menu
    return


def display_polar_sph(PHang, THang, data, phi, theta, plot_freq=0, dphi=None, blocking=None):
    """ 2D polar spherical plot with last plot iteration blocking - SPHERICAL gimbal """

    plt.ion()                                                                   # turn on plot interactive, makes graph non blocking
    plt.figure(1)                                                               # plot in figure 1
    plt.clf()                                                                   # clear figure before plotting new one

    ax = plt.axes()
    ax.set_aspect('equal')
    plt.xlabel('Xg = theta*cos(phi)')                                           # label X axis
    plt.ylabel('Yg = theta*sin(phi)')                                           # label Y axis

    PHang_mesh, THang_mesh = np.meshgrid(PHang, THang)                          # define X Y grid
    Xg = THang_mesh * np.cos(PHang_mesh*np.pi/180)                              # transform grid
    Yg = THang_mesh * np.sin(PHang_mesh*np.pi/180)

    # print("array data is: " +str(data))                                       # for debug print captured array
    zs = np.array(data)                                                         # parse captured array
    # print("Shape X is" +str(X.shape))                                         # for debug get X Y shape
    Z = zs.reshape(PHang_mesh.shape)                                            # reshape the array to X Y shape
    Z[np.isnan(Z)] = np.nanmin(Z)                                               # set all NaN values to lowest value (ignoring NaN)

    plt.rcParams['contour.negative_linestyle'] = 'solid'                        # set negative value contours to solid lines
    ax.contourf(Xg, Yg, Z, 10, cmap=cm.jet)                                     # filled contours, swap X/Y to align with Hang/Vang
    ax.contour(Xg, Yg, Z, 10, colors='gray', linewidths=0.5)                    # contour lines, swap X/Y to align with Hang/Vang
    sm = plt.cm.ScalarMappable(cmap="jet", norm=plt.Normalize(vmin=Z.min(), vmax=Z.max()))
    sm.set_array([])
    plt.colorbar(sm, ax=ax)                                                     # show jet colorbar
    plt.grid(visible=True, which='major', color='0.2', linestyle=':')           # plot major gridlines - gray dotted lines

    if dphi is None or np.isnan(dphi):
        if plot_freq != 0:
            plt.title("%0.2fGHz" % (plot_freq/1e9))
    else:
        if plot_freq != 0:
            plt.title("%0.2fGHz -- Delta_Phi=%g" % (plot_freq/1e9,dphi))
        else:
            plt.title("Delta_Phi=%g" % dphi)

    plt.draw()                                                                  # draw the surface on figure 1
    plt.pause(plt_pause)                                                        # allow time for the drawing to show on screen
    time.sleep(0.01)

    if blocking is None:
        intermediate = phi < PHang[-1] or theta < THang[-1]
    else:
        intermediate = not blocking

    if intermediate:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")
        plt.ioff()
        plt.show()                                                              # closing the plot unblock the function and go to menu
    return


def ant_pattern_3d(gain, phi, theta, plot_range, step, plot_freq=0, pangle=None, boresight='x', stl_max_mm=None, stl_min_mm=None, outname=None):
    """ 3d radiation pattern based on SPHERICAL COORDINATES
    this function creates a 3D radiation pattern model and either plots to screen or
    generates a STL file for 3D printing. this function will plot all of the power
    with a total dynamic range of plot_range. the values that will plot are from
    [max(gain)-plot_range ... max(gain)] all other points are set to minimum """

    if stl_max_mm is not None:                                                  # save to STL file
        mag_scale = stl_max_mm / plot_range                                     # mag_max = mag_scale * plot_range = stl_max_mm
        if stl_min_mm is not None:
            plot_min = plot_range * stl_min_mm / stl_max_mm                     # mag_min = mag_scale * plot_min = stl_min_mm
        else:
            plot_min = 0.0
    else:                                                                       # plot to screen
        mag_scale = 1.0                                                         # mag_max = plot_range
        plot_min = 0.0                                                          # mag_min = 0

    mag_max = mag_scale * plot_range                                            # mag_max = dynamic_range to plot * mag_scale
    gain_max = np.ceil(np.nanmax(gain))                                         # determine the max(gain)
    m = mag_scale*(gain - gain_max + plot_range)                                # re-scale everything based on max(gain)
    m[m < mag_scale * plot_min] = mag_scale * plot_min                          # all small values forced to plot_min

    x = m * np.sin(theta) * np.cos(phi)                                         # convert (gain,phi,theta) to (x,y,z)
    y = m * np.sin(theta) * np.sin(phi)                                         #
    z = m * np.cos(theta)                                                       #

    if stl_max_mm is not None:                                                  # ====== save to STL file ======
        u = theta.flatten()                                                     # create vector of theta
        v = phi.flatten()                                                       # create vector of phi

        umin = np.min(u)                                                        # find min(theta)
        umax = np.max(u)                                                        # find max(theta)
        vmin = np.min(v)                                                        # find min(phi)
        vmax = np.max(v)                                                        # find max(phi)

        points = np.array([u,v]).T                                              # create list of (theta,phi) pairs
        # append 4 points to the list, 1 point beyond each edge of the (theta,phi) mesh
        points = np.append(points, [[umin-1,(vmax+vmin)/2], [umax+1,(vmax+vmin)/2], [(umax+umin)/2,vmin-1], [(umax+umin)/2,vmax+1]], axis=0)
        delaunay_tri = Delaunay(points)                                         # create the Delaunay mesh for the point cloud

        x2 = np.append(x.flatten(),[0, 0, 0, 0])                                # additional 4 points all come back to origin (0,0,0)
        y2 = np.append(y.flatten(),[0, 0, 0, 0])                                # this creates a closed shape instead of a hollow shell
        z2 = np.append(z.flatten(),[0, 0, 0, 0])                                #

        if outname is not None:
            surf2stl.tri_write(outname,x2,y2,z2,delaunay_tri)                   # save STL using Delaunay tri to connect the nearest nodes
            print("Saved STL to file: %s\n" % outname)
        else:
            print("*** ERROR: no STL output file name specified ***")

    else:                                                                       # ====== plot to screen ======
        plt.ion()                                                               # turn on plot interactive, makes graph non blocking
        plt.figure(1, figsize=(8, 6))                                           # plot in figure 1, make default size larger (8" x 6")
        plt.clf()                                                               # clear figure before plotting new one

        ax = plt.axes(projection='3d')                                          # 3D projection
        if six.PY2:
            ax.set_aspect('auto')                                               # Python 2 syntax for 3d equal aspect ratio
        elif six.PY3:
            ax.set_box_aspect((1, 1, 1))                                        # Python 3 syntax for 3d equal aspect ratio
        ax.margins(x=0, y=-0.25)                                                # reduce margins to make plot larger

        if boresight == 'x':                                                    # HV gimbal has boresight on X-axis
            yy, zz = np.meshgrid(np.linspace(-mag_max, mag_max, 3), np.linspace(-mag_max, mag_max, 3))      # mag_max is the maximum spherical magnitude
            xx = np.zeros(yy.size)
            xx = xx.reshape(yy.shape)
            ax.plot_wireframe(xx, yy, zz, color='black')                        # plot reference DUT on YZ-plane (x=0) with black wireframe
        elif boresight == 'z':                                                  # SPHERICAL gimbal has boresight on Z-axis
            xx, yy = np.meshgrid(np.linspace(-mag_max, mag_max, 3), np.linspace(-mag_max, mag_max, 3))      # mag_max is the maximum spherical magnitude
            zz = np.zeros(xx.size)
            zz = zz.reshape(xx.shape)
            ax.plot_wireframe(xx, yy, zz, color='black')                        # plot reference DUT on XY-plane (z=0) with black wireframe

        stride = int(max(1,6/int(step)))                                        # if step<4, skip some data points for faster plotting
        surf = ax.plot_surface(x,y,z,facecolors=cm.jet(m/mag_max),shade=False,rstride=stride, cstride=stride, antialiased=False,
                        linewidth=0.01, alpha=0.85)                             # plot semi-transparent with color proportional to gain
        surf.set_edgecolors('k')                                                # display black edges
        zoom = 1.3                                                              # zoom in on image for nice size
        ax.set_xlim([-mag_max/zoom,mag_max/zoom])
        ax.set_ylim([-mag_max/zoom,mag_max/zoom])
        ax.set_zlim([-mag_max/zoom,mag_max/zoom])

        if boresight == 'x':
            ax.view_init(elev=15, azim=-15)                                     # set default camera angle
        elif boresight == 'z':
            ax.view_init(elev=75, azim=-105)                                    # set default camera angle
        plt.axis('off')
        if pangle is None or np.isnan(pangle):
            if plot_freq == 0:
                ax.set_title('3D radiation pattern')
            else:
                ax.set_title('%0.2fGHz\n\n3D radiation pattern' % (plot_freq/1e9))
        else:
            if plot_freq == 0:
                if boresight == 'x':
                    ax.set_title('Pol=%g\n\n3D radiation pattern' % (pangle))
                else:
                    ax.set_title('Delta_Phi=%g\n\n3D radiation pattern' % (pangle))
            else:
                if boresight == 'x':
                    ax.set_title('%0.2fGHz -- Pol=%g\n\n3D radiation pattern' % (plot_freq/1e9,pangle))
                else:
                    ax.set_title('%0.2fGHz -- Delta_Phi=%g\n\n3D radiation pattern' % (plot_freq/1e9,pangle))

        sm = plt.cm.ScalarMappable(cmap="jet", norm=plt.Normalize(vmin=gain_max-plot_range, vmax=gain_max))     # set range of colorbar to dynamic range
        sm.set_array([])
        plt.colorbar(sm, ax=ax)                                                 # display colorbar

        plt.draw()                                                              # draw the plot
        plt.pause(plt_pause)                                                    # allow time for the drawing to show on screen
        time.sleep(0.01)
        plt.show()

    return


def print_millibox3d_ant_pattern(Vang, Hang, data, vert, hori, step, plot_freq=0, plot_range=40, stl_max_mm=200, stl_min_mm=25, outname=None):
    """ 3d radiation pattern STL print based on MILLIBOX COORDINATES
    print radiation pattern based on the gimbal (H,V)(deg) coordinates - dynamic range to plot defaults to 40dB
    NOTE:
        HV gimbal has boresight in X-axis direction
        H has same sign as phi
        V has same sign as theta but is offset by 90deg (i.e., V=0 -> theta=90) """

    if outname is None:
        outname = os.path.join('..', '..', 'MilliBox_plot_data', 'stl', 'radpat.stl')

    V, H = np.meshgrid(Vang, Hang)                                              # format the data vs. H and V
    # print("V is "  +str(V)+ "and H is " +str(H)  )
    gain = np.array(data)
    gain = gain.reshape(V.shape)                                                # reshape the data

    ant_pattern_3d(gain, H*np.pi/180, (90+V)*np.pi/180, plot_range, step, plot_freq, boresight='x', stl_max_mm=stl_max_mm, stl_min_mm=stl_min_mm, outname=outname)      # translate from (H,V)(deg) to (phi,theta)(rad)

    return


def display_millibox3d_ant_pattern(Vang, Hang, data, vert, hori, step, plot_freq=0, block_final=True, pangle=None, blocking=None, plot_range=50):
    """ 3d radiation pattern plot based on MILLIBOX COORDINATES
    plot radiation pattern based on the gimbal (H,V)(deg) coordinates - dynamic range to plot defaults to 50dB
    NOTE:
        HV gimbal has boresight in X-axis direction
        H has same sign as phi
        V has same sign as theta but is offset by 90deg (i.e., V=0 -> theta=90) """

    V, H = np.meshgrid(Vang, Hang)                                              # format the data vs. H and V
    # print("V is "  +str(V)+ "and H is " +str(H)  )
    gain = np.array(data)
    gain = gain.reshape(V.shape)                                                # reshape the data

    ant_pattern_3d(gain, H*np.pi/180, (90+V)*np.pi/180, plot_range, step, plot_freq, pangle, boresight='x')     # translate from (H,V)(deg) to (phi,theta)(rad)

    if blocking is None:
        intermediate = vert < Vang[-1] or hori < Hang[-1]
    else:
        intermediate = not blocking

    if intermediate:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        if block_final:
            print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")

            plt.ioff()
            plt.show()                                                          # closing the plot unblock the function and go to menu

    return


def print_millibox3d_ant_pattern_sph(PHang, THang, data, phi, theta, step, plot_freq=0, dphi=None, plot_range=40, stl_max_mm=200, stl_min_mm=25, outname=None):
    """ 3d radiation pattern STL print based on MILLIBOX COORDINATES
    print radiation pattern based on the gimbal (THETA,PHI)(deg) coordinates - dynamic range to plot defaults to 40dB
    NOTE:
        SPHERICAL gimbal has boresight in Z-axis direction
        PHI has same sign as phi
        THETA has same sign as theta """

    if outname is None:
        outname = os.path.join('..', '..', 'MilliBox_plot_data', 'stl', 'radpat.stl')

    PHang_mesh, THang_mesh = np.meshgrid(PHang, THang)                          # format the data vs. Theta and Phi
    # print("Phi is "  +str(PHang)+ "and Theta is " +str(THang)  )
    gain = np.array(data)
    gain = gain.reshape(PHang_mesh.shape)                                       # reshape the data

    ant_pattern_3d(gain, PHang_mesh*np.pi/180, THang_mesh*np.pi/180, plot_range, step, plot_freq, dphi, boresight='z', stl_max_mm=stl_max_mm, stl_min_mm=stl_min_mm, outname=outname)   # translate from (PHI,THETA)(deg) to (phi,theta)(rad)

    return


def display_millibox3d_ant_pattern_sph(PHang, THang, data, phi, theta, step, plot_freq=0, block_final=True, dphi=None, blocking=None, plot_range=50):
    """ 3d radiation pattern plot based on MILLIBOX COORDINATES
    plot radiation pattern based on the gimbal (THETA,PHI)(deg) coordinates - dynamic range to plot defaults to 50dB
    NOTE:
        SPHERICAL gimbal has boresight in Z-axis direction
        PHI has same sign as phi
        THETA has same sign as theta """

    PHang_mesh, THang_mesh = np.meshgrid(PHang, THang)                          # format the data vs. Theta and Phi
    # print("Phi is "  +str(PHang)+ "and Theta is " +str(THang)  )
    gain = np.array(data)
    gain = gain.reshape(PHang_mesh.shape)                                       # reshape the data

    ant_pattern_3d(gain, PHang_mesh*np.pi/180, THang_mesh*np.pi/180, plot_range, step, plot_freq, dphi, boresight='z')  # translate from (PHI,THETA)(deg) to (phi,theta)(rad)

    if blocking is None:
        intermediate = phi < PHang[-1] or theta < THang[-1]
    else:
        intermediate = not blocking

    if intermediate:                                                            # intermediate plot
        plt.show()

    else:                                                                       # last plot becomes blocking
        if block_final:
            print("-----------CLOSE PLOT GRAPHIC TO RETURN TO MENU-----------------")

            plt.ioff()
            plt.show()                                                          # closing the plot unblock the function and go to menu

    return
