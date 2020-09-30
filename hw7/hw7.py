#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)
Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.
Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit_sitl import SITL
import math
import matplotlib.pyplot as plt
from geopy import distance
from dronekit import Vehicle, VehicleMode, connect, LocationGlobalRelative


import json
from websocket import create_connection
from drone_model import Drone_Model
ws = create_connection("ws://localhost:8000")

drone_model_object =  Drone_Model(1,0,0)

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
# This technique for starting SITL allows us to specify defffaults 
if not connection_string:
    sitl_defaults = '~/git/ardupilot/tools/autotest/default_params/copter.parm'
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    sitl_args = ['-I0', '--model', 'quad', '--home=41.714469, -86.241786,0,180']
    sitl.launch(sitl_args, await_ready=True, restart=True)
    connection_string = 'tcp:127.0.0.1:5760'

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)
print ('Current position of vehicle is: %s' % vehicle.location.global_frame)


def custom_sleep(drone_model, sleep_time):
    current_time = 0
    while(current_time<sleep_time):
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        drone_model.update_status(lat,lon)
        ws.send(drone_model.toJSON())
        print('Current location is: {0},{1}'.format(lat,lon))
        time.sleep(1)
        current_time+=1

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(3)
        print("Arming motors")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    
    print("Vehicle armed!")
    print("Taking off!")
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    print('Current location before takeoff is: {0},{1}'.format(lat,lon))
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    print('Current location after takeoff is: {0},{1}'.format(lat,lon))

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        drone_model_object.update_status(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        ws.send(drone_model_object.toJSON())
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


class Location_Tracker:
    def __init__(self):

        self.x_cor = []
        self.y_cor = []

    def plot_journey(self):
        plt.plot(self.x_cor, self.y_cor, 'bo')
        #plt.xlim(41.714329, 41.714529)
        #plt.ylim(-86.242185, -86.241985)
        plt.show()

    def add_xcor(self, x):
        self.x_cor.append(x)

    def add_ycor(self, y):
        self.y_cor.append(y)
        print(self.x_cor)
        print(self.y_cor)


def goToPoint(vehicle, point, loc):
    pos = vehicle.location.global_relative_frame
    vehicle.simple_goto(point)
    dist = distance.distance((pos.lat, pos.lon), (point.lat, point.lon)).meters
    alt_dist = abs(point.alt - pos.alt)
    while dist > 1 and vehicle.mode.name == "GUIDED" and alt_dist > .5:
        print("Distance %f" % dist)
        print(pos)
        time.sleep(.3)
        pos = vehicle.location.global_relative_frame
        alt_dist = abs(point.alt - pos.alt)
        dist = distance.distance((pos.lat, pos.lon), (point.lat, point.lon)).meters
        loc.add_xcor(pos.lon)
        loc.add_ycor(pos.lat)

def goInDirection(vehicle, direction, d, loc):
    pos = vehicle.location.global_relative_frame
    if direction == "NW":
        x = pos.lat + .01
        z = pos.lon - .01
        target_point = LocationGlobalRelative(x, z, 10)
        vehicle.simple_goto(target_point)
        cur_pos = vehicle.location.global_relative_frame
        dist = distance.distance((pos.lat, pos.lon), (cur_pos.lat, cur_pos.lon)).meters
        while dist < d and vehicle.mode.name == "GUIDED":
            print(cur_pos)
            time.sleep(.3)
            cur_pos = vehicle.location.global_relative_frame
            dist = distance.distance((pos.lat, pos.lon), (cur_pos.lat, cur_pos.lon)).meters
            loc.add_xcor(cur_pos.lon)
            loc.add_ycor(cur_pos.lat)
    if direction == "E":
        x = pos.lon + .01
        target_point = LocationGlobalRelative(pos.lat, x, 15)
        vehicle.simple_goto(target_point)
        cur_pos = vehicle.location.global_relative_frame
        dist = distance.distance((pos.lat, pos.lon), (cur_pos.lat, cur_pos.lon)).meters
        while dist < d and vehicle.mode.name == "GUIDED":
            time.sleep(.3)
            print(cur_pos)
            cur_pos = vehicle.location.global_relative_frame
            dist = distance.distance((pos.lat, pos.lon), (cur_pos.lat, cur_pos.lon)).meters
            loc.add_xcor(cur_pos.lon)
            loc.add_ycor(cur_pos.lat)

def changeAlt(vehicle, altitude_target):
    cur_location = vehicle.location.global_relative_frame
    target_point = LocationGlobalRelative(cur_location.lat, cur_location.lon, altitude_target)
    vehicle.simple_goto(target_point)
    while vehicle.mode.name == "GUIDED":
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= altitude_target * 0.95:
            print("Reached target altitude")
            break
        time.sleep(.5)

loc = Location_Tracker()

# Take off to 10 meters
arm_and_takeoff(10)

starting_pos = vehicle.location.global_relative_frame

# Hover for 5 seconds
print("Hover for 5 seconds")
time.sleep(5)

# Fly North West for 20 meters
print("Fly North West for 20 meters")
goInDirection(vehicle, "NW", 20, loc)

# Increase altitude to 15 meters
print("Increase altitude to 15 meters")
changeAlt(vehicle, 15)

# Fly East for 20 meters
print("Fly East for 20 meters")
goInDirection(vehicle, "E", 20, loc)

# Fly back to lat and lon with final altitude of 10 meters
print("Fly back to lat and lon with final altitude of 10 meters")
goToPoint(vehicle, LocationGlobalRelative(starting_pos.lat, starting_pos.lon, 10), loc)

#loc.plot_journey()

# Land
print("Returning to Launch")
vehicle.mode = VehicleMode("LAND")

time.sleep(30)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()











