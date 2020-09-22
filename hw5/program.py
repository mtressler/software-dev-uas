#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import random
from math import radians, sin, cos, atan2, sqrt
from geopy import distance
from dronekit_sitl import SITL
from dronekit import Vehicle, VehicleMode, connect, LocationGlobalRelative

def arm_and_takeoff(aTargetAltitude, vehicle):
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
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def hide_black_box():

    vertical_range = top - bottom
    horizontal_range = left - right

    # create latitude
    rand_num = random.random()
    vertical_offset = rand_num * vertical_range
    new_latitude = (bottom+vertical_offset)/1000000
    new_latitude = '%.6f'%(new_latitude)

    # create longitude
    rand_num = random.random()
    horizontal_offset = rand_num * horizontal_range
    new_longitude = (left - horizontal_offset)/1000000
    new_longitude = '%.6f'%(new_longitude)
    print('Black Box Hidden')
    return new_latitude, new_longitude

def get_distance(loc1, loc2):
    lat1 = radians(loc1.lat)
    lon1 = radians(loc1.lon)
    lat2 = radians(loc2.lat)
    lon2 = radians(loc2.lon)

    dlon = radians(lon2) - radians(lon1)
    dlat = lat2-lat1

    a = sin(dlat / 2)**2 + cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = a * c * 1000

    return distance

def search(curr_pos, vehicle, direction, index, borders):
    curr_lat = curr_pos.lat
    curr_long = curr_pos.lon

    curr_dir = direction[index%4]
    print(curr_dir)

    if curr_dir == 'up':
        if borders['top'] - curr_lat < .00014:
            borders['top'] = borders['top'] - .00005
            vehicle.simple_goto(LocationGlobalRelative(borders['top'], curr_long, 10), groundspeed=5)
            target_location = LocationGlobalRelative(borders['top'], curr_long, 10)
            target_distance = get_distance(vehicle.location.global_frame, target_location)
            index += 1
        else:
            vehicle.simple_goto(LocationGlobalRelative(curr_lat + .00009, curr_long, 10), groundspeed=5)
            target_location = LocationGlobalRelative(curr_lat + .00009, curr_long, 10)
            target_distance = get_distance(vehicle.location.global_frame, target_location)
    elif curr_dir == 'left':
        if borders['left'] - curr_long < .00017:
            borders['left'] = borders['left'] - .00005
            vehicle.simple_goto(LocationGlobalRelative(curr_lat, borders['left'], 10), groundspeed=5)
            target_location = LocationGlobalRelative(curr_lat, borders['left'], 10)
            target_distance = get_distance(vehicle.location.global_frame, target_location)
            index += 1
        else:
            vehicle.simple_goto(LocationGlobalRelative(curr_lat, curr_long + .00012, 10), groundspeed=5)
            target_location = LocationGlobalRelative(curr_lat, curr_long + .00012, 10)
            target_distance = get_distance(vehicle.location.global_frame, target_location)
    elif curr_dir == 'down':
        if curr_lat - borders['bottom'] < .00014:
            borders['bottom'] = borders['bottom'] + .00005
            vehicle.simple_goto(LocationGlobalRelative(borders['bottom'], curr_long, 10), groundspeed=5)
            target_location = (LocationGlobalRelative(borders['bottom'], curr_long, 10))
            target_distance = get_distance(vehicle.location.global_frame, target_location)
            index += 1
        else:
            vehicle.simple_goto(LocationGlobalRelative(curr_lat - .00009, curr_long, 10), groundspeed=5)
            target_location = LocationGlobalRelative(curr_lat - .00009, curr_long, 10)
            target_distance = get_distance(vehicle.location.global_frame, target_location)
    else:
        if curr_long - borders['right'] < .00017:
            borders['right'] = borders['right'] + .00005
            vehicle.simple_goto(LocationGlobalRelative(curr_lat, borders['right'], 10), groundspeed=5)
            target_location = LocationGlobalRelative(curr_lat, borders['right'], 10)
            target_distance = get_distance(vehicle.location.global_frame, target_location)
            index += 1
        else:
            vehicle.simple_goto(LocationGlobalRelative(curr_lat, curr_long - .00012, 10), groundspeed=5)
            target_location = LocationGlobalRelative(curr_lat, curr_long - .00012, 10)
            target_distance = get_distance(vehicle.location.global_frame, target_location)

    while get_distance(vehicle.location.global_frame, target_location) > target_distance*.01:
        continue

    return index, borders, vehicle

def ping(curr_pos, black_box_pos):
    dist_from_target = distance.distance((curr_pos.lat, curr_pos.lon), black_box_pos).meters
    if dist_from_target < 5:
        return black_box_pos
    return (0,0)

def main():
    global right, left, top, bottom
    
    direction = ['up', 'left', 'down', 'right']
    index = 0

    borders = { 'right' : -86.240335, 
                'left'  : -86.243146,
                'top'   :  41.715167,
                'bottom':  41.714350
    }

    #coordinates
    right  = -86240335
    left   = -86243146
    top    =  41715167
    bottom =  41714350 

    box_pos = hide_black_box()

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
        sitl_args = ['-I0', '--model', 'quad', '--home=41.714841, -86.241941,0,180']
        sitl.launch(sitl_args, await_ready=True, restart=True)
        connection_string = 'tcp:127.0.0.1:5760'   

    '''
('41.714639', '-86.240467')

    '''

    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True, baud=57600)
    print ('Current position of vehicle is: %s' % vehicle.location.global_frame)

    arm_and_takeoff(10, vehicle)

    print(box_pos)

    time.sleep(10)

    point1 = LocationGlobalRelative(41.714350, -86.240335, 10)

    vehicle.simple_goto(point1, groundspeed=5)

    print('Vehicle location: %s' % vehicle.location.global_frame)
    ping_result = ping(vehicle.location.global_frame, box_pos)

    while ping_result == (0,0):
        index, borders, vehicle = search(vehicle.location.global_frame, vehicle, direction, index, borders)
        print('Vehicle location: %s' % vehicle.location.global_frame)
        ping_result = ping(vehicle.location.global_frame, box_pos)

    print(ping_result[0])
    print(ping_result[1])
    point1 = LocationGlobalRelative(float(ping_result[0]), float(ping_result[1], 10))
    vehicle.simple_goto(point1, groundspeed=5)

    print('here')

    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

    # Shut down simulator if it was started.
    if sitl:
        sitl.stop()

if __name__ == "__main__":
    main()