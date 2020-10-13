#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
from geopy import distance
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dronekit_sitl import SITL
from dronekit import Vehicle, VehicleMode, connect, LocationGlobalRelative
from ned_utilities import ned_controller
from flight_plotter import Location

#responsible for connecting to drones, and having them get up to altitude
class Ground_Control:

    def __init__(self):
        self.copters = []
        self.sitls = []

        # Starting coordinates
        start_coordinates_1 = [41.71444,-86.2418,0]
        start_coordinates_2 = [41.71446,-86.2413,0]
        #start_coordinates_1 = [41.71445,-86.2418,0]
        #start_coordinates_2 = [41.71445,-86.2412,0]

        self.connect_virtual_vehicle(0,start_coordinates_1, 0)
        self.connect_virtual_vehicle(1,start_coordinates_2, 1)

        # Arm and takeoff to 10 meters
        self.arm_and_takeoff(10) 

    def connect_virtual_vehicle(self, instance, home, start_dir):
        sitl = SITL()
        sitl.download('copter', '3.3', verbose=True)
        instance_arg = '-I%s' %(str(instance))
        print("Drone instance is: %s" % instance_arg)
        home_arg = '--home=%s, %s,%s,180' % (str(home[0]), str(home[1]), str(home[2]))
        speedup_arg = '--speedup=1'
        sitl_args = [instance_arg, '--model', 'quad', home_arg, speedup_arg]
        sitl.launch(sitl_args, await_ready=True)
        tcp, ip, port = sitl.connection_string().split(':')
        port = str(int(port) + instance * 10)
        conn_string = ':'.join([tcp, ip, port])
        print('Connecting to vehicle on: %s' % conn_string)

        vehicle = connect(conn_string)
        vehicle.wait_ready(timeout=120)

        # Collections
        vehicle.direction = start_dir
        self.copters.append(vehicle)
        self.sitls.append(sitl)

    def copters_at_altitude(self, aTargetAltitude):
        while True:
            at_altitude = True
            ctr=1
            for c in self.copters:
                print ('Copter ID: {} at altitude {} '.format(ctr,str(c.location.global_relative_frame.alt))) 
                ctr = ctr + 1
                if (not c.location.global_relative_frame.alt >= aTargetAltitude * 0.95):
                    at_altitude = False
            time.sleep(3)

            if at_altitude == True:
                print("All drones have reached their target altitudes")
                break     

    def copters_arm(self):
        for c in self.copters:
            c.mode = VehicleMode("GUIDED")
            c.armed = True

        for c in self.copters:
            while not (c.armed):
                time.sleep(1)

    def land_drones(self):
        for c in self.copters:
            c.mode = VehicleMode("LAND")
        print ("LANDING....")
        time.sleep(5)

    def copters_armable(self):
    
        while True:
            unarmable = False
            for c in self.copters:
                if (not c.is_armable):
                    unarmable = True
            time.sleep(3)

            if unarmable == False:
                break     


    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        self.copters_armable()
    
        print("Arming motors")
        self.copters_arm()
    
        print("Vehicle armed!")

        print("All drones are now Taking off!")
        aTargetAltitude = 10
        for c in self.copters:
            c.simple_takeoff(aTargetAltitude)  # Take off to target altitude
            c.init_x = c.location.global_relative_frame.lat
            c.init_y = c.location.global_relative_frame.lon

        print("Waiting for copters to ascend")
        self.copters_at_altitude(aTargetAltitude)

#Responsible for keeping track of coordinates and eventually plotting the drones' journeys
class Location_Tracker:

    def __init__(self):

        self.x_cor = []
        for i in range(2):
            self.x_cor.append([])
        self.y_cor = []
        for i in range(2):
            self.y_cor.append([])
        self.z_cor = []
        for i in range(2):
            self.z_cor.append([])

    def plot_journey(self):

        plot_colors = ['bo', 'ro']

        #fig = plt.figure()
        #ax = plt.axes(projection='3d')
        #ax.plot3D(self.x_cor[0], self.y_cor[0], self.z_cor[0])

        for index in range(2):
            plt.plot(self.x_cor[index], self.y_cor[index], plot_colors[index])

        plt.xlim(41.7142, 41.7148)
        plt.ylim(-86.2419, -86.2412)
        plt.show()

    def add_xcor(self, x_list):
        for index, x_val in enumerate(x_list):
            self.x_cor[index].append(x_val)

    def add_ycor(self, y_list):
        for index, y_val in enumerate(y_list):
            self.y_cor[index].append(y_val)

    def add_zcor(self, z_list):
        for index, z_val in enumerate(z_list):
            self.z_cor[index].append(z_val)

#Defines the behaviors of drones
class Movements:

    @staticmethod
    def get_next_point(cur_l, tar_l):
        if abs(cur_l.lat - tar_l.lat) > abs(cur_l.lon - tar_l.lon):
            frac = .00001 / abs(cur_l.lat - tar_l.lat)
            if cur_l.lat > tar_l.lat:
                lat = cur_l.lat - .00001
            else:
                lat = cur_l.lat + .00001
            if cur_l.lon > tar_l.lon:
                lon = cur_l.lon - .00001*frac
            else:
                lon = cur_l.lon + .00001*frac
        else:
            frac = .00001 / abs(cur_l.lon - tar_l.lon)
            if cur_l.lat > tar_l.lat:
                lat = cur_l.lat - .00001*frac
            else:
                lat = cur_l.lat + .00001*frac
            if cur_l.lon > tar_l.lon:
                lon = cur_l.lon - .00001
            else:
                lon = cur_l.lon + .00001
        return Location(lat, lon)

    @staticmethod
    def avoid_collision(copters, loc):
        target_2 = [41.71444,-86.2418,10]
        target_1 = [41.71446,-86.2413,10]
        #target_2 = [41.71445,-86.2418,10]
        #target_1 = [41.71445,-86.2412,10]

        t2_loc = Location(target_2[0], target_2[1])
        t1_loc = Location(target_1[0], target_1[1])

        nedcontroller = ned_controller()

        cl1 = Location(copters[0].location.global_relative_frame.lat, copters[0].location.global_relative_frame.lon)
        cl2 = Location(copters[1].location.global_relative_frame.lat, copters[1].location.global_relative_frame.lon)

        #ned1 = nedcontroller.setNed(cl1, t1_loc)
        #ned2 = nedcontroller.setNed(cl2, t2_loc)
        ned1 = nedcontroller.setNed(cl1, Movements.get_next_point(cl1, t1_loc))
        ned2 = nedcontroller.setNed(cl2, Movements.get_next_point(cl2, t2_loc))
        nedcontroller.send_ned_velocity(ned1.north, ned1.east, ned1.down, 1, copters[0])
        nedcontroller.send_ned_velocity(ned2.north, ned2.east, ned2.down, 1, copters[1])

        #print()

        c1_dist = distance.distance((cl1.lat, cl1.lon), (target_1[0], target_1[1])).meters
        c2_dist = distance.distance((cl2.lat, cl2.lon), (target_2[0], target_2[1])).meters

        # While both drones not at target:
        print('Copter1 distance to target: ' + str(c1_dist))
        print('Copter2 distance to target: ' + str(c2_dist))
        print('cop1: ' + str(cl1.lat) + ', ' + str(cl1.lon))
        print('cop2: ' + str(cl2.lat) + ', ' + str(cl2.lon))
        while c1_dist > 1 or c2_dist > 1:
            print('Copter1 distance to target: ' + str(c1_dist))
            print('Copter2 distance to target: ' + str(c2_dist))

            cl1 = Location(copters[0].location.global_relative_frame.lat, copters[0].location.global_relative_frame.lon)
            cl2 = Location(copters[1].location.global_relative_frame.lat, copters[1].location.global_relative_frame.lon)
            
            drone_dist = distance.distance((cl1.lat, cl1.lon), (cl2.lat, cl2.lon)).meters
            print('Distance: ' + str(drone_dist))

            while drone_dist < 8:
                if drone_dist < 3:
                    if abs(cl1.lat - cl2.lat) < abs(cl1.lon - cl2.lon):
                        nedcontroller.send_ned_velocity(ned1.north+5, ned1.east, ned1.down, 1, copters[0])
                        nedcontroller.send_ned_velocity(ned2.north-5, ned2.east, ned2.down, 1, copters[1])
                    else:
                        nedcontroller.send_ned_velocity(ned1.north, ned1.east+5, ned1.down, 1, copters[0])
                        nedcontroller.send_ned_velocity(ned2.north, ned2.east-5, ned2.down, 1, copters[1])
                else:
                    if abs(cl1.lat - cl2.lat) < abs(cl1.lon - cl2.lon):
                        nedcontroller.send_ned_velocity(ned1.north+2, ned1.east, ned1.down, 1, copters[0])
                        nedcontroller.send_ned_velocity(ned2.north-2, ned2.east, ned2.down, 1, copters[1])
                    else:
                        nedcontroller.send_ned_velocity(ned1.north, ned1.east+2, ned1.down, 1, copters[0])
                        nedcontroller.send_ned_velocity(ned2.north, ned2.east-2, ned2.down, 1, copters[1])
                cl1 = Location(copters[0].location.global_relative_frame.lat, copters[0].location.global_relative_frame.lon)
                cl2 = Location(copters[1].location.global_relative_frame.lat, copters[1].location.global_relative_frame.lon)
                print('lat: ' + str(cl1.lat) + ',\tlon: ' + str(cl1.lon))
                print('lat: ' + str(cl2.lat) + ',\tlon: ' + str(cl2.lon))
                print('Distance: ' + str(drone_dist))
                loc.add_xcor([cl1.lat, cl2.lat])
                loc.add_ycor([cl1.lon, cl2.lon])
                drone_dist = distance.distance((cl1.lat, cl1.lon), (cl2.lat, cl2.lon)).meters
                #time.sleep(.2)
            
            
            #ned1 = nedcontroller.setNed(cl1, t1_loc)
            #ned2 = nedcontroller.setNed(cl2, t2_loc)
            ned1 = nedcontroller.setNed(cl1, Movements.get_next_point(cl1, t1_loc))
            ned2 = nedcontroller.setNed(cl2, Movements.get_next_point(cl2, t2_loc))
            nedcontroller.send_ned_velocity(ned1.north, ned1.east, ned1.down, 1, copters[0])
            nedcontroller.send_ned_velocity(ned2.north, ned2.east, ned2.down, 1, copters[1])

            print('lat: ' + str(cl1.lat) + ',\tlon: ' + str(cl1.lon))
            print('lat: ' + str(cl2.lat) + ',\tlon: ' + str(cl2.lon))
            loc.add_xcor([cl1.lat, cl2.lat])
            loc.add_ycor([cl1.lon, cl2.lon])

            c1_dist = distance.distance((cl1.lat, cl1.lon), (target_1[0], target_1[1])).meters
            c2_dist = distance.distance((cl2.lat, cl2.lon), (target_2[0], target_2[1])).meters

            #time.sleep(.2)


gc = Ground_Control()
loc = Location_Tracker()
Movements.avoid_collision(gc.copters, loc)
gc.land_drones()
loc.plot_journey()