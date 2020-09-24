#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
from geopy import distance
import matplotlib.pyplot as plt
from dronekit_sitl import SITL
from dronekit import Vehicle, VehicleMode, connect, LocationGlobalRelative

#responsible for connecting to drones, and having them get up to altitude
class Ground_Control:

    def __init__(self):
        self.copters = []
        self.sitls = []

        # Starting coordinates
        start_coordinates = [41.714429, -86.242085,0]
        offset = .00005

        # Copter list
        coordinates = [start_coordinates[0],start_coordinates[1],start_coordinates[2]]
        self.connect_virtual_vehicle(0,coordinates, 0)

        # top right
        coordinates = [start_coordinates[0]+offset,start_coordinates[1]+offset,start_coordinates[2]]
        self.connect_virtual_vehicle(1,coordinates, 1)

        # top left
        coordinates = [start_coordinates[0]+offset,start_coordinates[1]-offset,start_coordinates[2]]
        self.connect_virtual_vehicle(2,coordinates, 4)

        # bottom right
        coordinates = [start_coordinates[0]-offset,start_coordinates[1]+offset,start_coordinates[2]]
        self.connect_virtual_vehicle(3,coordinates, 2)

        # bottom left
        coordinates = [start_coordinates[0]-offset,start_coordinates[1]-offset,start_coordinates[2]]
        self.connect_virtual_vehicle(4,coordinates, 3)

        # Arm and takeoff to 10 meters
        self.arm_and_takeoff(10) 

    def connect_virtual_vehicle(self, instance, home, start_dir):
        sitl = SITL()
        sitl.download('copter', '3.3', verbose=True)
        instance_arg = '-I%s' %(str(instance))
        print("Drone instance is: %s" % instance_arg)
        home_arg = '--home=%s, %s,%s,180' % (str(home[0]), str(home[1]), str(home[2]))
        sitl_args = [instance_arg, '--model', 'quad', home_arg]
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
        time.sleep(30)

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
        for i in range(5):
            self.x_cor.append([])
        self.y_cor = []
        for i in range(5):
            self.y_cor.append([])

    def plot_journey(self):

        plot_colors = ['bo', 'ro', 'co', 'mo', 'ko']

        for index in range(5):
            plt.plot(self.x_cor[index], self.y_cor[index], plot_colors[index])
        41.714429, -86.242085
        plt.xlim(41.714329, 41.714529)
        plt.ylim(-86.242185, -86.241985)
        plt.show()

    def add_xcor(self, x_list):
        for index, x_val in enumerate(x_list):
            self.x_cor[index].append(x_val)

    def add_ycor(self, y_list):
        for index, y_val in enumerate(y_list):
            self.y_cor[index].append(y_val)

#Defines the behaviors of dances for a given list of drones
class Dances:

    #Loops and tracks coodinates of drones until all are at their desired points
    @staticmethod
    def at_target(copters, point_list, loc):

        curr = []
        x_list = []
        y_list = []

        for n in range(5):
            curr.append(copters[n].location.global_relative_frame)

        curr_dist0 = distance.distance((curr[0].lat, curr[0].lon), (point_list[0].lat, point_list[0].lon)).meters
        curr_dist1 = distance.distance((curr[1].lat, curr[1].lon), (point_list[1].lat, point_list[1].lon)).meters
        curr_dist2 = distance.distance((curr[2].lat, curr[2].lon), (point_list[2].lat, point_list[2].lon)).meters
        curr_dist3 = distance.distance((curr[3].lat, curr[3].lon), (point_list[3].lat, point_list[3].lon)).meters
        curr_dist4 = distance.distance((curr[4].lat, curr[4].lon), (point_list[4].lat, point_list[4].lon)).meters
        
        print('Copter 1 is %s from target' %  curr_dist1)
        print('Copter 2 is %s from target' %  curr_dist2)
        print('Copter 3 is %s from target' %  curr_dist3)
        print('Copter 4 is %s from target' %  curr_dist4)
        
        while curr_dist0 > 1 or curr_dist1 > 1 or curr_dist2 > 1 or curr_dist3 > 1 or curr_dist4 > 1:
           
            for n in range(5):
                curr[n] = copters[n].location.global_relative_frame
                x_list.append(curr[n].lat)
                y_list.append(curr[n].lon)

            loc.add_xcor(x_list)
            loc.add_ycor(y_list) 

            curr_dist0 = distance.distance((curr[0].lat, curr[0].lon), (point_list[0].lat, point_list[0].lon)).meters
            curr_dist1 = distance.distance((curr[1].lat, curr[1].lon), (point_list[1].lat, point_list[1].lon)).meters
            curr_dist2 = distance.distance((curr[2].lat, curr[2].lon), (point_list[2].lat, point_list[2].lon)).meters
            curr_dist3 = distance.distance((curr[3].lat, curr[3].lon), (point_list[3].lat, point_list[3].lon)).meters
            curr_dist4 = distance.distance((curr[4].lat, curr[4].lon), (point_list[4].lat, point_list[4].lon)).meters

            x_list = []
            y_list = []

            time.sleep(.3)

    # 0: stationary
    # 1: down
    # 2: left
    # 3: up
    # 4: right
    
    #Defines the dance where drones track around a square outline
    @staticmethod
    def cube(copters, loc):
        
        point_list = []
        offset = .0001

        start_time = time.time()

        while time.time() - start_time < 40:
            print("start move")
            for copter in copters:
                x = copter.location.global_relative_frame.lat
                y = copter.location.global_relative_frame.lon

                #Update drone directions
                if copter.direction == 0:
                    pass
                elif copter.direction == 1:
                    x -= offset
                    copter.direction = 2
                elif copter.direction == 2:
                    y -= offset
                    copter.direction = 3
                elif copter.direction == 3:
                    x += offset
                    copter.direction = 4
                else:
                    y += offset
                    copter.direction = 1

                point = LocationGlobalRelative(x, y, 10)

                point_list.append(point) 

                copter.simple_goto(point)

            print("waiting for coptor to reach target")
            #Block until all drones are at their target points
            Dances.at_target(copters, point_list, loc)
            print("coptors reached target")
            point_list = []


    # 0: stationary
    # 1: right + small down
    # 1.5: left + small down
    # 2: down + little left
    # 2.5: up + little left
    # 3: left + little up
    # 3.5: right + little up
    # 4: up + little right
    # 4.5: down + little right

    #Defines the dance where drones track along the outline of a star shape
    @staticmethod
    def star(copters, loc):
        
        point_list = []
        offset = .00005

        start_time = time.time()

        while time.time() - start_time < 40:
            print("start move")
            for copter in copters:
                x = copter.location.global_relative_frame.lat
                y = copter.location.global_relative_frame.lon

                #Update drone directions
                if copter.direction == 0:
                    pass
                elif copter.direction == 1:
                    x -= offset
                    y += offset * 2
                    copter.direction = 1.5
                elif copter.direction == 1.5:
                    x -= offset
                    y -= offset * 2
                    copter.direction = 2
                elif copter.direction == 2:
                    x -= offset * 2
                    y -= offset
                    copter.direction = 2.5
                elif copter.direction == 2.5:
                    x += offset * 2
                    y -= offset
                    copter.direction = 3
                elif copter.direction == 3:
                    x += offset
                    y -= offset * 2
                    copter.direction = 3.5
                elif copter.direction == 3.5:
                    x += offset
                    y += offset * 2
                    copter.direction = 4
                elif copter.direction == 4:
                    x += offset * 2
                    y += offset
                    copter.direction = 4.5
                elif copter.direction == 4.5:
                    x -= offset * 2
                    y += offset
                    copter.direction = 1

                point = LocationGlobalRelative(x, y, 10)

                point_list.append(point) 

                copter.simple_goto(point)

            print("waiting for coptor to reach target")
            #Block until all drones are at their target points
            Dances.at_target(copters, point_list, loc)
            print("coptors reached target")
            point_list = []


gc = Ground_Control()
loc = Location_Tracker()
#Dances.cube(gc.copters, loc)
Dances.star(gc.copters, loc)
gc.land_drones()
loc.plot_journey()