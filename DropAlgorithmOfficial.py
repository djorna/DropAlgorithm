#! /usr/bin/python

# Aero-Design Drop Algorithm 2015/2016

# This program calculates payload release point to hit target on ground
# Variables
import numpy as np


#initial gps coordinates of aircraft
p_gps_x=0#north
p_gps_y=0#east
p_gps_z=30
#initial velocity of aircraft from gps
v_gps_x=1
v_gps_y=1
v_gps_z=33
#target location in gps coordinates
target_N=0
target_S=0
#accelerations
acc_z = -9.81
acc_x = 0
acc_y=0
#velocity of probe
v_body_y = 1
v_body_z = 0
v_body_x = 1;
probe_dir=0 
#position of probe 
pos_z=p_gps_z
pos_x=p_gps_x
pos_y=p_gps_y
#velocity of wind

v_wind_x=0
v_wind_y=0

curr_time = 0
land_x = 0
land_y = 0
in_drop_pos = False

#sample period (seconds)
period=0.0001
p=1.2754 #common air density, kg/m^3

#probe constants for drag calculations
diam=11.5/100
length=10.0/100
coef_rect=0.82
coef_circ=0.47

#calculate components of wind
v_wind_y=v_body_y - v_gps_y
v_wind_x=v_body_x - v_gps_x

#get direction of probe w.r.t. gps
probe_dir = np.arctan(v_gps_y/v_gps_x) #direction of probe relative to earth axes, cw from north

#get speed from the initial direction of the plane

 
while not in_drop_pos:
        #calculate fall time and position:
        while pos_z>0:
                #calculate accelerations in body coordinates
                acc_z = -9.81+0.5*p*coef_rect*diam*length*(v_body_z**2)
                acc_x = -p*coef_circ*diam*length*(v_body_x+v_wind_x*np.cos(probe_dir)+v_wind_y*np.sin(probe_dir))**2
                acc_y = -p*coef_circ*diam*length*(v_wind_x*np.sin(probe_dir)+v_wind_y*np.cos(probe_dir))**2
                #calculate position
                pos_x += (v_body_x+v_wind_x*np.cos(probe_dir)+v_wind_y*np.sin(probe_dir))*period + 0.5*acc_x*period**2
                pos_z += v_body_z*period + 0.5*acc_z*period**2
                pos_y += (v_wind_x*np.sin(probe_dir)+v_wind_y*np.cos(probe_dir))*period + 0.5*acc_y*period**2

                #calculate initial speed for next sample period
                v_body_x+=period*acc_x
                v_body_z+=period*acc_z
                v_body_y+=period*acc_y

                #increment time by one step
                curr_time+=period

                #convert to Earth axes
                p_gps_y=pos_x*np.cos(probe_dir)+pos_y*np.sin(probe_dir)
                p_gps_x=pos_x*np.sin(probe_dir)+pos_y*np.cos(probe_dir)    
        
        in_drop_pos=True

        #check distance from and direction to target
        print 'Distance from target: %f m' % np.sqrt((target_N-p_gps_x)**2+(target_S-p_gps_y)**2)
        if np.arctan((target_N-p_gps_x)/(target_S-p_gps_y)):#Check if angular displacement is +/-
                print 'Turn right'
        else:
                print 'Turn left'
        print 'Required angle adjustment: %f degrees from north' % (np.sqrt((target_N-p_gps_x)**2+(target_S-land_y)**2) - probe_dir)
        print 'Fall time: %f s' % curr_time
        #reset variables
        curr_time = 0
        #reset gps coordinates to plane position
        p_gps_x=0
        p_gps_y=0
         

        
        

