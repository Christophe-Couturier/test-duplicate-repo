import carla
import random
import time
import math 
import queue
import signal
import sys
import csv
import choir
import json

def print_on_terminal(gt, sensors, decimals):

	if decimals == None:
		s = "GROUND TRUTH: \n* Heading: {0!s}\n* Position: ({1!s}, {2!s}, {3!s})\n* Velocity: ({4!s}, {5!s}, {6!s})\n* Norm of velocity: {12!s}\n\n SENSOR MEASUREMENTS: \n* Heading: {7!s}\n* Position: ({8!s}, {9!s}, {10!s})\n* Speed: {11!s}\r".format(gt.heading, gt.lat, gt.lon, gt.alt, gt.xspeed, gt.yspeed, gt.zspeed, sensors.imu_heading, sensors.gnss_lat, sensors.gnss_lon, sensors.gnss_alt, sensors.gnss_derived_speed, gt.norm_speed)
	else:
		s = "GROUND TRUTH: \n* Heading: {0!s}\n* Position: ({1!s}, {2!s}, {3!s})\n* Velocity: ({4!s}, {5!s}, {6!s})\n* Norm of velocity: {12!s}\n\n SENSOR MEASUREMENTS: \n* Heading: {7!s}\n* Position: ({8!s}, {9!s}, {10!s})\n* Speed: {11!s}\r".format(gt.heading, gt.lat, gt.lon, gt.alt, gt.xspeed, gt.yspeed, gt.zspeed, sensors.imu_heading, sensors.gnss_lat, sensors.gnss_lon, sensors.gnss_alt, round(sensors.gnss_derived_speed, decimals), round(gt.norm_speed, decimals))

	L_UP = '\033[1A'
	L_CLR = '\x1b[2K'
	print(s, end='\r')
	print(L_UP, end=L_CLR)
	print(L_UP, end=L_CLR)
	print(L_UP, end=L_CLR)
	print(L_UP, end=L_CLR)
	print(L_UP, end=L_CLR)
	print(L_UP, end=L_CLR)
	print(L_UP, end=L_CLR)
	print(L_UP, end=L_CLR)
	print(L_UP, end=L_CLR)


def initialize_csv():

	filename = '/home/yogoko/Documents/ego_vehicle.csv'
	header = ['GT lat', 'MEAS lat', 'GT lon', 'MEAS lon', 'GT alt', 'MEAS alt', 'GT heading', 'MEAS heading', 'GT velocity x', 'MEAS velocity x', 'GT velocity y', 'MEAS velocity y', 'GT velocity z', 'MEAS velocity z']
	
	f = open(filename, 'w', encoding='UTF8')
	writer = csv.writer(f)

	# write the header
	writer.writerow(header)

	return f

def fill_csv(f, gt, sensors):

	data = [gt.lat, sensors.gnss_lat, gt.lon, sensors.gnss_lon, gt.alt, sensors.gnss_alt, gt.heading, sensors.imu_heading, gt.xspeed, '0', gt.yspeed, '0', gt.zspeed, '0']

	writer = csv.writer(f)

	# write the data
	writer.writerow(data)

def publish_to_pvt_service(choir_handler, envid, sensors):
	
	message = message = {"latitude":sensors.gnss_lat,"longitude":sensors.gnss_lon}
	json_message = json.dumps(message)
	choir_handler.publish_yghost(203, json_message, envid)
	#choir_handler.publish(203, json_message)


def speed_from_haversine_distance(sensors):
	R = 6371000
	phi1 = sensors.gnss_lat*(math.pi)/180
	phi2 = sensors.gnss_lat_prev*(math.pi)/180
	delta_phi = (sensors.gnss_lat - sensors.gnss_lat_prev)*(math.pi)/180
	delta_lambda = (sensors.gnss_lon - sensors.gnss_lon_prev)*(math.pi)/180

	a = math.sin(delta_phi/2)*math.sin(delta_phi/2) + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)*math.sin(delta_lambda/2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	
	d = R * c 
	v = d/(sensors.gnss_timestamp-sensors.gnss_timestamp_prev)
	return v 

