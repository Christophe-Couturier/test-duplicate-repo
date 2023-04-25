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
import getopt

from utils_carla import *


class Ground_Truth:

	def __init__(self):
		self.lat = 0
		self.lon = 0
		self.alt = 0
		self.heading = 0
		self.xspeed = 0
		self.yspeed = 0
		self.zspeed = 0
		self.norm_speed = 0
		self.length = 0
		self.width = 0

	def get_dimensions(self, actor):
		self.length = int((actor.bounding_box.extent.x)*2*10)                    # *10 because the CDD specifies that the unit is 10 cm
		self.width = int((actor.bounding_box.extent.y)*2*10)                     # *10 because the CDD specifies that the unit is 10 cm


	def get_ground_truth_data(self, actor, carla_map):

		# Ground truth position
		location = carla_map.transform_to_geolocation(actor.get_location())
		self.lat = location.latitude
		self.lon = location.longitude
		self.alt = location.altitude
				
		# Retrieving ground truth heading 
		if ((actor.get_transform().rotation.yaw < -90) and (actor.get_transform().rotation.yaw > -180) ):
			self.heading = actor.get_transform().rotation.yaw + 450
		else:
			self.heading = actor.get_transform().rotation.yaw + 90
		
		# Retrieving ground truth speed 
		v_vect = actor.get_velocity()
		v_norm = math.sqrt((v_vect.x**2)+(v_vect.y**2)+(v_vect.z**2))   #print norm for debug

		self.xspeed = actor.get_velocity().x
		self.yspeed = actor.get_velocity().y
		self.zspeed = actor.get_velocity().z
		self.norm_speed = v_norm

class Sensor_Measurements:
	
	def __init__(self):
		self.imu_heading = 0
		self.gnss_lat = 0
		self.gnss_lon = 0
		self.gnss_alt = 0
		self.gnss_lat_prev = 0
		self.gnss_lon_prev = 0
		self.gnss_alt_prev = 0
		self.gnss_timestamp = 0
		self.gnss_timestamp_prev = 0
		self.gnss_derived_speed = 0
		
	def get_gnss_pos(self, event):
		self.gnss_lat = event.latitude
		self.gnss_lon = event.longitude
		self.gnss_alt = event.altitude
		self.gnss_timestamp = event.timestamp
		self.gnss_derived_speed = speed_from_haversine_distance(self)
		
	def get_imu_heading(self, event):
		self.imu_heading = (360*event.compass)/(2*math.pi)                 

class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        

class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)



class Data_Extractor:
	def __init__(self, choir_ip, envid, app_id, is_ego=False):
		self.is_ego = is_ego
		self.choir_ip = choir_ip
		self.envid = envid
		self.app_id = app_id
		self.ground_truth = Ground_Truth()
		self.sensor_measurements = Sensor_Measurements()
		self.actor = None
		self.imu_sensor = None 
		self.gnss_sensor = None
		self.choir_handler = None
		self.carla_client = None
		self.carla_world = None
		self.carla_map = None
		self.logfile = None


	def initialize_connections(self):
	
		try:
			self.choir_handler = choir.Choir(ip=self.choir_ip, app_id=self.app_id)
			#choir_handler.publish_yghost(123, "toto", 2)
			#choir_handler.publish(123, "toto", 2)
		except Exception as ex:
			print("Exception occured: {}".format(ex))
	
		# Connect to the client and retrieve the world object
		self.carla_client = carla.Client('localhost', 2000)  
		self.carla_world = self.carla_client.get_world()

		# Set synchronous mode, if not already done by another client
		synchronous_master = True
		settings = self.carla_world.get_settings()
		settings.fixed_delta_seconds = 0.017
		
		if settings.synchronous_mode == True:
			synchronous_master = False            # Another script has already enabled the synchronous mode
			print('Another client is the synchronous master.\n')
		else:
			settings.synchronous_mode = True       
			print('I am the synchronous master !\n')
		self.carla_world.apply_settings(settings)

		if self.is_ego == True:
			# Find the id of the hero vehicle
			vehicles = self.carla_world.get_actors().filter('*vehicle*')
			for vehicle in vehicles:
				if vehicle.attributes.get('role_name') == 'hero':
					self.actor = vehicle
					print("Id of ego_vehicle : " + str(self.actor.id))

			# Retrieve the sensors placed on ego vehicle
			gnss_sensors_list = self.carla_world.get_actors().filter('sensor.other.gnss')
			
			imu_sensors_list = self.carla_world.get_actors().filter('sensor.other.imu')
			for elt in gnss_sensors_list:
				if self.carla_world.get_actor(elt.id).parent.id == self.actor.id:
					print(self.carla_world.get_actor(elt.id).parent.id)
					self.gnss_sensor = self.carla_world.get_actor(elt.id)
			
			for elt in imu_sensors_list:
				if self.carla_world.get_actor(elt.id).parent.id == self.actor.id:
					print(self.carla_world.get_actor(elt.id).parent.id)
					self.imu_sensor = self.carla_world.get_actor(elt.id)
			print('GNSS sensor id of ego: ', str(self.gnss_sensor.id), ' IMU sensor id : ', str(self.imu_sensor.id), '\n')
						
		 
		else: #choose randomly a NPC vehicle
			vehicles = self.carla_world.get_actors().filter('*vehicle*')
			self.actor = vehicles[random.randrange(len(vehicles))]
				
			# attach an IMU and a GNSS sensor
			self.imu_sensor = IMUSensor(self.actor).sensor
			self.gnss_sensor = GnssSensor(self.actor).sensor
			print('GNSS sensor id of actor ', str(self.actor.id),': ', str(self.gnss_sensor.id), ' IMU sensor id : ', str(self.imu_sensor.id), '\n')


		# Indicate the callback functions to call at each sensor measurement
		self.gnss_sensor.listen(self.sensor_measurements.get_gnss_pos)
		self.imu_sensor.listen(self.sensor_measurements.get_imu_heading)

		# Get the map from the simulator
		self.carla_map = self.carla_world.get_map()

		#Initialize CSV log file
		self.logfile = initialize_csv()


	def extract_data(self, decimals):

		# Retrieve the dimensions of the vehicle
		self.ground_truth.get_dimensions(self.actor)

		# Send the dimensions of the vehicle to the VDP cache
		dimensions={"vehiclelength":self.ground_truth.length, "vehiclewidth":self.ground_truth.width, "vehiclelengthconfidence":0}
		self.choir_handler.cache_set_yghost(0,1,dimensions, self.envid)

		# Fill the Ground_Truth instance
		self.ground_truth.get_ground_truth_data(self.actor, self.carla_map) 
		# The Sensor_Measurements instance is filled by the callback functions at each sensor event.

		# Print the sensors data and ground truth data on the terminal + fill the log file	
		print_on_terminal(self.ground_truth, self.sensor_measurements, decimals)
		fill_csv(self.logfile, self.ground_truth, self.sensor_measurements)

		# Publish to the PVT service and set data into the VDP cache of the middleware
		publish_to_pvt_service(self.choir_handler, self.envid, self.sensor_measurements)
		dynamics = {"heading":self.sensor_measurements.imu_heading, "speed":self.sensor_measurements.gnss_derived_speed}
		self.choir_handler.cache_set_yghost(0,1,dynamics, self.envid)

		# The current sensor measurements become the previous ones before the next simulation step
		self.sensor_measurements.gnss_lat_prev = self.sensor_measurements.gnss_lat
		self.sensor_measurements.gnss_lon_prev = self.sensor_measurements.gnss_lon
		self.sensor_measurements.gnss_alt_prev = self.sensor_measurements.gnss_alt
		self.sensor_measurements.gnss_timestamp_prev = self.sensor_measurements.gnss_timestamp

			
def helper():
	print("Usage: {}".format(__file__))
	print("Options:")
	print("\t-r [INTEGER]\t\tNumber of decimals used to round the speed values printed on the terminal. ")
	print("\t-h|--help\t\tPrints this help.")



def main():

	# Algorith selection variables
	decimals = None

	try:
		opts, args = getopt.getopt(sys.argv[1:],"hr:", ["help", "verbose"])
	except getopt.GetoptError as e:
		print("error : {}".format(e))
		helper()
		sys.exit(2)
	for opt, arg in opts:
		if opt in ("-h", "--help"):
			helper()
			exit(0)
		elif opt == "-r":
			decimals = int(arg)


	# Instantiate the Data_Extractor objects
	manual_controlled_vehicle = Data_Extractor("192.168.202.176", 1, 12345678, is_ego=True)
	npc_vehicle = Data_Extractor("192.168.202.176", 2, 12345677)
	
	# Do everything that only need to be done once: connect to Choir and CARLA...
	manual_controlled_vehicle.initialize_connections()
	npc_vehicle.initialize_connections()


	while(True):
		# Get the simulator and sensors data for this tick
		manual_controlled_vehicle.extract_data(decimals)
		npc_vehicle.extract_data(decimals)

		# Wait for the simulator to tick before doing everything again
		manual_controlled_vehicle.carla_world.wait_for_tick()
		npc_vehicle.carla_world.wait_for_tick()



if __name__ == '__main__':

	main()

