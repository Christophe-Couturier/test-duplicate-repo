import carla
import random
import time
import math 
import logging
import queue
import signal
import sys

def spawn_walkers(nb_ped, world, client, blueprintsWalkers):


    SpawnActor = carla.command.SpawnActor
    walkers_list = []
    all_id = []
    # some settings
    percentagePedestriansRunning = 10.0     # how many pedestrians will run
    percentagePedestriansCrossing = 10.0    # how many pedestrians will cross the road
    numberPedestrians = nb_ped
    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(numberPedestrians):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. we spawn the walker objects
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
        	walkers_list.append({"id": results[i].actor_id})
        	walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    
    # 3. we spawn the walker controllers
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
        	logging.error(results[i].error)
            
        else:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put altogether the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
        
    all_actors = world.get_actors(all_id)

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    print('spawned %d walkers' % (len(walkers_list)))


def main():
	
	# Connect to the client and retrieve the world object
	client = carla.Client('localhost', 2000)
	world = client.get_world()
	#client.load_world('Town05')          #Uncomment if this client need to be started first.
	
	#Set synchronous mode, if not already done by another client
	synchronous_master = True
	settings = world.get_settings()
	settings.fixed_delta_seconds = 0.017
	
	
	if settings.synchronous_mode == True:
		synchronous_master = False            #another script has already enabled the synchronous mode
		print('Another client is the synchronous_master.\n')
	else:
		settings.synchronous_mode = True       
		print('I am the synchronous master !\n')
	world.apply_settings(settings)
	
	SpawnActor = carla.command.SpawnActor
	SetAutopilot = carla.command.SetAutopilot
	FutureActor = carla.command.FutureActor
	
	# Get Traffic Manager
	tm = client.get_trafficmanager(8000)
	tm.set_synchronous_mode(True)
	
	# Get the blueprint library and filter for the vehicle & walker blueprints
	vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
	walker_blueprints = world.get_blueprint_library().filter('*walker.pedestrian*')

	# Get the map's spawn points
	spawn_points = world.get_map().get_spawn_points()
	actors_list = []
	
	### DEBUG ###
	'''
	for i, spawn_point in enumerate(spawn_points):
		# Draw in the spectator window the spawn point index
		world.debug.draw_string(spawn_point.location, str(i), life_time=100)
	'''
	#############
	
	# Spawn vehicles randomly distributed throughout the map 
	# for each spawn point, we choose a random vehicle from the blueprint library
	
	batch = []
	vehicles_list = []
	max_vehicles = 1

	for n, transform in enumerate(spawn_points):
		if n <= max_vehicles -1:
			bp = random.choice(vehicle_blueprints)
			bp.set_attribute('role_name', 'autopilot')
			batch.append(SpawnActor(bp, transform)
					.then(SetAutopilot(FutureActor, True, tm.get_port())))


	for response in client.apply_batch_sync(batch, True):
		#if response.error:
		#   logging.error(response.error)
		#else:
		vehicles_list.append(response.actor_id)

	# Set automatic vehicle lights update if specified
	all_vehicle_actors = world.get_actors(vehicles_list)
	for actor in all_vehicle_actors:
		tm.update_vehicle_lights(actor, True)

	print('Spawned %d cars, bikes and motorbikes!' % (len(vehicles_list)))
	print('Wait a few seconds and spawn pedestrians...')

	# Wait a bit for the spawn points to empty themselves			  
	for i in range(100):
		
		if synchronous_master == True:
			world.tick()     
		   
		else:
			world.wait_for_tick() 

	# Spawn walkers with the dedicated function

	spawn_walkers(1 , world, client, walker_blueprints)



	while True:
		print('Running...', end='\r', flush='True')
		
		try:
			if synchronous_master == True:
				world.tick()     # only the master need to tick
		   
			else:
				world.wait_for_tick() 
				
		except:
			print("\nCTRL+C caught, destroying all vehicles...")
			settings = world.get_settings()
			settings.synchronous_mode = False
			world.apply_settings(settings)
			tm.set_synchronous_mode(False)
   
			for vehicle in world.get_actors().filter('*vehicle*'):
				vehicle.destroy()

			print("Bye!")
			sys.exit(0)
	
	
	
if __name__ == '__main__':

	main()

	
