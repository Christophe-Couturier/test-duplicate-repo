#!/usr/bin/python3
import carla
import random as rd
import argparse
import time

def set_autopilot(world: carla.World, vehicle_id: int):
    vehicle = world.get_actor(vehicle_id)
    success = False
    while not success:
        try:
            vehicle.set_autopilot(True)
            success = True
        except Exception as e:
            print("Error trying to set autopilot, retrying : {}".format(e))
            pass
            time.sleep(0.03)

def spawn_vehicle(world: carla.World, blueprint: carla.ActorBlueprint, spawn_point: carla.Transform, role_name: str) -> int:
    blueprint.set_attribute("role_name", role_name)
    vehicle = None
    while vehicle == None:
        # If the car cannot be spawned we try again
        print("Trying to spawn vehicle {} at {}".format(role_name, spawn_point))
        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        time.sleep(0.03)
    return vehicle.id


def spawn_gnss(world: carla.World, vehicle_id: int, period: float) -> int:
    sensor_blueprint = world.get_blueprint_library().find("sensor.other.gnss")
    sensor_blueprint.set_attribute("sensor_tick", str(period))
    gnss = world.spawn_actor(
            sensor_blueprint,
            carla.Transform(carla.Location(x=0, z=0)),
            attach_to=world.get_actor(vehicle_id),
        )
    return gnss.id

def delete_actor(world: carla.World, actor_id: int):
    actor = world.get_actor(actor_id)
    if actor != None:
        try:
            actor.destroy()
        except Exception as e:
            print("Error when trying to destroy actor : {}".format(e))

def main():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--n", help="number of instances to start", type=int, required=True)
    args = parser.parse_args()
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    blueprints = world.get_blueprint_library().filter("vehicle")
    # Getting random spawnpoints
    spawn_points = rd.choices([waypoint for waypoint in world.get_map().get_spawn_points()], k=args.n)
    
    vehicles = [spawn_vehicle(world, rd.choice(blueprints), spawn_points[i], "cam_vehicle_{}".format(i + 1)) for i in range(args.n)]
    sensors = [spawn_gnss(world, vehicle_id, 0.1) for vehicle_id in vehicles]
    [set_autopilot(world, vehicle_id) for vehicle_id in vehicles]
    print("Spawned {} cam vehicles".format(args.n))
    try:
        while True:
            time.sleep(0.5)
    except:
        for actor_id in vehicles + sensors:
            delete_actor(world, actor_id)
    
if __name__=="__main__":
    main()
