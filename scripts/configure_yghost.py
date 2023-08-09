#!/usr/bin/python3
 
from websocket import create_connection
import json
import argparse


class YghostInit:
    def __init__(self, ip: str):
        """Class to start/stop stations on YGhost and get their status

        Args:
            ip (str): ip of YGhost
        """
        self.ip = ip
        self.stop_msg = {"action": "stop", "id": 1}
        self.stop_all_msg = {"action": "stop_all"}
        self.start_msg = {
            "action": "start",
            "id": 1,
            "its_station": {},
        }

    def get_station_status(self) -> dict:
        """Get the current status of yghost stations as a dictionnary

        Returns:
            dict: dictionnary describing the status of YGhost stations
        """

        socket = create_connection("ws://{}/status".format(self.ip))
        msg = socket.recv()
        socket.close()
        return msg

    def get_num_stations(self) -> int:
        """Get the number of station already running on YGhost

        Returns:
            int: number of station running
        """

        return self.get_station_status()["num_stations"]

    def start_station(self, id: iter, station_ids=None, config=None):
        """Starts one or multiple stations on YGhost

        Args:
            id (list or int): id or list of ids of environments to be started
            log (bool, optional): displays a message each time a station is started if true. Defaults to False.
            config (dict): config to start the stations with 
            station_ids (list or int): value or list of values of station _id, must be the same length as id or None (default)
        """
        if station_ids != None and len(station_ids) != len(id):
            raise Exception("size of station_ids is not the same as id")
        
        ids = [id] if type(id) == int else id # if id is int, we put it in a list
        msg = self.start_msg
        if config != None:
            msg["its_station"] = config
            
        for index, i in enumerate(ids):
            if station_ids != None:
                msg["its_station"]["services"]["10-IdentityManager"] = \
                {
                    "identity.stationid": station_ids[index],
                }
                
            msg["id"] = i
            socket = create_connection("ws://{}/ctl".format(self.ip))
            socket.send(json.dumps(msg))
            # Note : closing the socket each time is the simplest way I found to be sure that YGhost had enough time to create a stations
            # Other wise we would just spam YGhost with websocket request and it wouldn't manage and fail to create all stations
            socket.close()
            print("Started station of id : ", i)
                

    def stop_station(self, env_id: int or list):
        """Stops the stations corresponding to the specified id on YGhost
        Args:
            env_id (list or int) : list of stations to stop
        Returns:
            None
        """
        msg = self.stop_msg
        ids = [env_id] if type(env_id) == int else env_id # if id is int, we put it in a list
        for id in ids:  
            socket = create_connection("ws://{}/ctl".format(self.ip))
            msg["id"] = id
            socket.send(json.dumps(msg))
            socket.close()
            print("Stopped station of id ", id)

    def stop_all(self):
        """Stops all station currently running on YGhost"""
        msg = self.stop_all_msg
        socket = create_connection("ws://{}/ctl".format(self.ip))
        socket.send(json.dumps(msg))
        socket.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "ip",
        help="ip of Yghost",
        type=str)
    parser.add_argument(
        "--stop_all",
        help="stops all stations",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--stop",
        help="stops station of the specified ids",
        default=None,
        required=False,
        type=int,
        nargs='+',
    )
    parser.add_argument(
        "--start",
        help="id(s) of station to start",
        default=0,
        required=False,
        type=int,
        nargs='+',
    )
    
    parser.add_argument(
        "--status",
        help="get status",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--config",
        help="path to config file",
        type=str,
        required=False,
    )
    parser.add_argument(
        "--station_id",
        help="station id of the statio, if multiple stations to start, will add env_id to station id for each station",
        type=int,
        required=False,
        nargs='+',
    )
    
    args = parser.parse_args()
    init = YghostInit(args.ip)
    config = None
    if args.config:
        with open(args.config, "r") as json_file:
            config = json.load(json_file)

    if args.stop_all:
        init.stop_all()
    elif args.stop != None:
        init.stop_station(args.stop)

    if args.start:
        station_id = None
        if args.station_id:
            station_id = args.station_id
        init.start_station(args.start, station_ids=station_id, config=config)


    if args.status:
        print(init.get_station_status())
