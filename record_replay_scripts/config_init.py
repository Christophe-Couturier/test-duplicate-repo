#!/usr/bin/python3
import choir
import json
import argparse
from recorder_utils import launch_simoultaneously
from typing import Dict, List
import os
from websocket import create_connection

service_order_filename = os.getcwd() + "/service_order.json"


class MultiConfigManager:
    def __init__(
        self,
        ip: str,
        port_base: int,
        app_id_base: int,
        env_ids: list,
        main_port: int = 8080,
        offline: bool = False,
    ):
        """Class to handle multiple Config Managers for multiple environment on the same ip (usually YGhost)
        Handles the websocket request in parallel to reduce exec time

        Args:
            ip (str): ip of YGhost
            port_base (int): base port used for the port mapping by YGhost
            app_id_base (int): base app_id to use, will use app_id + env_id for each environent
            env_ids (list): list of env_ids to manage
            offline (bool, optional): Wether to do the operation offline or not. Defaults to False.
        """
        self.ip = ip
        self.main_port = main_port
        self.port_base = port_base
        self.app_id_base = app_id_base
        self.offline = offline

        if env_ids == None:
            return

        self.config_manager: Dict[int:ConfigManager] = {
            id: ConfigManager(
                ip=ip, port=port_base + id, app_id=app_id_base + id, offline=offline
            )
            for id in env_ids
        }

    def gather_config(self, file_ns, services: List[str] = None):
        """Gather the configs of the environments and write them to a json file

        Args:
            file_ns (str): path to the json namespace, eg : /home/yghost/configs/config_ , will write to /home/yghost/configs/config_1.json for env1
            services (List[str], optional): List of services of which to save config if None will save config of all services. Defaults to None
        """
        tasks = [
            {
                "name": "env_id {}".format(id),
                "target": manager.config_save_procedure,
                "args": ["{}{}.json".format(file_ns, id)],
                "kwargs": {"services": services},
            }
            for id, manager in self.config_manager.items()
        ]
        
        launch_simoultaneously(tasks, "Error when saving config of ", use_event=False)

    def stop_stations(self):
        for env_id, _ in self.config_manager.items():
            stop_msg = json.dumps({"action": "stop", "id": env_id})
            socket = create_connection("ws://{}:{}/ctl".format(self.ip, self.main_port))
            socket.send(stop_msg)
            socket.close()
            print("Stopped station of id : ", env_id)

    def start_stations(self, file_ns, services: List[str] = None):
        for env_id, manager in self.config_manager.items():
            manager.read_json("{}{}.json".format(file_ns, env_id))
            start_msg = manager.get_start_json(env_id, services)
            socket = create_connection("ws://{}:{}/ctl".format(self.ip, self.main_port))
            socket.send(start_msg)
            # Note : closing the socket each time is the simplest way I found to be sure that YGhost had enough time to create a stations
            # Other wise we would just spam YGhost with websocket request and it wouldn't manage and fail to create all stations
            socket.close()
            print("Started station of id : ", env_id)
        # There is no need to start the services, all service specified in the start message are started

    def get_status(self):
        socket = create_connection("ws://{}:{}/status".format(self.ip, self.main_port))
        status = socket.recv()
        socket.close()
        return status


class ConfigManager:
    def __init__(
        self,
        ip: str = "127.0.0.1",
        port: int = 8080,
        app_id: int = 0,
        offline: bool = True,
    ):
        """Class to handle reading/writting config from the MW from/to a json file

        Args:
            ip (str, optional): ip of the MW. Defaults to "127.0.0.1".
            port (int, optional): port to use. Defaults to 8080.
            app_id (int, optional): application id to use. Defaults to 0.
            offline (bool, optional): wether to do config calls offline or not. Defaults to True.
        """
        self.offline = offline
        self.choir = choir.Choir(ip=ip, port=port, app_id=app_id)
        self.config: dict = {}
        self.service_order = {}
        try:
            with open(service_order_filename, "r") as order_file:
                self.service_order = json.load(order_file)
        except Exception as e:
            raise Exception("Error when loading config order service at {} : {}".format(service_order_filename, e))

    def read_mw(self, services: list = None):
        """Reads config from the middleware of the specified services, of services is not specified
        will read all services

        Args:
            services (list, optional): List of services to read config from
        """
        # getting the service and their status
        services = self.choir.config_list_services()

        for service in services:
            # for each service/instanceID we write an entry with status and mib config
            srv_name = service["serviceName"]
            status = service["status"]

            if srv_name not in self.config:
                self.config[srv_name] = {}

            self.config[srv_name]["status"] = status
            mib = self.choir.config_mib_get_all(srv_name, offline=self.offline)
            self.config[srv_name]["mib"] = mib

    def read_json(self, filename: str):
        """Reads the  config/status from a json file and saves it into a dictionnary to later be used
            If config is wrongly formatted will raise an exception

        Args:
            filename (str): path/to/file.json
        """
        try:
            with open(filename, "r") as file:
                json_dict = json.load(file)
            self.check_json(json_dict)
        except Exception as e:
            raise Exception("Error when loading json config at {} : {}".format(filename, e))

    @staticmethod
    def check_json(json_dict: dict):
        """Check if a json has the right format

        Args:
            json_dict (dict): json to check

        Returns:
            bool: True if the json is correct, false otherwise
        """
        for srv_name in json_dict:
            if ("mib" not in json_dict[srv_name]) or (
                "status" not in json_dict[srv_name]
            ):
                raise Exception("In json : {} doesn't have mib or status entry".format(srv_name))

    def write_json(self, filename: str):
        """Write the currently loaded config to a json

        Args:
            filename (str): path/to/file.json
        """
        with open(filename, "w") as file:
            json.dump(self.config, file, indent=4)

    def get_start_json(self, env_id: int, services: List[str] = None):
        """Obtain the message (in a json format) used to start a YGhost station

        Args:
            env_id (int): environment id of the station
            services (List[str], optional): List of service to start, if none all service which are started in saved
            config are started. Will not start service which are not in the saved config. Defaults to None.

        Returns:
            str: json message used to start a station
        """
        start_msg = {
            "action": "start",
            "id": env_id,
            "its_station": {},
        }
        if services == None:
            services = [srv_name for srv_name in self.config]

        if "itsnet" in services:
            if "itsnet" in self.config and self.config["itsnet"]["mib"] != {}:
                start_msg["its_station"]["itsnet"] = self.config["itsnet"]["mib"]

        service_dict = {}
        for service in services:
            if service == "itsnet":
                continue
            if service not in self.config or self.config[service]["mib"] == {}:
                print(
                    "Skipping {} because mib was empty or was not in config".format(
                        service
                    )
                )
                continue
            if self.config[service]["status"] != 1:
                print(
                    "Skipping {} because service was not started in config".format(
                        service
                    )
                )

            if service not in self.service_order:
                print(
                    "Skipping {} because it was not in the service order json".format(
                        service
                    )
                )

            service_dict[self.service_order[service]] = self.config[service]["mib"]

        start_msg["its_station"]["services"] = service_dict
        return json.dumps(start_msg)

    def start_services(self, services: List[str] = None):
        """Starts all the service specified, if not specified, will start services which were started
        in the saved config. Will start in the order specified in the list

        Args:
            services (List[str], optional): List of service to start. Defaults to None.
        """
        if services == None:
            services = [
                srv_name
                for srv_name in self.config
                if self.config[srv_name]["status"] == 1
            ]

        for srv_name in services:
            try:
                self.choir.config_start(srv_name)
            except Exception as e:
                print("{} couldn't be started : {}".format(srv_name, e))

    def config_save_procedure(self, filename: str, services: List[str] = None):
        self.read_mw(services=services)
        self.write_json(filename)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-e", "--env_id", help="env ids to target", required=False, type=int, nargs="+"
    )

    parser.add_argument(
        "--file_ns", help="path/to/file_<env_id>.json", type=str, required=False
    )
    parser.add_argument(
        "--ip", default="127.0.0.1", help="ip of MW", type=str, required=False
    )
    parser.add_argument(
        "--port_base", default=8100, help="port", type=int, required=False
    )
    parser.add_argument(
        "--app_id_base",
        default=1000,
        help="application id to use",
        type=int,
        required=False,
    )
    parser.add_argument(
        "--services", help="services to init", type=str, required=False, nargs="+"
    )
    parser.add_argument(
        "--offline",
        default=True,
        help="wether to do the operations offline of not",
        type=bool,
        required=False,
    )
    parser.add_argument(
        "--stop_services",
        default=True,
        help="wether to temporarily stop services when setting config",
        type=bool,
        required=False,
    )

    # Flags
    parser.add_argument(
        "-r",
        "--read",
        help="flag to gather config and statuses from MW and write it to a file",
        action="store_true",
        required=False,
    )

    parser.add_argument(
        "--start",
        help="flag to start the provided env ids with the provided config",
        action="store_true",
        required=False,
    )

    parser.add_argument(
        "--stop",
        help="flag to stop the provided env ids",
        action="store_true",
        required=False,
    )

    parser.add_argument(
        "--status",
        help="flag to get the status of the services",
        action="store_true",
        required=False,
    )

    args = parser.parse_args()

    # Required parameter check
    if not args.file_ns:
        if args.read or args.start:
            print("Error : file_ns arg not provided")
            return

    if not args.env_id:
        if args.read or args.start or args.stop:
            print("Error : env_id arg not provided")
            return

    services = None
    if args.services:
        services = args.services

    config_manager = MultiConfigManager(
        ip=args.ip,
        port_base=args.port_base,
        app_id_base=args.app_id_base,
        env_ids=args.env_id,
        offline=args.offline,
    )

    if args.read:
        config_manager.gather_config(args.file_ns, services=services)
    if args.stop:
        config_manager.stop_stations()
    if args.start:
        config_manager.start_stations(args.file_ns, services=services)
    if args.status:
        print(config_manager.get_status())


if __name__ == "__main__":
    main()
