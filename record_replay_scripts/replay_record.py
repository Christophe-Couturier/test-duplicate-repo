#!/usr/bin/python3
import choir
import argparse
from typing import Dict, List
import time
from recorder_utils import launch_simoultaneously

class RecordManager:
    def __init__(self, ip: str, port: int, app_id: int):
        """Class which handles MW record and replay commands

        Args:
            ip (str): ip of the MW
            port (int): port to use
            app_id (int): application id to use.
        """
        self.choir = choir.Choir(ip=ip, port=port, app_id=app_id)

    def init_replay_service(self):
        """Starts the replay service if it is not started
        """
        services = self.choir.config_list_services()
        for entry in services:
            if entry["serviceName"] == "Recorder":
                if self.get_recorder_status() in ["recording", "replaying"]:
                    # Stop recording if it wasn't
                    self.stop_action()
                if entry["status"] == 2:
                    self.choir.config_start("Recorder")
                elif entry["status"] == 3:
                    self.choir.config_restart("Recorder")

    def start_replay(self, filename: str):
        """Start the replay

        Args:
            filename (str): name of the record
        """
        self.choir.config_mib_set(
            "Recorder", {"recorder.action": "replay", "recorder.capturename": filename}, offline=False
        )

    def start_record(self, filename: str):
        """Starts the record

        Args:
            filename (str): name of the record
        """
        self.choir.config_mib_set(
            "Recorder", {"recorder.action": "record", "recorder.capturename": filename}, offline=False
        )

    def stop_action(self):
        """Stops current replay or record
        """
        self.choir.config_mib_set(
            "Recorder", {"recorder.action": "stop"}, offline=False
        )

    def get_recorder_status(self):
        return self.choir.config_mib_get(
            "Recorder", ["recorder.status"], offline=False
        )

class MultiRecordManager:
    def __init__(
        self,
        ip: str,
        port_base: int,
        app_id_base: int,
        env_ids: list,
    ):
        """Class to handle multiple Replay/Record Managers for multiple environment on the same ip (usually YGhost)
        Handles the replay, record, stop in parrallel to avoid delay

        Args:
            ip (str): ip of YGhost
            port_base (int): base port used for the port mapping by YGhost
            app_id_base (int): base app_id to use, will use app_id + env_id for each environent
            env_ids (list): list of env_ids to manage
        """
        self.ip = ip
        self.port_base = port_base
        self.app_id_base = app_id_base
        # Record will not work if they are operated offline
        self.record_managers: Dict[int, RecordManager] = {
            id: RecordManager(ip=ip, port=port_base + id, app_id=app_id_base + id)
            for id in env_ids
        }

    def init_replay_services(self):
        """Inits the replay services of all environments
        """
        for id, manager in self.record_managers.items():
            try:
                manager.init_replay_service()
            except Exception as e:
                print("Error when initializing replay service of env_id {} : {}".format(id, e))


    def start_replay(self, filename: str):
        """Start the replay on all environments
        Uses multithreading and event waiting to make sure the replays are launched simeoultaneously

        Args:
            filename (str): name of the record (each env stores its own record)
        """
        tasks = [
            {
                "name": "env_id {}".format(id),
                "target": manager.start_replay,
                "args": [filename],
                "kwargs": None,
            }
            for id, manager in self.record_managers.items()
        ]
        launch_simoultaneously(tasks, "Error when trying to start replay of ", use_event=True)
                
    def start_record(self, filename: str):
        """Start the record on all environments
        Uses multithreading and event waiting to make sure the record are launched simeoultaneously

        Args:
            filename (str): name of the record (each env stores its own record)
        """
        tasks = [
            {
                "name": "env_id {}".format(id),
                "target": manager.start_record,
                "args": [filename],
                "kwargs": None,
            }
            for id, manager in self.record_managers.items()
        ]
        launch_simoultaneously(tasks, "Error when trying to start record of ", use_event=True)

    def stop_action(self):
        """Stops the record/replay on all environments
        Uses multithreading and event waiting to make sure all is stopped simeoultaneously
        """
        tasks = [
            {
                "name": "env_id {}".format(id),
                "target": manager.stop_action,
                "args": [],
                "kwargs": None,
            }
            for id, manager in self.record_managers.items()
        ]
        launch_simoultaneously(tasks, "Error when trying to stop replay/record of ", use_event=True)

def main():
    parser = argparse.ArgumentParser()
    # Connection infos
    parser.add_argument(
        "-e",
        "--env_id",
        help="env ids to target",
        required=True,
        type=int,
        nargs="+",
    )
    parser.add_argument(
        "--ip", help="IP of YGhost", type=str, required=False, default="127.0.0.1"
    )
    parser.add_argument(
        "--port_base",
        help="port base, env_id will be added to target end_id, ",
        type=int,
        required=False,
        default=8100,
    )
    parser.add_argument(
        "--app_id_base",
        help="app_id base, env_id will be added to target end_id, ",
        type=int,
        required=False,
        default=1000,
    )
    
    # Path
    parser.add_argument(
        "--record_name",
        help="name for record file (file be the same for all env)",
        type=str,
        required=False,
    )
    
    # Flags
    parser.add_argument(
        "--record",
        help="flag to use to start a recording",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--replay",
        help="flag to use to start replaying",
        action="store_true",
        required=False,
    )

    parser.add_argument(
        "--stop",
        help="flag to use to stop current recording/replaying",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--wait",
        help="flag to use if you want to stop recording/replaying after a CTRC+C",
        action="store_true",
        required=False,
    )
    
    
    args = parser.parse_args()
    manager = MultiRecordManager(args.ip, args.port_base, args.app_id_base, args.env_id)
    
    if args.stop:
        manager.stop_action()
        print("Stopped Record/Replay")
            
    if args.record:
        if not args.record_name:
            raise Exception("No recording name provided")
        manager.init_replay_services()

        manager.start_record(filename=args.record_name)
        print("Started Record")
        
        if args.wait:
            try:
                print("Waiting for CTRL+C to stop")
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                manager.stop_action()
                print("Stopped Record")


    elif args.replay:
        if not args.record_name:
            raise Exception("No recording name provided")
            
        manager.init_replay_services()
        
        manager.start_replay(filename=args.record_name)
        print("Started Replay")       

        if args.wait:
            try:
                print("Waiting for CTRL+C to stop")
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                manager.stop_action()
                print("Stopped Replay")


if __name__ == "__main__":
    main()