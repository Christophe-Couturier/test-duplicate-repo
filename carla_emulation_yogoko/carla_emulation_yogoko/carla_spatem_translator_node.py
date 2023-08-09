import rclpy
from rclpy.node import Node
from carla_emulation_yogoko.utility import str_to_bytearray
from yogoko_ros_api_msgs.msg import PubSub
from diagnostic_msgs.msg import KeyValue
from carla_msgs.msg import CarlaTrafficLightStatusList, CarlaTrafficLightInfoList
from carla_msgs.srv import SpawnObject, DestroyObject
from json import dumps, load
import datetime

def get_TimeMark(utc_time: int, now: float):
    """get a TimeMark from utc_time in 1/10s and the current timestamp. Note: It does not process
    leap seconds, meaning that if a leap seconds occures (they happend once every year) between current time and utc_time input, the
    format will not be respected and the timemark will be slightly off.

    Args:
        time (int): utc time in 1/10seconds
        now (float): current utc timestamp

    Returns:
        int: TimeMark corresponding to the input time, 
    """
    # if utc_time is in more than one hour
    if utc_time - now * 10 >= 36_000:
        return 36_000
    # We ceil the output time to 36_990, 35_991 and 35_999 are reserved when a leap second occures
    if 35_999 >= utc_time % 36_000 >= 35_991:
        return 35_990
    return utc_time % 36_000
    
    


class SPATEMTranslatorNode(Node):
    def __init__(self):
        # Ros part
        super().__init__("spatem_translator")
        self.declare_parameter("sensor_name", "spatem_sensor")
        self.declare_parameter("topic", "spatem")
        self.declare_parameter("period", 1000)  # ms
        self.declare_parameter("protocol_version", 1)
        self.declare_parameter("station_id", 10000)
        self.declare_parameter("config", "")
        self.declare_parameter("spawn_sensor", True)

        # path to config
        # JSON config of the trafficlight:
        # <id of traffic light in carla>: {
        #   intersectionID: SPAT's intersectionID, optionnal, default 1
        #   intersectionRegion: SPAT's intersection, optionnal, default 1
        #   intersectionName: SPAT's intersection name, optionnal, default str(intersectionID)
        #   signalGroup : SPAT's signalGroup
        # }

        sensor_name = (
            self.get_parameter("sensor_name").get_parameter_value().string_value
        )

        self.status_in_topic = "/carla/{}/status".format(sensor_name)
        self.info_in_topic = "/carla/{}/info".format(sensor_name)
        self.out_topic = self.get_parameter("topic").get_parameter_value().string_value
        
        self.spawned_sensor = False
        self.sensor_id = 0

        file = None
        try:
            file = open(
                self.get_parameter("config").get_parameter_value().string_value, "r"
            )
        except Exception as e:
            self.get_logger().error("Exception occured when trying to open config file: {}".format(e))
            return
        try:
            self.config: dict = load(file)
            # Inits default value for config
            for _, entry in self.config.items():
                if "intersectionID" not in entry:
                    entry["intersectionID"] = 1
                if "intersectionRegion" not in entry:
                    entry["intersectionRegion"] = 1
                if "intersectionName" not in entry:
                    entry["intersectionName"] = str(entry["intersectionID"])
            file.close()
        except Exception as e:
            file.close()
            self.get_logger().error("Exception occured when trying to read config file: {}".format(e))
            return

        # Init sensor if requested
        if self.get_parameter("spawn_sensor").get_parameter_value().bool_value:
            cli = self.create_client(SpawnObject, "carla/spawn_object")
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("carla-ros-bridge not available, waiting again...")

            req = SpawnObject.Request()
            req.type = "sensor.pseudo.traffic_lights"
            req.id = "spatem_sensor"
            attribute = KeyValue()
            attribute.key = "sensor_tick"
            attribute.value = "0.1"
            req.attributes.append(attribute)
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.sensor_id = future.result().id
            self.spawned_sensor = True

        # Init subscription to carla ros bridge and publisher to yogoko_ros_api
        self.status_subscriber = self.create_subscription(
            CarlaTrafficLightStatusList,
            self.status_in_topic,
            self.listener_callback,
            10,
        )
        self.info_subscriber = self.create_subscription(
            CarlaTrafficLightInfoList, self.info_in_topic, self.update_durations, 10
        )
        self.spatem_publisher = self.create_publisher(PubSub, self.out_topic, 10)

        # Init timer to send spatem
        self.timer = self.create_timer(
            self.get_parameter("period").get_parameter_value().integer_value / 1000,
            self.timer_callback,
        )

        self.last_msg: CarlaTrafficLightStatusList = None
        self.scnd_last_msg: CarlaTrafficLightStatusList = None

        # Static SPATEM msg fields init :
        self.its_header = {
            "protocolVersion": self.get_parameter("protocol_version")
            .get_parameter_value()
            .integer_value,
            "messageID": 4,  # ID of spatem msg
            "stationID": self.get_parameter("station_id")
            .get_parameter_value()
            .integer_value,
        }

        # carla state map to spatem state
        self.spatem_state_map = {
            0: "stop-And-Remain",
            1: "permissive-clearance",
            2: "protected-Movement-Allowed",
            3: "caution-Conflicting-Traffic",
            4: "unavaillable",
        }

        # State to color
        self.color_state_map = {0: "red", 1: "yellow", 2: "green"}

        self.state_durations = {}  # {id: {state: duration}} 1/10s

        self.state_change_time = {}

        self.state_order = [2, 1, 0]

    def close(self):
        if self.spawned_sensor:
            cli = self.create_client(DestroyObject, "carla/destroy_object")
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("ROS Bridge not available, waiting again...")

            req = DestroyObject.Request()
            req.id = self.sensor_id
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

    def listener_callback(self, msg: CarlaTrafficLightStatusList):
        self.scnd_last_msg = self.last_msg
        self.last_msg = msg
        if self.scnd_last_msg != None and self.last_msg != None:
            self.update_state_change()

    def timer_callback(self):
        """Sends the SPATEM message computed from the last CarlaTrafficLightStatusList received to the topic"""
        if self.last_msg == None:
            return

        # We remove spaces because the MW does not process msg if it has spaces
        spatem_msg = dumps(self.get_spatem_msg(self.last_msg))
        pub_msg = PubSub()
        pub_msg.format = pub_msg.MESSAGE_FORMAT_JSON
        pub_msg.data = str_to_bytearray(spatem_msg)
        self.spatem_publisher.publish(pub_msg)

    def update_state_change(self):
        """Updates the time of the last traffic light state changed if it has"""
        for current_traffic_light in self.last_msg.traffic_lights:
            if str(current_traffic_light.id) not in self.config:
                # Skipping the traffic light we don't monitor
                continue

            for traffic_light in self.scnd_last_msg.traffic_lights:
                if current_traffic_light.id != traffic_light.id:
                    continue
                if current_traffic_light.state != traffic_light.state:
                    self.state_change_time[current_traffic_light.id] = int(datetime.datetime.utcnow().timestamp() * 10)
                break

    def update_durations(self, msg: CarlaTrafficLightInfoList):
        """Updates the state duration time with the received CarlaTrafficLightInfoList
        Time are in 10⁻1s
        """
        for traffic_light in msg.traffic_lights:
            if str(traffic_light.id) not in self.config:
                continue
            state_times = {
                0: int(10 * traffic_light.red_time),
                1: int(10 * traffic_light.yellow_time),
                2: int(10 * traffic_light.green_time),
            }
            self.state_durations[traffic_light.id] = state_times

    def get_change_time(self, id: int, current_state: int, state_change_time: int):
        """Gets a dictionnary with for each state in the next cycle, its start_time and end_time (in ms)

        Args:
            id (int): id of the trafficlight
            current_state (int): current state
            state_change_time (int): timestamp of the last state change (in millisecond)

        Returns:
            dict: dict{state (int): {start: (int), end: (int)}}
        """
        # if the trafficlight is off or unknown
        if current_state not in self.color_state_map:
            return None
        times = {}

        current_state_index = self.state_order.index(current_state)
        last_change_time = state_change_time

        for k in range(len(self.state_order)):  # iterating through all other states
            state = self.state_order[(current_state_index + k) % len(self.state_order)]
            # if we have no information on this state duration, we just put starting time and end here
            if id not in self.state_durations:
                times[state] = {"start": last_change_time}
                break
            else:
                times[state] = {
                    "start": last_change_time,
                    "end": last_change_time + self.state_durations[id][state],
                }
                last_change_time += self.state_durations[id][state]

        return times

    @staticmethod
    def get_intersection_hash(conf_entry: dict) -> str:
        """Returns a minimal "hash" of an intersection to check if two intersections are equal

        Args:
            conf_entry (dict): Dict containing intersectionID and intersectionRegion

        Returns:
            str: hashed intersection name
        """
        inter_hash = str(conf_entry["intersectionID"])
        inter_hash += "_" + str(conf_entry["intersectionRegion"])

        return inter_hash

    def get_spatem_msg(self, msg: CarlaTrafficLightStatusList) -> dict:
        """Create a SPATEM message from a CarlaTrafficLightStatusList ROS message

        Args:
            msg (CarlaTrafficLightStatusList): ROS Message

        Returns:
            dict: SPATEM message
        """

        spatem = {"header": self.its_header}
        spat = {"intersections": []}

        # Computing SPAT timestamp and moy
        now = datetime.datetime.utcnow()
        moy = int((now - datetime.datetime(now.year, 1, 1)).total_seconds() // 60)
        timestamp = int(
            ((now - datetime.datetime(now.year, 1, 1)).total_seconds() % 60) * 1_000
        )

        visited_intersection = {}  # intersection "hash": intersection field

        for status in msg.traffic_lights:
            if str(status.id) not in self.config:
                continue

            conf_entry = self.config[str(status.id)]
            inter_hash = self.get_intersection_hash(conf_entry)

            # Configuring intersection field header
            if inter_hash not in visited_intersection:
                intersection = {}
                intersection["status"] = "0000"
                intersection["revision"] = 1
                intersection["timeStamp"] = timestamp
                intersection["moy"] = moy
                intersection["id"] = {
                    "id": conf_entry["intersectionID"],
                    "region": conf_entry["intersectionRegion"],
                }
                intersection["name"] = conf_entry["intersectionName"]
                intersection["states"] = []
                visited_intersection[inter_hash] = intersection

            intersection = visited_intersection[inter_hash]

            # Creating movement field (= a traffic light) to intersection
            movement = {"signalGroup": conf_entry["signalGroup"]}
            if status.id not in self.state_change_time:
                timings = None
            else:
                timings = self.get_change_time(
                    status.id, status.state, self.state_change_time[status.id]
                )
            time_speed = []
            if timings == None:
                time_speed.append({"eventState": self.spatem_state_map[status.state]})
            else:
                for timing_state, timing in timings.items():
                    state_time = {"eventState": self.spatem_state_map[timing_state]}
                    if "end" in timing:
                        state_time["timing"] = {}
                        state_time["timing"]["startTime"] = get_TimeMark(timing["start"], now.timestamp())
                        state_time["timing"]["minEndTime"] = get_TimeMark(timing["end"], now.timestamp())

                    time_speed.append(state_time)

            movement["state-time-speed"] = time_speed
            intersection["states"].append(movement)

        # Completing the SPAT and SPATEM
        for _, intersection in visited_intersection.items():
            spat["intersections"].append(intersection)

        spatem["spat"] = spat
        return spatem


def main(args=None):
    rclpy.init(args=args)
    translator = SPATEMTranslatorNode()
    try:
        rclpy.spin(translator)
    except BaseException as e:
        translator.get_logger().error(str(e))
        translator.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()