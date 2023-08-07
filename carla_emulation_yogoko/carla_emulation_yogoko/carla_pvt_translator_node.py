import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from carla_emulation_yogoko.utility import str_to_bytearray
from yogoko_ros_api_msgs.msg import PubSub
from json import dumps
from math import atan2, sin, cos, degrees, radians, sqrt

class PvtTranslatorNode(Node):
    def __init__(self):
        # Ros part
        super().__init__("pvt_translator")
        self.declare_parameter("vehicle_name", "ego_vehicle")
        self.declare_parameter("sensor_name", "gnss")
        self.declare_parameter("topic", "pvt")
        
        # GPS Position in carla are centered around (0,0), having the possibility to offset the position might be usefull
        self.declare_parameter("lat_offset", 0.0)
        self.declare_parameter("lon_offset", 0.0)
        self.declare_parameter("alt_offset", 0.0)
        
        vehicle_name = self.get_parameter("vehicle_name").get_parameter_value().string_value
        sensor_name = self.get_parameter("sensor_name").get_parameter_value().string_value
        self.in_topic = "/carla/{}/{}".format(vehicle_name, sensor_name)
        
        self.out_topic = self.get_parameter("topic").get_parameter_value().string_value
        
        self.lat_offset = self.get_parameter("lat_offset").get_parameter_value().double_value
        self.lon_offset = self.get_parameter("lon_offset").get_parameter_value().double_value
        self.alt_offset = self.get_parameter("alt_offset").get_parameter_value().double_value

        # Init subscription to carla ros bridge and publisher to yogoko_ros_api
        self.gnss_subscriber = self.create_subscription(NavSatFix, self.in_topic, self.listener_callback, 10)
        self.pvt_publisher = self.create_publisher(PubSub, self.out_topic, 10)

        # stores the last message received  in order to compute derivative related data
        self.last_msg = None

    def listener_callback(self, msg: NavSatFix):
        if not self.last_msg:
            self.last_msg = msg
            return
        
        pvt_msg =  dumps(self.get_pvt_msg(msg, self.last_msg))
        
        pub_msg = PubSub()
        pub_msg.format = pub_msg.MESSAGE_FORMAT_JSON
        pub_msg.data = str_to_bytearray(pvt_msg)
        self.get_logger().info(pvt_msg)
        self.pvt_publisher.publish(pub_msg)
        self.last_msg = msg



    def get_pvt_msg(self, last_msg: NavSatFix, scnd_last_msg: NavSatFix):
        """ create the PVT message corresponding to the two last messages received

        Args:
            last_msg (NavSatFix): last position received
            scnd_last_msg (NavSatFix): second last position received

        Returns:
            dictonnary: {"latitude", "longitude", "altitude", "speed", "heading"}
        """
        speed, heading = self.get_speed_heading(last_msg, scnd_last_msg)
        return {
            "latitude": last_msg.latitude + self.lat_offset, 
            "longitude": last_msg.longitude + self.lon_offset,
            "altitude": last_msg.altitude + self.alt_offset,
            "speed": speed,
            "heading": heading,
        }

    @staticmethod
    def get_speed_heading(last_msg: NavSatFix, scnd_last_msg: NavSatFix):
        """ get the speed and heading obtained from the difference of two messages 

        Args:
            last_msg (NavSatFix): last position received
            scnd_last_msg (NavSatFix): second last position received

        Returns:
            tupple: (speed, heading) 
        """
        R = 6371000
        lat, prev_lat = radians(last_msg.latitude), radians(scnd_last_msg.latitude)
        lon, prev_lon = radians(last_msg.longitude), radians(scnd_last_msg.longitude)

        # speed from haversine distance
        delta_phi = lat - prev_lat
        delta_lambda = lon - prev_lon

        a = sin(delta_phi / 2) * sin(delta_phi / 2) + cos(lat) * cos(prev_lat) * sin(
            delta_lambda / 2
        ) * sin(delta_lambda / 2)
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        d = R * c
        dt_sec = last_msg.header.stamp.sec - scnd_last_msg.header.stamp.sec
        dt_nanosec = last_msg.header.stamp.nanosec - scnd_last_msg.header.stamp.nanosec
        dt = dt_sec + dt_nanosec / 1e9
        v = 0
        if dt > 1e-20:
            v = d / dt

        # heading
        dlon = lon - prev_lon
        X = cos(lat) * sin(dlon)
        Y = cos(prev_lat) * sin(lat) - sin(prev_lat) * cos(lon) * cos(dlon)
        heading = (degrees(atan2(X, Y)) + 360) % 360

        return v, heading


def main(args=None):
    rclpy.init(args=args)
    translator = PvtTranslatorNode()
    try:
        rclpy.spin(translator)
    except:
        translator.destroy_node()

if __name__ == "__main__":
    main()