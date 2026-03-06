import rclpy
from rclpy.node import Node
from .mqtt.MQTT import OldMQTT
import time
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
 
class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')

        self.declare_parameters(
            namespace="",
            parameters=[
                ("mqtt_ip", "localhost"),
                ("mqtt_listener", "manual"),
                ("mqtt_topic", "DriveControl"),
                ("x_axis_max", 4.0),
                ("x_axis_min", -4.0),
                # note: keep z min/max sensible: min negative, max positive (but code handles either order)
                ("z_axis_max", 12.0),
                ("z_axis_min", -12.0)
            ]
        )
        
        # parameters
        self.sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        self.mqtt_ip = self.get_parameter("mqtt_ip").get_parameter_value().string_value
        self.mqtt_listener = self.get_parameter("mqtt_listener").get_parameter_value().string_value
        self.mqtt_topic = self.get_parameter("mqtt_topic").get_parameter_value().string_value
        self.x_max = self.get_parameter("x_axis_max").get_parameter_value().double_value
        self.x_min = self.get_parameter("x_axis_min").get_parameter_value().double_value
        self.z_max = self.get_parameter("z_axis_max").get_parameter_value().double_value
        self.z_min = self.get_parameter("z_axis_min").get_parameter_value().double_value
        
        self.get_logger().info(f"mqtt_ip is set to {self.mqtt_ip}")
        self.get_logger().info(f"mqtt_listener is set to {self.mqtt_listener}")
        self.get_logger().info(f"mqtt_topic is set to {self.mqtt_topic}")
        self.get_logger().info(f"x_axis_max is set to {self.x_max}")
        self.get_logger().info(f"x_axis_min is set to {self.x_min}")
        self.get_logger().info(f"z_axis_max is set to {self.z_max}")
        self.get_logger().info(f"z_axis_min is set to {self.z_min}")
		
        self.mqtt = OldMQTT(self.mqtt_ip)
        self.mqtt.createListener(self.mqtt_listener, self.mqtt_topic)

        self.mqtt_pub = self.create_publisher(
            String,
            f"/app_communicate/manual_control/mqtt/{self.mqtt_topic}",
            10
        )

        self.twist_pub = self.create_publisher(
            Twist,
            "app_twist",
            10
        )

        time.sleep(1)

    def publishMqttMessage(self):
        try:
            msg = String()
            msg.data = self.mqtt.MQTT_Message[self.mqtt_topic]
            self.mqtt_pub.publish(msg)
        except Exception:
            # nothing available yet
            pass
      
    def controls(self):
        """
        Parse the MQTT message safely and publish Twist.
        Expected message format (example): "<something> <something> Drive <x> <y>"
        We check length and types and default to 0.0 on parse errors.
        """
        try:
            raw = self.mqtt.MQTT_Message.get(self.mqtt_topic, "")
            if not raw:
                return

            parts = raw.split()  # splits on whitespace robustly
            # require at least 5 tokens (index 0..4) for cmd,x,y
            if len(parts) < 5:
                self.get_logger().warning(f"MQTT message too short: '{raw}'")
                return

            cmd = parts[2]
            # parse axes with safe defaults
            try:
                x_axis = float(parts[3])
            except Exception as e:
                self.get_logger().warning(f"Failed parsing x_axis '{parts[3]}' -> {e}; defaulting to 0")
                x_axis = 0.0
            try:
                y_axis = float(parts[4])
            except Exception as e:
                self.get_logger().warning(f"Failed parsing y_axis '{parts[4]}' -> {e}; defaulting to 0")
                y_axis = 0.0

            if cmd == 'Drive':
                # use symmetric exponential mapping around zero for joystick-like controls
                mps_x = self.map_range_exp_symmetric(x_axis, -100.0, 100.0, self.x_min, self.x_max)
                mps_y = self.map_range(y_axis, -100.0, 100.0, self.z_min, self.z_max)
                t = Twist()
                t.angular.z = float(mps_y)
                t.linear.x = float(mps_x)
                self.twist_pub.publish(t)
            elif cmd == 'Stop' or cmd == 'Break':
                t = Twist()
                t.angular.z = 0.0
                t.linear.x = 0.0
                self.twist_pub.publish(t)

        except Exception as e:
            # keep exceptions visible in logs rather than silently passing
            self.get_logger().error(f"controls() exception: {e}")

    @staticmethod
    def map_range(x, in_min, in_max, out_min, out_max):
        # linear clamped mapping (keeps original behavior)
        if x >= in_max:
            return out_max
        elif x <= in_min:
            return out_min
        else:
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        
    @staticmethod
    def map_range_exp_symmetric(x, in_min, in_max, out_min, out_max, gamma=1.5):
        """
        Map x from [in_min,in_max] -> [out_min,out_max] using symmetric exponential curve about the midpoint.
        Handles any ordering of out_min/out_max.
        gamma == 1.0 => linear
        gamma > 1 => softer near 0, stronger near extremes
        """
        # Clamp input first
        if x <= in_min:
            return out_min
        if x >= in_max:
            return out_max

        # compute midpoints and normalized -1..1 value
        mid_in = (in_max + in_min) / 2.0
        half_in = (in_max - in_min) / 2.0
        if half_in == 0:
            return (out_min + out_max) / 2.0
        norm = (x - mid_in) / half_in  # -1..1

        # signed exponential
        sign = 1.0 if norm >= 0 else -1.0
        mag = abs(norm)
        mag_exp = mag ** gamma
        norm_exp = sign * mag_exp  # still -1..1

        # map to output range
        mid_out = (out_max + out_min) / 2.0
        half_out = (out_max - out_min) / 2.0
        return mid_out + norm_exp * half_out


def main(args=None):
    rclpy.init(args=args)
    
    manual_control = ManualControl()
    
    rate = manual_control.create_rate(30)
    try:
        while rclpy.ok():
            rclpy.spin_once(manual_control)
            manual_control.publishMqttMessage()  
            manual_control.controls()
    finally:
        manual_control.destroy_node()
        rclpy.shutdown()