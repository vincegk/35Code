import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests
import time

# Airtable API credentials
AIRTABLE_API_KEY = "patBbpDlEyRz6x61D.808a9bc80a3049deb00565ac16ce50052b1ddea7eb97f5e60ba91ac5ea9f2722"
AIRTABLE_BASE_ID = "appoPPSGGY0Bgjwle"
AIRTABLE_TABLE_NAME = "Bot"
AIRTABLE_URL = f"https://api.airtable.com/v0/{AIRTABLE_BASE_ID}/{AIRTABLE_TABLE_NAME}"
HEADERS = {"Authorization": f"Bearer {AIRTABLE_API_KEY}"}

# WASD movement mapping
WASD_MAPPING = {
    "w": {"linear": {"x": 0.2, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},  # Forward
    "s": {"linear": {"x": -0.2, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}, # Backward
    "a": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.5}},  # Rotate Left
    "d": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -0.5}}, # Rotate Right
    "stop": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}} # Stop
}

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def fetch_airtable_command(self):
        """Fetch the current movement mode from Airtable."""
        try:
            response = requests.get(AIRTABLE_URL, headers=HEADERS)
            if response.status_code == 200:
                data = response.json()
                if "records" in data and data["records"]:
                    for record in data["records"]:
                        fields = record["fields"]
                        if "Mode" in fields:
                            return fields["Mode"].lower()  # Read the movement mode
            else:
                self.get_logger().info(f"Airtable request failed: {response.status_code}")
        except Exception as e:
            self.get_logger().info(f"Airtable API error: {e}")
        return "stop"

    def execute_movement(self, mode):
        """Move the robot based on Airtable 'Mode' value."""
        velocity = Twist()

        if mode in WASD_MAPPING:
            self.get_logger().info(f"Executing movement: {mode.upper()}")
            linear_values = WASD_MAPPING[mode]["linear"]
            angular_values = WASD_MAPPING[mode]["angular"]
        else:
            self.get_logger().info(f"Invalid mode: {mode}. Stopping.")
            linear_values = WASD_MAPPING["stop"]["linear"]
            angular_values = WASD_MAPPING["stop"]["angular"]

        velocity.linear.x = linear_values["x"]
        velocity.linear.y = linear_values["y"]
        velocity.linear.z = linear_values["z"]

        velocity.angular.x = angular_values["x"]
        velocity.angular.y = angular_values["y"]
        velocity.angular.z = angular_values["z"]

        self.publisher.publish(velocity)
        self.get_logger().info(f"Published: Linear={velocity.linear}, Angular={velocity.angular}")

def main():
    rclpy.init()
    node = MoveRobot()

    try:
        while rclpy.ok():
            mode = node.fetch_airtable_command()
            node.execute_movement(mode)
            time.sleep(0.01)  # Polling interval
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
