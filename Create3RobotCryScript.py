import os
import requests
import time
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration

# 🔹 Secure Airtable credentials
API_KEY = "patOjO3jX7WMStKMz.1ef04f7d1a20500ea3be207219600830a0feef095407863e4f51a926ed65fb2c"
BASE_ID = "appnDZneIEGsXZQhQ"
TABLE_NAME = "Table 1"
URL = f"https://api.airtable.com/v0/{BASE_ID}/{TABLE_NAME}?maxRecords=1"

# Authorization Header
headers = {"Authorization": f"Bearer {API_KEY}"}


class AirtableBattleCry(Node):
    def __init__(self):
        super().__init__('airtable_battle_cry')
        self.publisher_cmd_audio = self.create_publisher(AudioNoteVector, '/cmd_audio', 10)
        self.timer = self.create_timer(2.0, self.check_airtable)  # Check Airtable every 2 sec
        self.get_logger().info("✅ Airtable Battle Cry Node Started!")

    def check_airtable(self):
        """Checks Airtable for the lateost command and triggers the battle cry if needed."""
        self.get_logger().info("🔍 Checking Airtable for new commands...")

        try:
            response = requests.get(URL, headers=headers)
            response.raise_for_status()
            data = response.json()

            # Debug: Print Airtable response
            self.get_logger().info(f"🌐 Airtable Response: {data}")

            if "records" in data and data["records"]:
                record = data["records"][0]["fields"]
                command = record.get("Command", "").strip().upper()  # Normalize case

                self.get_logger().info(f"📢 Command Received: {command}")

                if command == "CRY":
                    self.play_battle_cry()
                else:
                    self.get_logger().info("⚠️ No valid command detected.")

            else:
                self.get_logger().info("⚠️ No records found in Airtable.")

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"❌ API request failed: {e}")

    def play_battle_cry(self):
        """Plays the first two seconds of the Super Mario theme accurately."""
        self.get_logger().info("🎵 Playing Mario Theme!")

        notes = [
            (659, 150_000_000),  # E5 (150ms)
            (659, 150_000_000),  # E5 (150ms)
            (0,   100_000_000),  # Rest (100ms)
            (659, 150_000_000),  # E5 (150ms)
            (0,   100_000_000),  # Rest (100ms)
            (523, 150_000_000),  # C5 (150ms)
            (659, 150_000_000),  # E5 (150ms)
            (784, 300_000_000),  # G5 (300ms)
            (0,   300_000_000),  # Rest (300ms)
            (392, 300_000_000)   # G4 (300ms)
        ]

        for frequency, nanoseconds in notes:
            msg = AudioNoteVector()
            msg.notes = [AudioNote(frequency=frequency, max_runtime=Duration(sec=0, nanosec=nanoseconds))]

            self.publisher_cmd_audio.publish(msg)
            self.get_logger().info(f"🔊 Playing {frequency} Hz for {nanoseconds / 1_000_000} ms")

            time.sleep(nanoseconds / 1_000_000_000 + 0.05)  # Convert ns to sec & add slight buffer


def main():
    """Starts the Airtable Battle Cry node."""
    print("🟢 Starting AirtableBattleCry Node...")
    rclpy.init()
    node = AirtableBattleCry()  # ✅ Create an instance of the class
    print("🔄 Node initialized. Spinning...")
    rclpy.spin(node)  # ✅ Keeps the node running
    print("🛑 Node shutdown.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

