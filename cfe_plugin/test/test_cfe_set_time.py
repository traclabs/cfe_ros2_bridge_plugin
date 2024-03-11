import unittest
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cfe_msgs.msg import CFEESHousekeepingTlm
from cfe_msgs.msg import CFETIMETimeCmd

# https://stackoverflow.com/a/27506692
def swap32(i):
    return struct.unpack("<I", struct.pack(">I", i))[0]

class ParticipantNode(Node):
   def __init__(self):
      super().__init__('cfe_time_test_subscriber')

      # Create a subscriber that allows us to receive ES housekeeping telemetry
      self.subscription = self.create_subscription(
         CFEESHousekeepingTlm,
         '/groundsystem/cfe_es_hk_tlm',
         self.listener_callback,
         10)
      self.subscription  # prevent unused variable warning
      self.num_messages_received = 0
      self.last_cfe_met_sec_received = 0

      # Create a publisher so that we can send the SET MET command
      self.publisher = self.create_publisher(
         CFETIMETimeCmd,
         '/groundsystem/cfe_time_set_met_cmd',
         10)

   # Function that sends a CFE Set MET command using time_seconds as the desired seconds value.
   def publish_set_met_cmd(self, time_seconds):
      msg = CFETIMETimeCmd()
      msg.payload.seconds = swap32(time_seconds)
      msg.payload.micro_seconds = 0
      self.publisher.publish(msg)

   def listener_callback(self, msg):
      # Record that a message was seen.
      self.num_messages_received += 1

      # Extract time from ES housekeeping packet secondary header.
      (met_seconds, met_subseconds) = struct.unpack(">IH", msg.telemetry_header.sec.time)
      self.last_cfe_met_sec_received = met_seconds

      # Generate a log message
      self.get_logger().info('I heard something with met %d' % met_seconds)

   def get_messages_heard(self):
      return self.num_messages_received
   
   def get_last_cfe_met_sec_received(self):
      return self.last_cfe_met_sec_received

class TestTelemetryFlow(unittest.TestCase):
   @classmethod
   def setUpClass(cls):
      # Initialize the ROS context for the test node
      rclpy.init()

   @classmethod
   def tearDownClass(cls):
      # Shutdown the ROS context
      rclpy.shutdown()

   def test_flow(self):
      subscriber = ParticipantNode()

      time.sleep(3)

      # Send the Set MET command.
      time_seconds = 200000
      subscriber.publish_set_met_cmd(time_seconds)

      time.sleep(3)

      # Wait for a message response.
      rclpy.spin_once(subscriber, timeout_sec=30)

      # Assert that we've heard a message and that the time in the secondary header corresponds to the new MET value.
      self.assertTrue(subscriber.get_messages_heard() > 0, "Didn't receive any subscribed messages.")
      self.assertTrue(subscriber.get_last_cfe_met_sec_received() >= time_seconds, "Time didn't change as expected.")

      subscriber.destroy_node()

if __name__ == '__main__':
   unittest.main()
