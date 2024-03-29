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
      self.set_met_publisher = self.create_publisher(
         CFETIMETimeCmd,
         '/groundsystem/cfe_time_set_met_cmd',
         10)

      # Create a publisher so that we can send the Set Spacecraft Time Correlation Factor command
      self.set_stcf_publisher = self.create_publisher(
         CFETIMETimeCmd,
         '/groundsystem/cfe_time_set_stcf_cmd',
         10)

   # Function that sends a CFE Set STCF (Spacecraft Time Correlation Factor) message that sets the SCTF to the desired value.
   def publish_set_stcf_cmd(self, stcf):
      msg = CFETIMETimeCmd()
      msg.payload.seconds = swap32(stcf)
      msg.payload.micro_seconds = 0
      self.set_stcf_publisher.publish(msg)

   # Function that sends a CFE Set MET command using time_seconds as the desired seconds value.
   def publish_set_met_cmd(self, time_seconds):
      msg = CFETIMETimeCmd()
      msg.payload.seconds = swap32(time_seconds)
      msg.payload.micro_seconds = 0
      self.set_met_publisher.publish(msg)

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

      # Send the Set STCF (Spacecraft Time Correlation Factor) function.  This ensures that the Spacecraft Time is identical to the MET.
      subscriber.publish_set_stcf_cmd(0)

      time.sleep(3)

      # Send the Set MET command.
      time_seconds = 200000
      subscriber.publish_set_met_cmd(time_seconds)

      time.sleep(3)

      # Spin for a few messages to ensure we see fresh messages.
      for i in range(10):
         rclpy.spin_once(subscriber, timeout_sec=30)

      # Assert that we've heard a message and that the time in the secondary header corresponds to the new MET value.
      self.assertGreater(subscriber.get_messages_heard(), 0)
      self.assertGreaterEqual(subscriber.get_last_cfe_met_sec_received(), time_seconds)
      self.assertLess(subscriber.get_last_cfe_met_sec_received(), time_seconds + 120)

      subscriber.destroy_node()

if __name__ == '__main__':
   unittest.main()
