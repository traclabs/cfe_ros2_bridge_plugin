import unittest

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cfe_msgs.msg import CFEESHousekeepingTlm

class TelemetryFlowSubscriber(Node):
   def __init__(self):
      super().__init__('telemetry_flow_subscriber')
      self.subscription = self.create_subscription(
         CFEESHousekeepingTlm,
         '/groundsystem/cfe_es_hk_tlm',
         self.listener_callback,
         10)
      self.subscription  # prevent unused variable warning
      self.num_messages_received = 0

   def listener_callback(self, msg):
      self.num_messages_received += 1
      self.get_logger().info('I heard something')

   def get_messages_heard(self):
      return self.num_messages_received

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
      subscriber = TelemetryFlowSubscriber()

      rclpy.spin_once(subscriber, timeout_sec=30)

      self.assertTrue(subscriber.get_messages_heard() > 0, "Didn't receive any subscribed messages.")

      subscriber.destroy_node()

if __name__ == '__main__':
   unittest.main()
