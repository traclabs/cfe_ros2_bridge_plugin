import os
import unittest
import time
import tempfile
import datetime

import rclpy
from rclpy.node import Node

from cfe_msgs.msg import CFHkPacket
from cfdp_msgs.srv import CfdpXfrCmd

class ParticipantNode(Node):
   def __init__(self):
      super().__init__('cfe_cfdp_subscriber')

      # Create a subscriber that allows us to receive CF housekeeping telemetry
      self.subscription = self.create_subscription(
         CFHkPacket,
         '/groundsystem/cfdp_hk_tlm',
         self.listener_callback,
         10)
      self.subscription  # prevent unused variable warning
      self.num_messages_received = 0

      # Create a CFDP service client
      self.cfdp_client = self.create_client(CfdpXfrCmd, 'cfdp/cmd/put')
      while not self.cfdp_client.wait_for_service(timeout_sec=5.0):
         self.get_logger().info('service not available, waiting again...')
      self.cfdp_request = CfdpXfrCmd.Request()

   def listener_callback(self, msg):
      # Record that a message was seen.
      self.num_messages_received += 1

      # Generate a log message
      self.get_logger().info('I heard something')

   def get_messages_heard(self):
      return self.num_messages_received
   
   def transfer_file_to_cfe(self, f):
      # "ros2 service call /cfdp/cmd/put cfdp_msgs/srv/CfdpXfrCmd '{\"src\": \"gswtest.txt\", \"dst\": \"/cf/gswtestdst724c.txt\", \"dstid\":25}'"
      self.cfdp_request.dstid = 25
      self.cfdp_request.src = f
      self.cfdp_request.dst = ("/cf/%s" % os.path.basename(f))
      self.future = self.cfdp_client.call_async(self.cfdp_request)
      rclpy.spin_until_future_complete(self, self.future)
      return self.future.result()

class TestGSWCFECFDPFlow(unittest.TestCase):
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

      # Spin for a few messages to ensure we see fresh messages.
      for i in range(10):
         rclpy.spin_once(subscriber, timeout_sec=30)

      # Assert that we've heard a message.
      self.assertGreater(subscriber.get_messages_heard(), 0)

      # Now, create a local temporary file.
      with tempfile.NamedTemporaryFile(delete=False) as fp:
         s = 'Hello CFE world, greetings!  Time on GSW now is %s.' % str(datetime.datetime.now())
         fp.write(s.encode('utf-8'))
         fp.close()

         # the file is closed, but not removed
         # Now, transfer the file up to the CFE.
         subscriber.transfer_file_to_cfe(fp.name)

         time.sleep(3)

      subscriber.destroy_node()

if __name__ == '__main__':
   unittest.main()
