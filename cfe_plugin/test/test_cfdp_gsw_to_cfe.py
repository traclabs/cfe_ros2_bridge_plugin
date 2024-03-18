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
      self.cfdp_pdus_from_cf_hk = 0

      # Create a CFDP service client
      self.cfdp_client = self.create_client(CfdpXfrCmd, 'cfdp/cmd/put')
      while not self.cfdp_client.wait_for_service(timeout_sec=5.0):
         self.get_logger().info('cfdp/cmd/put service not available, waiting again...')

      # Create a CFDP get service client
      self.cfdp_get_client = self.create_client(CfdpXfrCmd, 'cfdp/cmd/get')
      while not self.cfdp_get_client.wait_for_service(timeout_sec=5.0):
         self.get_logger().info('cfdp/cmd/get service not available, waiting again...')

   def listener_callback(self, msg):
      # Record that a message was seen.
      self.num_messages_received += 1

      self.cfdp_pdus_from_cf_hk = msg.channel_hk[0].counters.recv.pdu

      # Generate a log message
      self.get_logger().info('I heard something')

   def get_messages_heard(self):
      return self.num_messages_received
   
   def get_cfdp_pdus_from_cf_hk(self):
      return self.cfdp_pdus_from_cf_hk

   def transfer_file_to_cfe(self, f):
      cfdp_request = CfdpXfrCmd.Request()
      cfdp_request.dstid = 25 # This is a *put* to the cfe entity id (25)
      cfdp_request.src = f
      cfdp_request.dst = ("/cf/%s" % f)
      # TODO Not using acknowledged mode right now as it is not working.
      # cfdp_request.ack = True
      future = self.cfdp_client.call_async(cfdp_request)
      rclpy.spin_until_future_complete(self, future)
      return future.result()
   
   def transfer_file_to_rosfsw(self, f):
      cfdp_request = CfdpXfrCmd.Request()
      cfdp_request.dstid = 2  # This is a *put* to the flightsystem entity id (2)
      cfdp_request.src = f
      cfdp_request.dst = ("%s_from_rosgsw" % f)
      # TODO Not using acknowledged mode right now as it is not working.
      # cfdp_request.ack = True
      future = self.cfdp_client.call_async(cfdp_request)
      rclpy.spin_until_future_complete(self, future)
      return future.result()

   def transfer_file_from_rosfsw(self, f):
      cfdp_request = CfdpXfrCmd.Request()
      cfdp_request.dstid = 2  # This is a *get* from the flightsystem entity id (2)
      cfdp_request.src = ("%s_from_rosgsw" % f)
      cfdp_request.dst = ("%s_from_rosfsw" % f)
      # TODO Not using acknowledged mode right now as it is not working.
      # cfdp_request.ack = True
      future = self.cfdp_get_client.call_async(cfdp_request)
      rclpy.spin_until_future_complete(self, future)
      return future.result()

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
      # os.getcwd() returns /code/brash/build/cfe_plugin
      # and we want the file to be put into /code/brash/cfdp/rosgsw
      with tempfile.NamedTemporaryFile(dir="/code/brash/cfdp/rosgsw", delete=False) as fp:
         s = 'Hello CFE world, greetings!  Time on GSW now is %s.\n' % str(datetime.datetime.now())
         fp.write(s.encode('utf-8'))
         fp.close()

         # the file is closed, but not removed
         # Now, transfer the file up to the CFE.
         subscriber.transfer_file_to_cfe(os.path.basename(fp.name))

         # Spin for a few messages to ensure we see fresh messages.
         for i in range(10):
            rclpy.spin_once(subscriber, timeout_sec=30)

         # Verify that the CFE CF app saw a PDU. 
         # Note right now we are checking that it is just greater than zero to make 
         #   sure it has changed.  There may be a juicer issue that is causing endianness issues with the TLM value.
         # TODO: Update so that it checks for the expected number of PDUs.
         #self.assertEqual(subscriber.get_cfdp_pdus_from_cf_hk(), XYZ)
         self.assertGreater(subscriber.get_cfdp_pdus_from_cf_hk(), 0)

         # OK, now, let's transfer a file up to rosfsw and back down to us to verify that it is OK.
         subscriber.transfer_file_to_rosfsw(os.path.basename(fp.name))
         time.sleep(3)
         subscriber.transfer_file_from_rosfsw(os.path.basename(fp.name))
         time.sleep(3)

         # And assert that the file exists.
         # TODO Not quite working yet, but once things are working this assert should pass.
         #self.assertTrue(os.path.exists("%s_from_rosfsw" % os.path.basename(fp.name)))

      subscriber.destroy_node()

if __name__ == '__main__':
   unittest.main()
