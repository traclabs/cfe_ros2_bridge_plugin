# This file implements CFDP Tests on rosgsw with fsw and rosfsw systems.
#  The default configuration assumes the standard directory structure for Docker-based testing.
#  Additional tweaks may be needed to run outside of Docker and/or on systems without shared folders.
#
# Developer Debug Options:
# - To run tests with verbosity: ` PYTEST_ADDOPTS="--log-cli-level=INFO -v --no-header --full-trace" colcon test --event-handlers console_cohesion+ --ctest-args " -VVV" --return-code-on-test-failure --packages-select cfe_plugi`
#  - Use @unittest.skip() to skip tests for debug purposes
#  - All test files are automatically generated and deleted upon test conclusion. To suppress file deletion toggle the debug_disable_cleanup line in setupClass
#  - For temporary debug output in test driver use "self.subscriber.get_logger().info(...)
# 

import os
import unittest
import time
import tempfile
import datetime
import pathlib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


from cfe_msgs.msg import CFHkPacket, CFTxFileCmd
from cfdp_msgs.srv import CfdpXfrCmd

class ParticipantNode(Node):
   def __init__(self):
      super().__init__('cfe_cfdp_subscriber')

      # Create a subscriber that allows us to receive CF housekeeping telemetry
      self.subscription = self.create_subscription(
         CFHkPacket,
         '/groundsystem/cfdp_hk_tlm',
         self.listener_callback,
         QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
         ))

      self.subscription  # prevent unused variable warning
      
      self.reset_test()

      # Create a CFDP service client
      self.cfdp_client = self.create_client(CfdpXfrCmd, '/groundsystem/cfdp/cmd/put')
      while not self.cfdp_client.wait_for_service(timeout_sec=5.0):
         self.get_logger().info('cfdp/cmd/put service not available, waiting again...')

      # Create a CFDP get service client
      self.cfdp_get_client = self.create_client(CfdpXfrCmd, '/groundsystem/cfdp/cmd/get')
      while not self.cfdp_get_client.wait_for_service(timeout_sec=5.0):
         self.get_logger().info('cfdp/cmd/get service not available, waiting again...')

      self.cf_tx_cmd_sender = self.create_publisher( CFTxFileCmd, "/groundsystem/cfdp_tx_cmd", 10)

   def reset_test(self):
      self.num_messages_received = 0
      self.cfdp_pdus_from_cf_hk = 0
      

   def listener_callback(self, msg):
      # Record that a message was seen.
      self.num_messages_received += 1

      self.fsw_cf_hk = msg
      
      self.cfdp_pdus_from_cf_hk = msg.channel_hk[0].counters.recv.pdu

      # Generate a log message
      self.get_logger().info(f'I heard some cfdp pdu traffic. rx pdus={self.cfdp_pdus_from_cf_hk}')

   def get_messages_heard(self):
      return self.num_messages_received
   
   def get_cfdp_pdus_from_cf_hk(self):
      return self.cfdp_pdus_from_cf_hk
   def get_fsw_cfdp_rcv_bytes(self):
      return self.fsw_cf_hk.channel_hk[0].counters.recv.file_data_bytes

   def transfer_req(self, cfdp_request):
      future = self.cfdp_client.call_async(cfdp_request)
      rclpy.spin_until_future_complete(self, future)
      return future.result()
   
   def transfer_get_req(self, cfdp_request):
      future = self.cfdp_get_client.call_async(cfdp_request)
      rclpy.spin_until_future_complete(self, future)
      return future.result()
      

class TestGSWCFECFDPFlow(unittest.TestCase):
   @classmethod
   def setUpClass(cls):
      cls.debug_disable_cleanup = False # Set to True For DEBUG Use Only. TODO: Make this toggleable from ENV variable
      
      # Initialize the ROS context for the test node
      rclpy.init()

      cls.subscriber = ParticipantNode()

      # Retrieve a message to validate system state
      rclpy.spin_once(cls.subscriber, timeout_sec=30)

      if cls.subscriber.get_messages_heard() != 1:
         raise Exception("Subscription verification failed, unable to begin test sequence")

      # Prepare test parameters for temporary files
      # os.getcwd() returns {brash_workspace}/build/cfe_plugin
      # and we want the file to be put into {brash_workspace}/cfdp/rosgsw
      full_path = pathlib.Path(os.getcwd())
      cls.dir_rosgsw = str(full_path.parents[1]) + "/cfdp/rosgsw"
      cls.dir_rosfsw = str(full_path.parents[1]) + "/cfdp/rosfsw"
      cls.dir_fsw    = str(full_path.parents[1]) + "/../cFS/build/exe/cpu2/cf" # TODO: ENV variable to override if needed
      
      # Create cfdp/rosgsw and cfdp/rosfsw if they do not exist yet
      pathlib.Path(cls.dir_rosgsw).mkdir(parents=True, exist_ok=True)
      pathlib.Path(cls.dir_rosfsw).mkdir(parents=True, exist_ok=True)

      if not os.path.exists(cls.dir_fsw):
         #cls.subscriber.get_logger().info(f"NOT EXISTS: path {cls.dir_fsw}")
         # WARNING: Some tests may need to be skipped or altered in this case
         cls.dir_fsw = None

   @classmethod
   def tearDownClass(cls):
      cls.subscriber.destroy_node()
      
      # Shutdown the ROS context
      rclpy.shutdown()


   def setUp(self):
      self.subscriber.reset_test()
      
      rclpy.spin_once(self.subscriber, timeout_sec=30)
      self.base_pdu_cnt = self.subscriber.get_cfdp_pdus_from_cf_hk()
      self.base_fsw_cfdp_rcv_bytes = self.subscriber.get_fsw_cfdp_rcv_bytes()

   def assertFilesEqual(self, file1, file2):
      """Helper method to assert that two binary files are equal."""
      # This function courtesy of ChatGPT
        
      with open(file1, 'rb') as f1, open(file2, 'rb') as f2:
         content1 = f1.read()
         content2 = f2.read()
         self.assertEqual(content1, content2, f"Files {file1} and {file2} are not equal")

   def test_trivial_file_to_cfe_noack(self):
      self.do_file_xfr(self.transfer_file_to_cfe,
                       ack=False, cfe_expected_pdu_cnt=3)

   def test_trivial_file_to_cfe_ack(self):
      self.do_file_xfr(self.transfer_file_to_cfe,
                       ack=True, file_size=32, cfe_check_file_size=True)

   def test_64k_file_to_cfe_noack(self):
      self.do_file_xfr(self.transfer_file_to_cfe,
                       ack=False, file_size=65536,
                        cfe_check_file_size=True)
      
   def test_64k_file_to_cfe_ack(self):
      self.do_file_xfr(self.transfer_file_to_cfe,
                       ack=True, file_size=65536,
                       cfe_check_file_size=True)

   def test_trivial_file_to_rosfsw_noack(self):
      self.do_file_xfr(self.transfer_file_to_rosfsw,
                       ack=False,
                       check_file_exists=True)

   def test_trivial_file_to_rosfsw_ack(self):
      self.do_file_xfr(self.transfer_file_to_rosfsw,
                       ack=True,
                       check_file_exists=True)

   def test_64k_file_to_rosfsw_noack(self):
      self.do_file_xfr(self.transfer_file_to_rosfsw, 
                       ack=False, file_size=65536,
                       check_file_exists=True)

   def test_64k_file_to_rosfsw_ack(self):
      self.do_file_xfr(self.transfer_file_to_rosfsw,
                       ack=True, file_size=65536,
                       check_file_exists=True)

   def test_trivial_file_from_rosfsw_noack(self):
      self.do_file_xfr(self.transfer_file_from_rosfsw,
                       ack=False,
                       check_file_exists=True, src_base=self.dir_rosfsw)

   def test_trivial_file_from_rosfsw_ack(self):
      self.do_file_xfr(self.transfer_file_from_rosfsw,
                       ack=True,
                       check_file_exists=True, src_base=self.dir_rosfsw)
      
   def test_64k_file_from_rosfsw_noack(self):
      self.do_file_xfr(self.transfer_file_from_rosfsw, 
                       ack=False, file_size=65536,
                       check_file_exists=True, src_base=self.dir_rosfsw)

   def test_64k_file_from_rosfsw_ack(self):
      self.do_file_xfr(self.transfer_file_from_rosfsw,
                       ack=True, file_size=65536,
                       check_file_exists=True, src_base=self.dir_rosfsw)

   @unittest.skip(reason="FIXME: Juicer bug with fixed-length strings interferes with sending cmd")
   def test_trivial_file_from_fsw_noack(self):
      self.do_file_xfr(self.transfer_file_from_cfe,
                       ack=False,
                       check_file_exists=True, src_base=self.dir_fsw)
   @unittest.skip(reason="FIXME: Juicer bug with fixed-length strings interferes with sending cmd")
   def test_trivial_file_from_fsw_ack(self):
      self.do_file_xfr(self.transfer_file_from_cfe,
                       ack=True,
                       check_file_exists=True, src_base=self.dir_fsw)
      
   @unittest.skip(reason="FIXME: Juicer bug with fixed-length strings interferes with sending cmd")
   def test_trivial_file_from_fsw_to_rosfsw_noack(self):
      self.do_file_xfr(self.transfer_file_from_cfe_to_rosfsw,
                       ack=False,
                       check_file_exists=True, src_base=self.dir_fsw)
   @unittest.skip(reason="FIXME: Juicer bug with fixed-length strings interferes with sending cmd")
   def test_trivial_file_from_fsw_to_rosfsw_ack(self):
      self.do_file_xfr(self.transfer_file_from_cfe_to_rosfsw,
                       ack=True,
                       check_file_exists=True, src_base=self.dir_fsw)
      
      
   # If size is 0, generate a trivial text file, otherwise a random binary file of specified length
   def generate_test_file(self, fp, size=0):
      if size==0:
         s = 'Hello CFE world, greetings!  Time on GSW now is %s.\n' % str(datetime.datetime.now())
         fp.write(s.encode('utf-8'))
      else:
         fp.write(os.urandom(size))

      fp.close()
      self.addCleanup(self.cleanup_file, fp.name)
      
   def cleanup_file(self, filename):
      # Cleanup function to remove the file
      if os.path.exists(filename) and not self.debug_disable_cleanup:
         os.remove(filename)

   def transfer_file_from_cfe_to_rosfsw(self, f, ack):
      return self.transfer_file_from_cfe(f, ack, dest_id=2)
   def transfer_file_from_cfe(self, f, ack = False, dest_id = 1):
      req = CFTxFileCmd()
      
      
      req.cfdp_class = 1 if ack else 0
      req.keep = 1
      req.chan_num = 1
      req.priority = 0
      req.dest_id = dest_id
      req.src_filename = f"/cf/{f}"
      req.dst_filename = f"{f}_rcvd"

      self.subscriber.cf_tx_cmd_sender.publish(req)

      # TODO: Spin and verify cmd receipt. 

      dst_path=f"{self.dir_rosgsw}/{f}_rcvd"
      self.addCleanup(self.cleanup_file, dst_path)
      return dst_path
         
   def transfer_file_to_cfe(self, f, ack = False):
      cfdp_request = CfdpXfrCmd.Request()
      cfdp_request.dstid = 25 # This is a *put* to the cfe entity id (25)
      cfdp_request.src = f
      cfdp_request.dst = ("/cf/%s_rcvd" % f)
      cfdp_request.ack = ack

      # TODO: Do we care about return value? If so, we should assert here
      self.subscriber.transfer_req(cfdp_request)

      if self.dir_fsw: # Only if we have access to fsw dst path
         dst_path=f"{self.dir_fsw}/{f}_rcvd"
         self.addCleanup(self.cleanup_file, dst_path) # TODO: VERIFY and cleanup
         return dst_path
      else:
         return None
   
   def transfer_file_to_rosfsw(self, f, ack = False):
      cfdp_request = CfdpXfrCmd.Request()
      cfdp_request.dstid = 2  # This is a *put* to the flightsystem entity id (2)
      cfdp_request.src = f
      cfdp_request.dst = ("%s_rcvd" % f)
      cfdp_request.ack = ack

      # TODO: Do we care about return value? If so, we should assert here
      self.subscriber.transfer_req(cfdp_request)
      if self.dir_rosfsw:
         dst_path=f"{self.dir_rosfsw}/{f}_rcvd"
         self.addCleanup(self.cleanup_file, dst_path)
         return dst_path
      else:
         return None
         
   # TODO: When used with do_file_xfr, an extra flag may be needed if src_base is inaccessible (ie: no shared dirs)
   def transfer_file_from_rosfsw(self, f, ack = False):
      cfdp_request = CfdpXfrCmd.Request()
      cfdp_request.dstid = 2  # This is a *get* from the flightsystem entity id (2)
      cfdp_request.src = ("%s" % f)
      cfdp_request.dst = ("%s_rcvd" % f)
      cfdp_request.ack = ack
      
      # TODO: Do we care about return value? If so, we should assert here
      self.subscriber.transfer_get_req(cfdp_request)

      dst_path=f"{self.dir_rosgsw}/{f}_rcvd"
      self.addCleanup(self.cleanup_file, dst_path)
      return dst_path
            
   # xfr_fn = Reference to the function to perform the actual file transfer
   # ack = ACK mode
   # cfe_expected_pdu_check = If non-zero, verify this number of PDUs were received on fsw side
   # cfe_check_file_size = If set, verify received PDU bytes matchees expected file size. This is mutually exclusive with cfe_expected_pdu_check.
   # file_size = If non-zero, the size of a non-trivial binary file to generate
   # src_base = Base directory for src file. If omitted, self.dir_rosgsw will be used
   # dst_base = Base directory for dst file. If omitted, file verification will be skipped. @DEPRECATED? in favor of xfr_fn return value
   def do_file_xfr(self, xfr_fn, ack=False,
                   cfe_expected_pdu_cnt=0, cfe_check_file_size=False, file_size=0,check_file_exists=False,
                   src_base=None
                   ):
      if src_base is None:
         src_base=self.dir_rosgsw

                  
      # Generate file of trivial length
      #  NOTE: We want delete_on_close=False, but that requires Python 3.12+
      with tempfile.NamedTemporaryFile(dir=src_base, delete=False) as fp:
         self.generate_test_file(fp,file_size)
         if file_size==0:
            os.stat(fp.name).st_size
         
         # the file is closed, but not removed
         # Now, transfer the file up to the CFE.
         rcv_path = xfr_fn(os.path.basename(fp.name), ack)

                  
         # Wait for expected number of PDUs received OR timeout OR file creation
         # WARNING: File existence check is dependent on file not previously existing. This should be guaranteed when using temporary named files, but may break if that changes in the future.
         success = False
         for i in range(10): # TODO/FIXME: Current tests should finish in <10 IF there are no retries

            # This should guarantee a minimum 1s delay between iterations if condition is not met
            rclpy.spin_once(self.subscriber, timeout_sec=30)
                  
            if cfe_expected_pdu_cnt > 0 and self.subscriber.get_cfdp_pdus_from_cf_hk() == self.base_pdu_cnt+cfe_expected_pdu_cnt:
               success = True
               break
            if cfe_check_file_size and self.subscriber.get_fsw_cfdp_rcv_bytes()==self.base_fsw_cfdp_rcv_bytes+file_size:
               success = True
               break

            # Note: In some cases file may be created before it's been completely received, so skipping this check for now
            # For better results, we need to add tlm hooks to ros cfdp to verify file receipt
            if check_file_exists and rcv_path and os.path.exists(rcv_path):
               success = True
               break

         assert success

         # Assert that destination file exists and matches input (if dst path is available)
         if rcv_path:
            #self.subscriber.get_logger().info(f"verify path {rcv_path}")
            assert os.path.exists(rcv_path)

            self.assertFilesEqual(fp.name, rcv_path)

     

if __name__ == '__main__':
   unittest.main(verbosity=2)
