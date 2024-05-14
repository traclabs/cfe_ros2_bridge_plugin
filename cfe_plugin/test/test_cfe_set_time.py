import unittest
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cfe_msgs.msg import CFEESHousekeepingTlm
from cfe_msgs.msg import CFETIMETimeCmd

import socket

# https://stackoverflow.com/a/48473935
NTP_SERVER = '10.5.0.3'
TIME1970 = 2208988800
def query_sntp_time_from_cfe_sntp_app():
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data = '\x1b' + 47 * '\0'
    client.sendto(data.encode('utf-8'), (NTP_SERVER, 123))
    data, address = client.recvfrom(1024)
    if data: print('Response received from:', address)
    t = struct.unpack('!12I', data)[10] - TIME1970
    return t

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
        msg.payload.seconds = stcf
        msg.payload.micro_seconds = 0
        self.set_stcf_publisher.publish(msg)

    # Function that sends a CFE Set MET command using time_seconds as the desired seconds value.
    def publish_set_met_cmd(self, time_seconds):
        msg = CFETIMETimeCmd()
        msg.payload.seconds = time_seconds
        msg.payload.micro_seconds = 0
        self.set_met_publisher.publish(msg)

    def listener_callback(self, msg):
        # Record that a message was seen.
        self.num_messages_received += 1

        # Extract time from ES housekeeping packet secondary header.
        (met_seconds, met_subseconds) = struct.unpack(">IH", msg.telemetry_header.sec.time)
        self.last_cfe_met_sec_received = met_seconds

        # Generate a log message
        self.get_logger().info('I heard something from cfe_es_hk_tlm with met %d where the 6 bytes of time is [ %02x %02x %02x %02x %02x %02x ]' % 
                                (met_seconds,
                                msg.telemetry_header.sec.time[0],
                                msg.telemetry_header.sec.time[1],
                                msg.telemetry_header.sec.time[2],
                                msg.telemetry_header.sec.time[3],
                                msg.telemetry_header.sec.time[4],
                                msg.telemetry_header.sec.time[5]
                                ))
      

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

        # Spin for a few messages to ensure we see fresh messages.
        for i in range(10):
            rclpy.spin_once(subscriber, timeout_sec=30)

        # The initial STCF is a CFE compiled in default value
        initial_stcf = 1000000  # See CFE_MISSION_TIME_DEF_STCF_SECS

        # The CFE EPOCH is a compiled in default value (currently Jan 1 1980)
        # See CFE_MISSION_TIME_EPOCH_YEAR
        time_between_unix_epoch_and_cfe_epoch = 315532800

        # Send the Set STCF (Spacecraft Time Correlation Factor) function such that the MET corresponds to UTC.
        # Do this by setting the STCF to be the offset from UNIX time to the start of the CFE EPOCH such that
        # MET of 0 represents Jan 1 1980.
        subscriber.publish_set_stcf_cmd(time_between_unix_epoch_and_cfe_epoch)

        # Get current UNIX time in seconds
        time_at_t = int(time.time())
        # Convert this current UNIX time into CFE time
        current_time_in_cfe_epoch = time_at_t - time_between_unix_epoch_and_cfe_epoch
        # Send the Set MET command to set the CFE MET to CFE "current time"
        subscriber.publish_set_met_cmd(current_time_in_cfe_epoch)

        time.sleep(3)

        # Spin for a few messages to ensure we see fresh messages.
        for i in range(10):
            rclpy.spin_once(subscriber, timeout_sec=30)

        # Assert that we've heard a message and that the time in the secondary header corresponds to the new MET value.
        self.assertGreater(subscriber.get_messages_heard(), 0)
        # Since we've waited some time, assert that the last MET received in a CFE packet header is greater than our initial time_at_t above.
        self.assertGreaterEqual(subscriber.get_last_cfe_met_sec_received(), time_at_t)
        # Also assert that the last MET received in a CFE packet header is less than 30 seconds from the current time.
        self.assertLess(subscriber.get_last_cfe_met_sec_received(), int(time.time()) + 30)

        # Now test the SNTP service.
        # Act as an SNTP client and query the CFE SNTP application and verify that it gives a correct time now that
        # CFE has been set up with an accurate time.
        tolerance = 60
        # Since we've set the CFE time accurately above, we should be able to compare SNTP time through CFE with 
        # current UNIX time and get agreement within a tolerance.
        # Get current UNIX time
        time_now = int(time.time())
        t = query_sntp_time_from_cfe_sntp_app()
        self.assertGreater(t, time_now - tolerance)
        self.assertLess(t, time_now + tolerance)

        subscriber.destroy_node()

if __name__ == '__main__':
    unittest.main()
