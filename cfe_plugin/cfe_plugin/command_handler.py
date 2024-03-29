"""
.. module:: cfe_ros2_bridge_plugin.cfe_plugin.command_handler
   :synopsis: Class that listens for commands from ROS2

.. moduleauthor:: Tod Milam

"""

from cfe_msgs.msg import CFEMSGCommandHeader


class CommandHandler():
    """This class listens for commands from ROS2.

    Attributes
    ----------
    node : rosnode
        the ROS2 node
    header_size : int
        the size of the cFE header
    cmd_info :
        the configuration information for this command
    callback : method
        the method to call when a command is received
    cfe_mid : int
        the command cFE message id
    cmd_code : int
        the cFE command code
    msg_length : int
        the size of the message without the header

    Methods
    process_callback(msg):
        Handle callback holding ROS2 command.
    -------
    """
    def __init__(self, node, cmd_info, callback, mid, cid, size):
        '''
        Initializes the attributes of the command handler object.

        Args:
            node (rosnode): The ROS2 node
            cmd_info (): The configuration information for the command
            callback (method): The method to call when a command is received
            mid (int): The cFE message id of the command
            cid (int): The cFE command code of the command
            size (int): The size of the data packet
        '''
        self._header_size = 7
        self._node = node
        self._cmd_info = cmd_info
        self._callback = callback
        self._cfe_mid = mid
        self._cmd_code = cid
        self._msg_length = size - self._header_size

    def process_callback(self, msg):
        '''
        Handle callback holding ROS2 command.

        Args:
            msg (): The ROS2 message containing the command data
        '''
        for t in msg.__dir__():
            if isinstance(getattr(msg, t), CFEMSGCommandHeader):
                cmd_header = getattr(msg, t, None)
                if cmd_header != None:
                    cmd_header.msg.ccsds.pri.stream_id = self._cfe_mid
                    cmd_header.msg.ccsds.pri.length = self._msg_length
                    cmd_header.sec.function_code = self._cmd_code
                break
        self._callback(self._cmd_info, msg)
