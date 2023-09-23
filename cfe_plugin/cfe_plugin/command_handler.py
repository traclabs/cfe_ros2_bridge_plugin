
from cfe_msgs.msg import CFEMSGCommandHeader


class CommandHandler():
    def __init__(self, node, cmd_info, callback, mid, cid, size):
        self._header_size = 7
        self._node = node
        self._cmd_info = cmd_info
        self._callback = callback
        self._cfe_mid = mid
        self._cmd_code = cid
        self._msg_length = size - self._header_size

    def process_callback(self, msg):

        for t in msg.__dir__():
            if isinstance(getattr(msg, t), CFEMSGCommandHeader):
                cmd_header = getattr(msg, t, None)
                if cmd_header != None:
                    cmd_header.msg.ccsds.pri.stream_id = self._cfe_mid
                    cmd_header.msg.ccsds.pri.length = self._msg_length
                    cmd_header.sec.function_code = self._cmd_code
                break
        self._callback(self._cmd_info, msg)
