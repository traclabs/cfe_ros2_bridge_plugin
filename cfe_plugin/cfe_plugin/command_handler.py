

class CommandHandler():
    def __init__(self, node, cmd_info, callback, mid, cid):
        self._node = node
        self._cmd_info = cmd_info
        self._callback = callback
        self._cfe_mid = mid
        self._cmd_code = cid
        self._msg_length = 8

    def process_callback(self, msg):
        cmd_header = getattr(msg, 'cmd_header', None)
        # NOTE: Special case - NOLABNoArgsCmdt.msg has it misspelled
        if cmd_header == None:
            cmd_header = getattr(msg, 'cmd_heade', None)
        if cmd_header != None:
            # override values with ones from config file
            cmd_header.msg.ccsds.pri.stream_id = self._cfe_mid
            # cmd_header.msg.ccsds.pri.length = self._msg_length
            cmd_header.sec.function_code = self._cmd_code
        self._callback(self._cmd_info, msg)
