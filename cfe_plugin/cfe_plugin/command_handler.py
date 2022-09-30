
class CommandHandler():
    def __init__(self, node, cmd_info, callback):
        self._node = node
        self._cmd_info = cmd_info
        self._callback = callback

    def process_callback(self, msg):
        self._callback(self._cmd_info, msg)

