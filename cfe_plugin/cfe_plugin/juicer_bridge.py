import yaml
from brash_bridge.juicer_interface import JuicerInterface
from brash_bridge.juicer_telem_receiver import JuicerTelemReceiver

# Receive port where the CFS TO_Lab app sends the telemetry packets
# udpRecvPort = 1235


class FSWPlugin(JuicerInterface):

    def __init__(self):
        with open("juicerConfig.yaml", "r") as ymlfile:
            cfg = yaml.safe_load(ymlfile)

        JuicerInterface.__init__(self)
        ros_name_map = {}
        for key in self.symbol_name_map.keys():
            symbol = self.symbol_name_map[key]
            msg_type = symbol.get_ros_name()
            ros_name_map[msg_type] = symbol
        self._tlm_receiver = JuicerTelemReceiver(cfg, ros_name_map, self.get_msg_list())

    def get_latest_data(self, key):
        return self._tlm_receiver.get_latest_data(key)
