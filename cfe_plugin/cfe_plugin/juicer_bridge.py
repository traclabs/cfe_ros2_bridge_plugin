import yaml
from brash_bridge.juicer_if import *
from brash_bridge.juicer_telem_receiver import JuicerTelemReceiver

# Receive port where the CFS TO_Lab app sends the telemetry packets
#udpRecvPort = 1235

class FSWPlugin(JuicerInterface):

    def __init__(self):
        with open("juicerConfig.yaml", "r") as ymlfile:
            cfg = yaml.safe_load(ymlfile)

        JuicerInterface.__init__(self)
        ros_name_map = {}
        for key in self.symbol_name_map.keys():
            symbol = self.symbol_name_map[key]
            msg_type = symbol.getROSName()
            ros_name_map[msg_type] = symbol
        self.tlm_receiver = JuicerTelemReceiver(cfg, ros_name_map, self.get_msg_list())

    def get_latest_data(self, key):
        return self.tlm_receiver.get_latest_data(key)
