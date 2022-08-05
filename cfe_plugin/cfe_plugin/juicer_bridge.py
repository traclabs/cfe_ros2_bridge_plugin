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
        rosNameMap = {}
        for key in self.symbolNameMap.keys():
            symbol = self.symbolNameMap[key]
            msgType = symbol.getROSName()
            rosNameMap[msgType] = symbol
        self.tlmReceiver = JuicerTelemReceiver(cfg, rosNameMap, self.getMsgList())

    def getLatestData(self, key):
        return self.tlmReceiver.getLatestData(key)

