import rclpy
#import yaml
from brash_bridge.juicer_if import *


class FSWPlugin(JuicerInterface):

    def __init__(self):
        self._logger = rclpy.logging.get_logger("JuicerConverter")
        JuicerInterface.__init__(self)

    def createROSMsgs(self, msgs_dir):
        for key in self._symbolNameMap.keys():
            #self.outputROSMsgFile(self._symbolNameMap[key], msgs_dir)
            if self._symbolNameMap[key].getShouldOutput():
                self.outputROSMsgFile(self._symbolNameMap[key], msgs_dir)

        msgList = []
        cmdList = []
        tlmList = []
        for key in self._symbolNameMap.keys():
            symbol = self._symbolNameMap[key]
            if len(symbol.getFields()) > 0 and symbol.getShouldOutput():
                mn = symbol.getROSName()
                if mn[0].isupper():
                    msgList.append(mn)
                    if symbol.getIsCommand():
                        item = {"rosName": mn, "cfeId": "0x1200"}
                        cmdList.append(item)
                    if symbol.getIsTelemetry():
                        item = {"rosName": mn, "cfeId": "0x200"}
                        tlmList.append(item)
        cfgItems = {"commands": cmdList, "telemetry": tlmList}
        #print(yaml.dump(cfgItems))

        return msgList

    def outputROSMsgFile(self, symbol, msgs_dir):
        if len(symbol.getFields()) > 0:
            v_names = {}
            mn = symbol.getROSName()
            fn = msgs_dir + "/msg/" + mn + ".msg"
            f = open(fn, "w")
            f.write("# cFE NAME: " + symbol.getName() + "\n")
            f.write("# ROS topic: " + symbol.getROSTopic() + "\n")
            if symbol.getIsCommand():
                f.write("# Command message" + "\n")
            if symbol.getIsTelemetry():
                f.write("# Telemetry message" + "\n")
                f.write("int32 seq" + "\n")
            #f.write("int32 seq\n")
            fields = symbol.getFields()
            fields.sort(key=fieldSortOrder)
            for field in symbol.getFields():
                typename = field.getTypeName()
                fn = field.getROSName()
                if fn not in v_names.keys():
                    v_names[fn] = 0
                else:
                    v_names[fn] += 1
                    #print(mn + " has duplicate field name " + fn)
                    fn = fn + "_" + str(v_names[fn])
                f.write(typename + " " + fn + "\n")
            f.close()
