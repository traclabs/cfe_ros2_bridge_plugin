import threading
import socket
import rclpy
from struct import unpack
import importlib

class JuicerTelemReceiver():
    def __init__(self, config, rosNameMap, msgList):
        self.rosNameMap = rosNameMap
        self.msgList = msgList
        self.port = config["telemetryPort"]
        telemetrys = config["telemetry"]
        self.tlmMap = {}
        for tlmEntry in telemetrys:
            self.tlmMap[tlmEntry['cfeId']] = tlmEntry['rosName']
        self.logger = rclpy.logging.get_logger("JuicerTelemReceiver")
        self.logger.info("telem map is " + str(self.tlmMap))
        self.recvBuffSize = 4096

        self.running = True
        self.recv_thread = threading.Thread(target=self.receiveThread)

        self.logger.warn("starting thread to receive CFS telemetry")
        self.recv_thread.start()
        self.currentValue = {}

    def stopThread(self):
        self.running = false
        self.recv_thread.join()

    def receiveThread(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.port))

        socketErrCount = 0
        while self.running:
            try:
                # receive message
                datagram, host = self.sock.recvfrom(self.recvBuffSize)

                # ignore data if not long enough (doesn't contain header)
                if len(datagram) < 6:
                    continue

                self.handlePacket(datagram)

            except socket.error:
                self.logger.warn("Error receiving telemetry data.")

    def handlePacket(self, datagram):
        packetid = self.getPktId(datagram)
        if packetid in self.tlmMap:
            self.logger.info("Received packet for " + self.tlmMap[packetid])
            MsgType = getattr(importlib.import_module("tod_test.msg"), self.tlmMap[packetid])
            msg = MsgType()
            setattr(msg, "seq", self.getSeqCount(datagram))
            self.parsePacket(datagram, 0, self.tlmMap[packetid], msg)
            symbol = self.rosNameMap[self.tlmMap[packetid]]
            self.currentValue[symbol.getName()] = msg
        else:
            self.logger.warn("Don't know how to handle message id " + packetid)

    def parsePacket(self, datagram, offset, rosName, msg):
        symbol = self.rosNameMap[rosName]
        fields = symbol.getFields()
        for field in fields:
            fsym = field.getTypeSymbol()
            self.logger.info("handle field " + field.getROSName() + " of type " + fsym.getROSName())
            offs = offset + field.getByteOffset()
            val = None
            # self.msgList contains list of data types that need to be processed
            if fsym.getROSName() in self.msgList:
                MsgType = getattr(importlib.import_module("tod_test.msg"), fsym.getROSName())
                fmsg = MsgType()
                val = self.parsePacket(datagram, offs, fsym.getROSName(), fmsg)
            else:
                if (fsym.getROSName() == 'string') or (fsym.getROSName() == 'char'):
                    # copy code from cfs_telem_receiver
                    ca = ""
                    for s in range(int(fsym.getSize())):
                        tf = unpack('c', datagram[(offs+s):(offs+s+1)])
                        ca = ca + codecs.decode(tf[0], 'UTF-8')
                    val = ca
                else:
                    size = fsym.getSize()
                    fmt = self.getUnpackFormat(fsym.getROSName())
                    TlmField = unpack(fmt, datagram[offs:(offs+size)])
                    val = TlmField[0]
            # do something with val here
            if val != None:
                setattr(msg, field.getROSName(), val)

    def getLatestData(self, key):
        retval = None
        if key in self.currentValue:
            retval = self.currentValue[key]
            #self.logger.info("Returning data for " + key)
        return None

    def getUnpackFormat(self, rosName):
        retval = "B"
        if rosName == "uint64":
            retval = "Q"
        elif rosName == "uint32":
            retval = "I"
        elif rosName == "uint16":
            retval = "H"
        elif rosName == "uint8":
            retval = "B"
        elif rosName == "int64":
            retval = "q"
        elif rosName == "int32":
            retval = "i"
        elif rosName == "int16":
            retval = "h"
        elif rosName == "int8":
            retval = "b"
        elif rosName == "char":
            retval = "s"
        elif rosName == "bool":
            retval = "?"
        else:
            self.logger.warn("Failed to get unpack format for " + rosName)
        return retval

    @staticmethod
    def getPktId(datagram):
        streamid = unpack(">H", datagram[:2])
        return hex(streamid[0])

    @staticmethod
    def getSeqCount(datagram):
        streamid = unpack(">H", datagram[2:4])
        return streamid[0] & 0x3FFF  ## sequence count mask
