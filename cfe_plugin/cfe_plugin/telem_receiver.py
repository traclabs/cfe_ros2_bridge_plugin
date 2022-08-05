import threading
import socket
import rclpy
from struct import unpack
import importlib

class TelemReceiver():
    def __init__(self, node, msg_pkg, port, telem_info, msg_list):
        self.node = node
        self.ros_name_map = {}
        self.msg_list = msg_list
        self.port = port
        self.msg_pkg = msg_pkg
        self.tlm_map = {}
        self.logger = rclpy.logging.get_logger("TelemReceiver")
        for tlm in telem_info:
            self.node.get_logger().info("type: " + str(tlm))
            self.node.get_logger().info("  cfe_mid: " + str(telem_info[tlm]['cfe_mid']))
            self.node.get_logger().info("  topic_name: " + telem_info[tlm]['topic_name'])
            self.tlm_map[telem_info[tlm]['cfe_mid']] = tlm
            self.ros_name_map[tlm] = telem_info[tlm]['topic_name']
        self.logger.info("telem map is " + str(self.tlm_map))
        self.recv_buff_size = 4096

        self.running = True
        self.recv_thread = threading.Thread(target=self.receiveThread)

        self.logger.warn("starting thread to receive CFS telemetry")
        self.recv_thread.start()
        self.current_value = {}

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
                datagram, host = self.sock.recvfrom(self.recv_buff_size)

                # ignore data if not long enough (doesn't contain header)
                if len(datagram) < 6:
                    continue

                self.handlePacket(datagram)

            except socket.error:
                self.logger.warn("Error receiving telemetry data.")

    def handlePacket(self, datagram):
        packetid = self.getPktId(datagram)
        if packetid in self.tlm_map:
            self.logger.info("Received packet for " + self.tlm_map[packetid])
            MsgType = getattr(importlib.import_module(self.msg_pkg + ".msg"), self.tlm_map[packetid])
            msg = MsgType()
            setattr(msg, "seq", self.getSeqCount(datagram))
            self.parsePacket(datagram, 0, self.tlm_map[packetid], msg)
            symbol = self.ros_name_map[self.tlm_map[packetid]]
            self.current_value[symbol.getName()] = msg
        else:
            self.logger.warn("Don't know how to handle message id " + packetid)

    def parsePacket(self, datagram, offset, ros_name, msg):
        symbol = self.rosNameMap[rosName]
        fields = symbol.getFields()
        for field in fields:
            fsym = field.getTypeSymbol()
            self.logger.info("handle field " + field.getROSName() + " of type " + fsym.getROSName())
            offs = offset + field.getByteOffset()
            val = None
            # self.msg_list contains list of data types that need to be processed
            if fsym.getROSName() in self.msg_list:
                MsgType = getattr(importlib.import_module(self.msg_pkg + ".msg"), fsym.getROSName())
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
                    tlm_field = unpack(fmt, datagram[offs:(offs+size)])
                    val = tlm_field[0]
            # do something with val here
            if val != None:
                setattr(msg, field.getROSName(), val)

    def getLatestData(self, key):
        retval = None
        if key in self.current_value:
            retval = self.current_value[key]
            #self.logger.info("Returning data for " + key)
        return None

    def getUnpackFormat(self, ros_name):
        retval = "B"
        if ros_name == "uint64":
            retval = "Q"
        elif ros_name == "uint32":
            retval = "I"
        elif ros_name == "uint16":
            retval = "H"
        elif ros_name == "uint8":
            retval = "B"
        elif ros_name == "int64":
            retval = "q"
        elif ros_name == "int32":
            retval = "i"
        elif ros_name == "int16":
            retval = "h"
        elif ros_name == "int8":
            retval = "b"
        elif ros_name == "char":
            retval = "s"
        elif ros_name == "bool":
            retval = "?"
        else:
            self.logger.warn("Failed to get unpack format for " + ros_name)
        return retval

    @staticmethod
    def getPktId(datagram):
        streamid = unpack(">H", datagram[:2])
        return hex(streamid[0])

    @staticmethod
    def getSeqCount(datagram):
        streamid = unpack(">H", datagram[2:4])
        return streamid[0] & 0x3FFF  ## sequence count mask
