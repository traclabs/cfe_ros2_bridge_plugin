import json
import rclpy
import sqlite3
from sqlite3 import Error

from fsw_ros2_bridge.fsw_plugin_interface import *
from fsw_ros2_bridge.telem_info import TelemInfo
from fsw_ros2_bridge.command_info import CommandInfo
from cfe_plugin.juicer_fields import JuicerFieldEntry
from cfe_plugin.juicer_symbols import JuicerSymbolEntry

class JuicerInterface(FSWPluginInterface):
#class FSWPlugin(FSWPluginInterface):

    def __init__(self):
        dbFile = "/home/tmilam/code/juicer/build/core-cpu1.sqlite"
        self.logger = rclpy.logging.get_logger("JuicerInterface")
        self.jsonConfigFile = "juicerConfig.json"

        self.conn = self.create_connection(dbFile)
        self.fieldNameMap = dict()
        self.symbolNameMap = dict()
        self.symbolIdMap = dict()

        self.telem_info = []
        self.command_info = []
        self.recv_map = {}

        self.loadConfig()
        self.loadData()

        for key in self.symbolNameMap.keys():
            symbol = self.symbolNameMap[key]
            if symbol.getShouldOutput():
                if symbol.getIsCommand():
                    #print("Found command " + symbol.getName())
                    cKey = symbol.getName()
                    cMsgType = symbol.getROSName()
                    cTopic = symbol.getROSTopic()
                    #c = CommandInfo(cKey, cMsgType, cTopic)
                    #self.command_info.append(c)
                elif symbol.getIsTelemetry():
                    tKey = symbol.getName()
                    tMsgType = symbol.getROSName()
                    tTopic = symbol.getROSTopic()
                    t = TelemInfo(tKey, tMsgType, tTopic)
                    self.telem_info.append(t)

    def create_connection(self, db_file):
        """ create a database connection to the SQLite database
            specified by the db_file
        :param db_file: database file
        :return: Connection object or None
        """
        conn = None
        try:
            conn = sqlite3.connect(db_file)
        except Error as e:
            self.logger.error(e)

        return conn

    def retrieve_all_fields(self):
        """
        Query all rows in the fields table
        :return: A mapping of field name to field object
        """
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM fields")

        rows = cur.fetchall()
        for row in rows:
            myField = JuicerFieldEntry(row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7])
            self.fieldNameMap[myField.getName()] = myField
            symbol = self.symbolIdMap[myField.getSymbol()]
            if symbol is not None:
                typeid = myField.getType()
                myField.setTypeSymbol(self.symbolIdMap[typeid])
                symbol.addField(myField)
        return self.fieldNameMap

    def retrieve_all_symbols(self):
        """
        Query all rows in the symbols table
        :return: A mapping of symbol name to symbol object
        """
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM symbols")
        rows = cur.fetchall()

        for row in rows:
            mySymbol = JuicerSymbolEntry(row[0], row[1], row[2], row[3])
            self.symbolIdMap[mySymbol.getId()] = mySymbol
            if not mySymbol.getName().startswith("_"):
                self.symbolNameMap[mySymbol.getName()] = mySymbol
        return self.symbolIdMap

    def loadData(self):
        self.retrieve_all_symbols()
        self.retrieve_all_fields()
        self.pruneSymbolsAndFields()
        self.markCmdTlmSymbols()

    def loadConfig(self):
        with open(self.jsonConfigFile, "r") as jsonfile:
            jsonConfig = json.load(jsonfile)
            self.cmdIds = jsonConfig["commands"]
            self.tlmIds = jsonConfig["telemetry"]

    def pruneSymbolsAndFields(self):
        self.logger.info("Pruning out things that aren't needed.")
        self.emptySymbols = []
        for key in self.symbolNameMap.keys():
            symbol = self.symbolNameMap[key]
            if len(symbol.getFields()) == 0:
                self.emptySymbols.append(symbol)
                # for some reason some messages need to be added twice, so just automatically do it for all of them
                self.emptySymbols.append(symbol)
        self.logger.info("There are " + str(len(self.emptySymbols)) + " empty symbols")
        for symbol in self.emptySymbols:
            mn = symbol.getROSName()
            # if it starts with lower case then it is ROS2 native type so ignore it
            if mn[0].isupper():
                altSym = self.findAlternativeSymbol(symbol)
                if altSym is not None:
                    self.emptySymbols.remove(symbol)
                    symbol.setAlternative(altSym)
                #else:
                    #print("Unable to find an alternative for " + symbol.getName())
        self.logger.info("There are " + str(len(self.emptySymbols)) + " empty symbols left after pruning")

    def findAlternativeSymbol(self, emptySymbol):
        #print("empty symbol " + emptySymbol.getROSName() + " from " + emptySymbol.getName())
        for key in self.symbolNameMap.keys():
            symbol = self.symbolNameMap[key]
            if emptySymbol.getName().startswith(symbol.getName()):
                if not emptySymbol == symbol:
                    if emptySymbol.getSize() == symbol.getSize():
                        #print("Should replace " + emptySymbol.getName() + " with " + symbol.getName())
                        return symbol
                    else:
                        self.logger.warn("Can't replace " + emptySymbol.getName() + " with " + symbol.getName())
                        self.logger.warn("wrong size " + str(emptySymbol.getSize()) + " vs " + str(symbol.getSize()))
            elif emptySymbol.getName() == "CFE_EVS_SetEventFormatMode_Payload_t" and symbol.getName() == "CFE_EVS_SetEventFormatCode_Payload":
                self.logger.info("Handling the SetEventFormatMode vs SetEventFormatCode problem")
                return symbol
        return None

    def getTelemetryMessageInfo(self):
        return self.telem_info

    def getCommandMessageInfo(self):
        return self.command_info

# NOTE: getLatestData is in juicer_bridge.py
#    def getLatestData(self, key):
        #return self.recv_map[key].getLatestData()
#        return None

    def getMsgList(self):
        msgList = []
        for key in self.symbolNameMap.keys():
            symbol = self.symbolNameMap[key]
            if len(symbol.getFields()) > 0 and symbol.getShouldOutput():
                mn = symbol.getROSName()
                if mn[0].isupper():
                    msgList.append(mn)
        return msgList

    def getTopicList(self):
        topicList = []
        for key in self.symbolNameMap.keys():
            symbol = self.symbolNameMap[key]
            if len(symbol.getFields()) > 0:
                tn = symbol.getROSTopic()
                topicList.append(tn)
        return topicList

    def markCmdTlmSymbols(self):
        self.logger.info("Marking symbols that are necessary for messages.")
        self.emptySymbols = []
        for key in self.symbolNameMap.keys():
            symbol = self.symbolNameMap[key]
            if symbol.getIsCommand() or symbol.getIsTelemetry():
                self.markOutputSymbol(symbol)

    def markOutputSymbol(self, symbol):
        symbol.setShouldOutput(True)
        fields = symbol.getFields()
        for field in fields:
            fieldSymbol = field.getTypeSymbol()
            self.markOutputSymbol(fieldSymbol)

def fieldSortOrder(field):
    return field.getBitOffset()

