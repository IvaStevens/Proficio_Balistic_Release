## Test Executive

import sys
import os
import math

from time import sleep


from ConfigParser import SafeConfigParser
from argparse import ArgumentParser
from PyDragonfly import Dragonfly_Module, CMessage, copy_to_msg, copy_from_msg, read_msg_data, MT_EXIT
from dragonfly_utils import respond_to_ping
import Dragonfly_config as rc


class ExecTester():
    def printMsg(msg):
        print "====== Recieved msg: ======= \n",
              "State: ", msg.state, "\n"
              #"Force: ", msg.force, "\n"
              "Dir:   ", msg.direction, "\n"
              "============================ \n"
    def __run__(self):
        self.mod = Dragonfly_Module(0, 0)
        self.mod.ConnectToMMM()
        self.mod.Subscribe(rc.MT_TASK_STATE_CONFIG)

        self.msg = CMessage()

        while(true):
            ## listen for start msg,
            rcv = self.mod.ReadMessage(self.msg, 0)
            hdr = self.msg.GetHeader()
            msgType = hdr.msg_type
            if msgType == rc.MT_TASK_STATE_CONFIG:
                mdf = rc.MDF_TASK_STATE_CONFIG()
                copy_from_msg(mdf, self.msg)
                printMsg(mdf)

                ## wait a few seconds,
                sleep(5)
                
                ## send either success or fail msg
                mdf = rc.MDF_BURT_STATUS()
                mdf.state =
                mdf.trialComplete = true;
                mdf.trialSuccess = true;
                
                newMsg = CMessage(rc.MT_BURT_STATUS)
                copy_to_msg(mdf, newMsg)
                self.mod.Send(newMsg);
                
                ## verify that you recieve the next trial.
            
