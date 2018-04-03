#Run start button

import sys
import os
import math

from time import sleep
from pdb import set_trace as bp


from ConfigParser import SafeConfigParser
from argparse import ArgumentParser
from PyDragonfly import Dragonfly_Module, CMessage, copy_to_msg, copy_from_msg, MT_EXIT
#from dragonfly_utils import respond_to_ping
import Dragonfly_config as rc
import serial


class StartButton():    
    def listen(self):
        print "Running Start button..."
        self.mod = Dragonfly_Module(0, 0)
        self.mod.ConnectToMMM()

        self.msg = CMessage()
        out = rc.MDF_MOVE_HOME()
        
        while True:
            ser = serial.Serial('/dev/ttyACM0', 9600)
            while True:
                try:
                    outMsg = ser.readline()
                    print outMsg
                    
                    # send start/stop message
                    if ("On" in outMsg) or ("Off" in outMsg): # 'oN' or 'ofF' msg sent
                        
                        out.shouldMove = "Off" not in outMsg # double negative intentional
                        
                        #send msg
                        print "sending msg"
                        newMsg = CMessage(rc.MT_MOVE_HOME)
                        copy_to_msg(out, newMsg)
                        self.mod.SendMessage(newMsg)
                except:
                    # send stop message
                    out.moving = False;
                    newMsg = CMessage(rc.MT_MOVE_HOME)
                    copy_to_msg(out, newMsg)
                    self.mod.SendMessage(newMsg);
                    break;
    
startBtn = StartButton()
startBtn.listen()