#Run start button

import sys
import signal
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
        self.mod = Dragonfly_Module(63, 0) #pretend 64 is a valid MID for now
        self.mod.ConnectToMMM()
        out = rc.MDF_MOVE_HOME()
        
        # Handle interrupts gracefully, disconnect from dragonfly
        def sigHandler(*args):
          print('You pressed Ctrl+C... probably')          
          
          # Stop moving arm
          out.moving = False;
          newMsg = CMessage(rc.MT_MOVE_HOME)
          copy_to_msg(out, newMsg)
          self.mod.SendMessage(newMsg)          
          
          # Disconnect from dragonfly
          self.mod.DisconnectFromMMM()
          sys.exit(0)
          
        
        # Use signal handler
        signal.signal(signal.SIGINT, sigHandler)
        
        # Listen for button press
        while True:
            ser = serial.Serial('/dev/ttyUSB1', 9600)
            while True:
                try:
                    outMsg = ser.readline()
                    print outMsg
                    
                    # send start/stop message
                    if ("On" in outMsg) or ("Off" in outMsg):
                        out.shouldMove = "Off" not in outMsg # intentional double negative 
                        
                        #send msg
                        print "sending msg"
                        newMsg = CMessage(rc.MT_MOVE_HOME)
                        copy_to_msg(out, newMsg)
                        self.mod.SendMessage(newMsg)
                except KeyboardInterrupt:
                    pass
                except:
                    # send stop message
                    out.moving = False;
                    print "Error, relaunching"
                    newMsg = CMessage(rc.MT_MOVE_HOME)
                    copy_to_msg(out, newMsg)
                    self.mod.SendMessage(newMsg)
                    break
                    
        self.mod.DisconnectFromMMM()
    
startBtn = StartButton()
startBtn.listen()
