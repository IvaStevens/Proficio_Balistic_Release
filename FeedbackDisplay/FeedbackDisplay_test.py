
import sys
import os
import math
import time
from pdb import set_trace as bp

from cocos.actions import *
from cocos.director import director
from cocos.layer import Layer, ColorLayer
from cocos.scene import Scene
from cocos.sprite import Sprite
from cocos.text import Label

from enumParser import ENUMS as enums

from primitives import Polygon, Circle

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon as Shapelygon

import pyglet
from pyglet.gl import *


from ConfigParser import SafeConfigParser
from argparse import ArgumentParser
from PyDragonfly import Dragonfly_Module, CMessage, copy_to_msg, copy_from_msg, MT_EXIT
from dragonfly_utils import respond_to_ping
import Dragonfly_config as rc

try:
    import winsound
except ImportError:
    import os
    def playsound(frequency,duration): # Linux machines
        os.system('beep -f %s -l %s' % (frequency,duration))
else:
    def playsound(frequency,duration): # Windows machines
        winsound.Beep(frequency,duration)

MAX_WIDTH = 1280
OFFSET=20


BEGINTRIAL_TS = 1
FORCERAMP_TS = 2
REWARD_TS = 5

# Colors
WHITE = (255, 255, 255, 255)
GREEN = (0, 100, 0, 255)
RED   = (255, 0, 0, 255)
BRGDY = (100, 0, 0, 255)
MSTRD = (175, 175, 0, 255)
GRY_D = (40, 40, 40, 255)
GRY_L = (180, 180, 180, 255) 
BLACK = (0, 0, 0, 255)
BLUE  = (0, 50, 150, 255)

# Position origins
FEEDBACK_ORIGIN = [(20, 405), (20, 615), (28, 615), (28, 405)]
TARGET_ORIGIN = [(240, 680), (240, 720), (460, 720), (460, 680)]
POSITION_ORIGIN = [(260, 60), (260, 740), (440, 740), (440, 60)]
CURSOR_ORIGIN = [50, 400]

currentAngle = 0


class Display(ColorLayer):
    is_event_handler = True

    def __init__(self):
        super(Display, self).__init__(*GRY_L)

        self.mod = Dragonfly_Module(0, 0)
        self.mod.ConnectToMMM()
        self.mod.Subscribe(rc.MT_TASK_STATE_CONFIG)
        self.mod.Subscribe(rc.MT_INPUT_DOF_DATA)
        self.mod.Subscribe(rc.MT_COMBO_WAIT)
        self.mod.Subscribe(rc.MT_TRIAL_CONFIG)
        self.mod.Subscribe(rc.MT_END_TASK_STATE)
        self.mod.Subscribe(rc.MT_PING)
        self.mod.Subscribe(rc.MT_RT_POSITION_FEEDBACK)
        bp()
        self.mod.Subscribe(rc.MT_BURT_STATUS)
        
        self.msg = CMessage()
        
        self.timer_sec = 0
        self.timer_min = 0
        
        self.transformationType = 0
        
        dims = director.get_window_size()
        self.width = dims[0]
        self.height = dims[1]
        self.oldWidth = 1280
        self.oldHeight = 1024
        
        self.blank_display = False
        self.background = ColorLayer(*GRY_L)
        
        #positionOrigin = self.getPositionBarOrigin(self.width, self.height)
        self.position_bar = Polygon(v=POSITION_ORIGIN, color=(0.05, 0.05, 0.05, 1), stroke=0)
        #self.rotatePositionBar(currentAngle)
        #elf.resizePolygon(self.position_bar)
        
        self.cursor = Circle(x=CURSOR_ORIGIN[0], y=CURSOR_ORIGIN[1], width=50, color=(0, 0.2, 1.0, 1), stroke=0)
        
        self.tgt_window   = Polygon(v=TARGET_ORIGIN, color=(0, 0.6, 0.2, 1), stroke=0)
        #self.resizePolygon(self.tgt_window)
        
        self.pos_fdbk     = Polygon(v=FEEDBACK_ORIGIN, color=(0, 0.2, 1.0, 1), stroke=0)
        self.resizePolygon(self.pos_fdbk)
        
        self.pos_fdbk_txt = pyglet.text.Label('',
                                      font_name='times new roman',
                                      font_size=32,
                                      color=BLACK,
                                      width=250,
                                      bold=True,
                                      x=1140, y=40)

        self.combo_wait_txt = pyglet.text.Label('',
                                      font_name='times new roman',
                                      font_size=32,
                                      color=WHITE,
                                      width=250,
                                      bold=True,
                                      x=478, y=840)
                                      
        self.score_txt = pyglet.text.Label('',
                                      font_name='times new roman',
                                      font_size=36,
                                      color=BLACK,
                                      width=250,
                                      bold=True,
                                      x=500, y=240)                                      

        self.reward_txt = pyglet.text.Label('',
                                      font_name='times new roman',
                                      font_size=22,
                                      color=BLACK,
                                      width=250,
                                      bold=True,
                                      x=475, y=120)                                      


        self.reset_score()

        self.schedule_interval(self.update, 0.01)
        print "init"

        #     def timer_count_down(self, dt):
        #         self.timer_sec -= 1
        #         
        #         if self.timer_sec < 0:
        #             if self.timer_min > 0:
        #                 self.timer_min -= 1
        #                 self.timer_sec = 59
        #             else:
        #                 self.unschedule(self.timer_count_down)
        #                 self.combo_wait_txt.text = ''
        #                 self.screen_on()
        #                 return
        #         
        #         self.combo_wait_txt.text = 'Relax Time   %d:%02d' % (self.timer_min, self.timer_sec)
        # 
        #         if self.timer_min == 0 and self.timer_sec <= 5 and self.timer_sec > 0:
        #             winsound.PlaySound(os.path.join(os.environ.get('ROBOT_CONFIG'), 'default', 'gocue.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)

    # Convert base 255 color to base 1.0 color scale
    def colorMod(self, color):
        # Base 255 color
        temp = tuple( c/255.0 for c in color )
        return temp #+ (1.0,)

    def resizePolygon(self, polygon):
        for i in xrange(4):
            polygon.v[i] = (polygon.v[i][0] * self.width / self.oldWidth, polygon.v[i][1] * self.height / self.oldHeight)


    def getPositionBarOrigin(self, pageWidth, pageHeight):
        cx = pageWidth/2
        cy = pageHeight/2
        
        length = (pageWidth - 80) / 2
        height = 50
        top = cy - height
        bot = cy + height
        left = cx - length
        right = cy + length
        return [(left, top), (left, bot), (bot, right), (top, right)]

    # Rotate a point around specified origin
    def rotatePoint(self, oldPoint, angle, origin):
        ox, oy = origin
        print("origin: ", origin)
        px, py = oldPoint
        print "old point: ", oldPoint
        print "angle", angle
        
        qx = ox + (math.cos(angle) * (px - ox)) - (math.sin(angle) * (py - oy))
        qy = oy + (math.sin(angle) * (px - ox)) + (math.cos(angle) * (py - oy))
        print "new point: ", qx, qy
        return qx, qy
    
    # Move the target origin, and change its sixe
    def newTarget(self, width, xCenter):
        global TARGET_ORIGIN
        left  = xCenter - width/2
        right = xCenter + width/2
        _, top = TARGET_ORIGIN[0]
        _, bot = TARGET_ORIGIN[1]
        newTarget = [(left, top), (left, bot), (right, bot), (right, top)]
        return newTarget

    # Set the task state
    def setState(self, msg, state = 1):
      # States: start, forceRamp, forceHold, targetMove, targetHold, rest, reset, success, error
        
        # Default state:
        
        # Cursor either 80 or 50 pixles
        self.cursor.width = 50
        # Target either invisible (GREY), green, yellow, red
        self.tgt_window.color = colorMod(MSTRD)
        # Background either white or grey
        self.background.color = colorMod(WHITE)
        
        # Move robot back to original position.
        if state == enums.STATES.START:
          pass
        # If success or failure, hide cursor until robot repositioned
        elif state == enums.STATES.ERROR or state == enums.STATES.FAIL:
          self.tgt_window.color = colorMod(BRGDY)
          self.cursor.width = 0
        elif state == enums.STATES.SUCCESS:
          self.tgt_window.color = colorMod(GREEN) 
          self.cursor.wdith = 0
        elif state == enums.STATES.REST:
          self.background.color = colorMod(GREY)
          self.cursor.width = 80 # 50 if active
          self.tgt_window.color = colorMod(GREY)
        elif state == enums.STATES.RESET:
          pass
        

    # This function probably should never be used...
    def outsideOfBar(self):
        point = Point(self.cursor.v[0], self.cursor.v[1])
        polygon = Shapelygon(self.position_bar.v)
        return polygon.contains(point)

    # Move the cursor to a new location
    def moveCursor(self, circle, x, y):
        circle.x = x
        circle.y = y

    #Rotate a shape about origin
    def rotate(self, polygon, angle, origin):
        top, left = origin[0]
        bot, right = origin[2]
        
        cx = self.width*0.5
        cy = self.height*0.5
        center = (cx, cy)
        
        topL = self.rotatePoint(origin[0], angle, center)
        botL = self.rotatePoint(origin[1], angle, center)
        botR = self.rotatePoint(origin[2], angle, center)
        topR = self.rotatePoint(origin[3], angle, center)
        
        rotated = [topL, botL, botR, topR]
        # print "rotating"
        polygon.v = rotated
        # self.resizePolygon(self.position_bar)
        if topL[0] > 0:
          print rotated
        return rotated


    def reset_score(self):
        self.score = 0
        self.score_force_level = 0
        self.score_target_dist = 999
        self.score_force_mult = 0
        self.score_target_mult = 0
        self.score_txt.text = "Score: %d" % self.score
        self.reward_txt.text = ""


    def update_reward(self):
        reward = self.score_force_mult * self.score_target_mult
        pts = "points"
        if reward == 1:
            pts = "point"
        self.reward_txt.text = "Current Reward: %d %s" % (reward, pts)


    def increment_score(self):
        self.score += 1 #self.score_force_mult * self.score_target_mult
        self.score_txt.text = "Score: %d" % self.score


    def screen_off(self):
        self.color = (0, 0, 0)
        self.blank_display = True


    def screen_on(self):
        self.blank_display = False
        self.color = (180, 180, 180)


    def update(self, dt):
        global currentAngle
        print "updating...."
        #bp()
        while True:
            time.sleep(1)
            rcv = self.mod.ReadMessage(self.msg, 0)
            currentAngle = (currentAngle + 45) % 360
            #self.rotatePositionBar(currentAngle)
            if rcv == 1:
                hdr = self.msg.GetHeader()
                msg_type = hdr.msg_type

                if msg_type == rc.MT_PING:
                    self.reset_score()

                # -------------- Actual messages -----------------
                
                # Get cursor position from BURT
                elif msg_type == rc.MT_BURT_STATUS:
                    mdf = rc.MDF_INPUT_DOF_DATA()
                    copy_from_msg(mdf, self.msg)
                    x, y = mapBurt2Display(mdf.pos_x, mdf.pos_y, mdf.pos_z)
                    self.moveCursor(x, y)
                    
                    # change state to show success or failure
                    if mdf.task_complete:
                        if mdf.task_success:
                            #self.setState(mdf, state)
                            pass
                        else:
                            #self.setState(mdf, state)
                            pass
                            
                
                
                # Get state information from Exec
                elif msg_type == rc.MT_TASK_STATE_CONFIG:
                    mdf = rc.MDF_INPUT_DOF_DATA()
                    copy_from_msg(mdf, self.msg)
                    #self.setState(mdf, mdf.state)
                
                # ------------------------------------------------
                
                elif msg_type == rc.MT_INPUT_DOF_DATA:
                    mdf = rc.MDF_INPUT_DOF_DATA()
                    copy_from_msg(mdf, self.msg)

                    if mdf.tag == 'carduinoIO':
                        fdbk = 5 - mdf.dof_vals[7]  # invert to match phyiscal setup
                        x_pos = int((fdbk * (MAX_WIDTH - 2*OFFSET))/5.0)
                        
                        x_pos += 20
                        
                        self.pos_fdbk_txt.text = "%.2f V" % fdbk
                        
                        self.pos_fdbk.v[0] = (x_pos, 405)
                        self.pos_fdbk.v[1] = (x_pos, 615)
                        self.pos_fdbk.v[2] = (x_pos+8, 615)
                        self.pos_fdbk.v[3] = (x_pos+8, 405)
                        

                 # if msg_type == rc.MT_FORCE_SENSOR_DATA:
                 #     mdf = rc.MDF_FORCE_SENSOR_DATA()
                 #     copy_from_msg(mdf, self.msg)
                 #     x_fdbk = mdf.data[0]
                 #     x_fdbk_width = int((x_fdbk / MAX_FDBK) * MAX_WIDTH)
    
                # updates real time position of handle on screen 
                # receives messages from cube_sphere while loop
                elif msg_type == rc.MT_RT_POSITION_FEEDBACK: 
                    mdf = rc.MDF_RT_POSITION_FEEDBACK()
                    copy_from_msg(mdf, self.msg)

                    x_pos = mdf.distanceFromCenter;
                    
                    x_pos += 20

                    self.pos_fdbk.v[0] = (x_pos, 405)
                    self.pos_fdbk.v[1] = (x_pos, 615)
                    self.pos_fdbk.v[2] = (x_pos+8, 615)
                    self.pos_fdbk.v[3] = (x_pos+8, 405)
                    self.resizePolygon(self.pos_fdbk)
                    self.transformPolygon(self.pos_fdbk, self.transformationType)
                    
                elif msg_type == rc.MT_COMBO_WAIT:
                    mdf = rc.MDF_COMBO_WAIT()
                    copy_from_msg(mdf, self.msg)
                
                    print mdf.duration
                
                    duration = mdf.duration / 1000      # convert to seconds
                    self.timer_sec = duration % 60
                    self.timer_min = duration / 60

                    self.screen_off()
                    self.schedule_interval(self.timer_count_down, 1)
                
                elif msg_type == rc.MT_TRIAL_CONFIG:
                    self.unschedule(self.timer_count_down)
                    self.combo_wait_txt.text = ''
                    self.screen_on()
                    
                elif msg_type == rc.MT_END_TASK_STATE:
                    mdf = rc.MDF_END_TASK_STATE()
                    #copy_from_msg(mdf, self.msg)
                    read_msg_data(mdf, self.msg)
        
                    print mdf.id, mdf.outcome
        
                    if (mdf.id == REWARD_TS) and (mdf.outcome == 1):
                        self.increment_score()

                    if (mdf.id in [2, 3, 4]) and (mdf.outcome == 0):
                        print "screen off"
                        self.screen_off()

                #elif msg_type == rc.MT_TASK_STATE_CONFIG:
                 #   mdf = rc.MDF_TASK_STATE_CONFIG()
                 #   copy_from_msg(mdf, self.msg)

                 #   if mdf.background_color == 'gray':
                 #       self.color = (180, 180, 180)
                 #   elif mdf.background_color == 'red':
                 #       self.color = (150, 12, 12)
                 #   elif mdf.background_color == 'green':
                 #       self.color = (0, 150, 50)

                 #   if mdf.fdbk_display_color == 'gray':
                 #       self.tgt_window.color = (0.3, 0.3, 0.3, 1)
                 #   elif mdf.fdbk_display_color == 'yellow':
                 #       self.increment_score()
                 #       self.tgt_window.color = (0.5, 0.5, 0.0, 1)
                 #   elif mdf.fdbk_display_color == 'green':
                 #       self.tgt_window.color = (0.0, 0.6, 0.2, 1)
                 #   elif mdf.fdbk_display_color == 'red':
                 #       self.tgt_window.color = (0.6, 0.05, 0.05, 1)
                 #   
                 #   if not math.isnan(mdf.direction) and mdf.direction in range(-1,3) and not mdf.direction == self.transformationType:
                 #       self.position_bar.v = FEEDBACK_ORIGIN
                 #       self.resizePolygon(self.position_bar)
                 #       self.transformPolygon(self.position_bar, mdf.direction)
                 #       self.transformPolygon(self.pos_fdbk, mdf.direction)
                 #       self.transformationType = mdf.direction
                 #   
                 #   if not(math.isnan(mdf.target[0])) and not(math.isnan(mdf.target[1])):
                 #       x_tgt_lo = mdf.target[0] + 20
                 #       x_tgt_hi = mdf.target[1] + 20
                 #       
                 #       self.tgt_window.v[0] = (x_tgt_lo, 430)
                 #       self.tgt_window.v[1] = (x_tgt_lo, 590)
                 #       self.tgt_window.v[2] = (x_tgt_hi, 590)
                 #       self.tgt_window.v[3] = (x_tgt_hi, 430)
                 #       self.resizePolygon(self.tgt_window)
                 #       self.transformPolygon(self.tgt_window, self.transformationType)
            else:
                self.rotate(self.position_bar, math.radians(currentAngle), POSITION_ORIGIN)
                self.rotate(self.tgt_window, math.radians(currentAngle), TARGET_ORIGIN)
                break

    # Convert point from BURT to appropriate displayable position
    def mapBurt2Display(self, x, y, z = 0):
        return (x, y)

    def transformPolygon(self, polygon, transformationType):
        for i in xrange(4):
            polygon.v[i] = self.transformPoint(polygon.v[i], transformationType)


    def transformPoint(self, input_point, transformationType):
        # tt values are determined by adding XorYorZ and UpOrDown
        if transformationType == 0: # goes from left to right XorYorZ=1 UpOrDown=-1 
            return input_point
        elif transformationType == 2: # goes from right to left XorYorZ=1 UpOrDown=1
            return (self.width - input_point[0], input_point[1])
        elif transformationType == -1: # backward XorYorZ=0 UpOrDown=-1
            return (input_point[1], self.width - input_point[0])
        elif transformationType == 1: # forwards XorYorZ=0 UpOrDown=1
            return (input_point[1], input_point[0])
        else:
            print "unknown transform type"

    #def on_key_release( self, keys, mod ):


    def draw(self):
        super( Display, self).draw()
        #bp()

        if not self.blank_display:
            self.background.draw()
            self.position_bar.render()
            self.tgt_window.render()
            #self.pos_fdbk.render()
            self.cursor.render()
            #self.pos_fdbk_txt.draw()
            self.score_txt.draw()
            self.reward_txt.draw()

        self.combo_wait_txt.draw()
        


if __name__ == "__main__":
    director.init(width=700, height=800, caption="FeedbackDisplay", resizable=True, do_not_scale=True) #, fullscreen=True)

    scene = Scene( Display() )
    
    director.run( scene )

