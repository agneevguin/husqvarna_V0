#! /usr/bin/env python
import rospy, math
import numpy as np
import sys, termios, tty, select, os
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
 
class KeyTeleop(object):
  cmd_bindings = {'q':np.array([1,1]),
                  'w':np.array([1,0]),
                  'e':np.array([1,-1]),
                  'a':np.array([0,1]),
                  'd':np.array([0,-1]),
                  'z':np.array([-1,-1]),
                  'x':np.array([-1,0]),
                  'c':np.array([-1,1]),
                  's':np.array([0,0])
                  }
  def init(self):
    # Save terminal settings
    self.settings = termios.tcgetattr(sys.stdin)
    # Initial values
    self.inc_ratio = 0.1
    self.speed = np.array([0.3, 1.0])
    self.command = np.array([0, 0])
    self.update_rate = 10   # Hz
    self.alive = True
    # Setup publishers
    self.pub_twist = rospy.Publisher('/cmd_vel', Twist)
    self.pub_mode = rospy.Publisher('/cmd_mode', UInt16)
    self.sim_cmd = rospy.Publisher('/sim_cmd', UInt16)
    self.beacon_cmd = rospy.Publisher('/beacon_cmd', UInt16)
    self.confinement_mode = rospy.Publisher('/confinement_cmd', UInt16)
    
    self.searching = False
    
    self.shapeNum = 0x20
 
  def fini(self):
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
 
  def run(self):
    try:
      self.init()
      self.print_usage()
      r = rospy.Rate(self.update_rate) # Hz
      while not rospy.is_shutdown():
        ch = self.get_key()
        self.process_key(ch)
        self.update()
        r.sleep()
    except rospy.exceptions.ROSInterruptException:
      pass
    finally:
      self.fini()
 
  def print_usage(self):
    msg = """
    Keyboard Teleop that Publish to /cmd_vel (geometry_msgs/Twist)
    --------------------------------------------------
    H:       Print this menu
    Moving around:
      Q   W   E
      A   S   D
      Z   X   C
 
    G :   Quit
    --------------------------------------------------
    """
    self.loginfo(msg)
    self.show_status()
 
  # Used to print items to screen, while terminal is in funky mode
  def loginfo(self, str):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    print(str)
    tty.setraw(sys.stdin.fileno())
 
  # Used to print teleop status
  def show_status(self):
    msg = 'Status:\tlinear %.2f\tangular %.2f' % (self.speed[0],self.speed[1])
    self.loginfo(msg)
 
  # For everything that can't be a binding, use if/elif instead
  def process_key(self, ch):
	#
	# AM_DRIVER COMMANDS
	#
    if ch == 'h':
      self.print_usage()
    elif ch in self.cmd_bindings.keys():
      self.command = self.cmd_bindings[ch]
    elif ch == 'g':
      self.loginfo('Quitting')
      # Stop the robot
      twist = Twist()
      self.pub_twist.publish(twist)
      rospy.signal_shutdown('Shutdown')
    elif ch == '1':
      # Systematic mode
      mode = UInt16()
      mode.data = 0x90
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == '2':
      # Random mode
      mode = UInt16()
      mode.data = 0x91
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == 'j':
      # Cutting disc ON
      mode = UInt16()
      mode.data = 0x93
      self.pub_mode.publish(mode)
    elif ch == 'k':
      # Cutting disc OFF
      mode = UInt16()
      mode.data = 0x92
      self.pub_mode.publish(mode)
    elif ch == 'v':
      # Load Beacons
      mode = UInt16()
      mode.data = 0x0A
      self.beacon_cmd.publish(mode)
    elif ch == 'b':
      # Save Beacons
      mode = UInt16()
      mode.data = 0x09
      self.beacon_cmd.publish(mode)
    elif ch == 'n':
      # Load Confinement
      mode = UInt16()
      mode.data = 0x0A
      self.confinement_mode.publish(mode)
    elif ch == 'm':
      # Save Confinement
      mode = UInt16()
      mode.data = 0x09
      self.confinement_mode.publish(mode)
    #elif ch == 'n':
      # Cutting height HIGH
     # mode = UInt16()
     # mode.data = 0x94
     # self.pub_mode.publish(mode)
    #elif ch == 'm':
      # Cutting height LOW
     # mode = UInt16()
      #mode.data = 0x95
      #self.pub_mode.publish(mode)
    #elif ch == 'b':
      # Request SEARCHING (charge)
     # mode = UInt16()
      #if self.searching == False:
       #   mode.data = 0x100
        #  self.searching = True
      #else:
       #   mode.data = 0x101
        #  self.searching = False
      #self.pub_mode.publish(mode)

    #
    # STEERING COMMANDS
    #
    elif ch == '0':
      # MOVETO test
      mode = UInt16()
      mode.data = 0x10
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == '3':
      # Test sequence
      mode = UInt16()
      mode.data = 0x30
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == '4':
      # Simple RANDOM mode
      mode = UInt16()
      mode.data = 0x41
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == '5':
      # BORDER challenge
      mode = UInt16()
      mode.data = 0x42
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == '6':
      # Systematic REPEAT
      mode = UInt16()
      mode.data = 0x21
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == '7':
      # PATH mode (ODOM COMBINED)
      mode = UInt16()
      mode.data = 0x44
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == '8':
      # TURN 90 degs test
      mode = UInt16()
      mode.data = 0x12
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == '9':
      # TURN 90 degs test
      mode = UInt16()
      mode.data = 0x13
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == 'l':
      # Drive L and estimate all visible beacons
      mode = UInt16()
      mode.data = 0x40
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == 'p':
      # Sample for estimation of New Beacon
      mode = UInt16()
      mode.data = 0x5
      self.beacon_cmd.publish(mode)
      self.command = np.array([0, 0])
    elif ch == 'f':
      # Finalize estimatation of New Beacon
      mode = UInt16()
      mode.data = 0x6
      self.beacon_cmd.publish(mode)
      self.command = np.array([0, 0])
	#
	# CONFINEMENT COMMANDS
	#
    elif ch == 'o':
      # Generate a BOX from Beacons using confinement
      mode = UInt16()
      mode.data = 0x01
      self.confinement_mode.publish(mode)
    elif ch == 'y':
      # GENERATE PATH from POLYGON
      mode = UInt16()
      mode.data = 0x04
      self.confinement_mode.publish(mode)
    elif ch == 'u':
      # CLEAR PATH/COVERAGE
      mode = UInt16()
      mode.data = 0x02
      self.confinement_mode.publish(mode)
    elif ch == 'i':
      # INSERT (RECORD) NEW POINT AS POLYGON
      mode = UInt16()
      mode.data = 0x03
      self.confinement_mode.publish(mode)    
    elif ch == 'I':
      # CLOSE POLYGON
      mode = UInt16()
      mode.data = 0x07
      self.confinement_mode.publish(mode)
    elif ch == 't':
      # GENERATE COVERAGE from POLYGON
      mode = UInt16()
      mode.data = 0x05
      self.confinement_mode.publish(mode)
    #elif ch == 'm':
      # TOGGLE MODE for CONFINEMENT
     # mode = UInt16()
     # mode.data = 0x06
     # self.confinement_mode.publish(mode)
    elif ch == 'r':
      # Generate a confinment around the mower
      mode = UInt16()
      mode.data = self.shapeNum
      self.confinement_mode.publish(mode)
      
      self.shapeNum = self.shapeNum + 1
      if self.shapeNum > 0x23:
          self.shapeNum = 0x20
        
    # Simulator commands
    elif ch == ' ':
        mode = UInt16()
        mode.data = 0x01
        self.sim_cmd.publish(mode)
    else:
      self.command = np.array([0, 0])
      

 
  def update(self):
    if rospy.is_shutdown():
      return
    twist = Twist()
    cmd  = self.speed*self.command
    twist.linear.x = cmd[0]
    twist.angular.z = cmd[1]
    self.pub_twist.publish(twist)
 
  # Get input from the terminal
  def get_key(self):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    #return key.lower()
    return key
 
if __name__ == '__main__':
  rospy.init_node('keyboard_teleop')
  teleop = KeyTeleop()
  teleop.run()
