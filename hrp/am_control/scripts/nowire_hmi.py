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
    self.app_cmd_pub = rospy.Publisher('/app_cmd', UInt16)
    
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
    output: /cmd_vel (geometry_msgs/Twist)
            /app_cmd (std_msgs/UInt16)
    --------------------------------------------------
                      'l': start beacon estimation
    Moving around:    'o': start auto confinement recording
      Q   W   E       'O': pause auto confinement recording
      A   S   D       'i': add confinement point
      Z   X   C       'I': close confinement polygon
                      'u': clear all confinents
                      'p': save installation
                      'P': start mowing
                      'k': pause (enter manual mode) / resume mowing
                      'G': quit
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
    elif ch == 'l':
      # Drive L and estimate all visible beacons
      mode = UInt16()
      mode.data = 0x40
      self.app_cmd_pub.publish(mode)
      self.command = np.array([0, 0])
    elif ch == 'u':
      # CLEAR PATH/COVERAGE
      mode = UInt16()
      mode.data = 0x02
      self.app_cmd_pub.publish(mode)    
    elif ch == 'o':
      # Start Auto Recording confinement
      mode = UInt16()
      mode.data = 0x06
      self.app_cmd_pub.publish(mode) 
    elif ch == 'O':
      # Pause Auto Recording confinement
      mode = UInt16()
      mode.data = 0x08
      self.app_cmd_pub.publish(mode)
    elif ch == 'i':
      # INSERT (RECORD) NEW POINT AS POLYGON
      mode = UInt16()
      mode.data = 0x03
      self.app_cmd_pub.publish(mode)    
    elif ch == 'I':
      # Start auto or close polygon
      mode = UInt16()
      mode.data = 0x07
      self.app_cmd_pub.publish(mode)
    elif ch == 'p':
      # save installation
      mode = UInt16()
      mode.data = 0x09
      self.app_cmd_pub.publish(mode)
    elif ch == 'P':
      # start mowing
      mode = UInt16()
      mode.data = 0x0B
      self.app_cmd_pub.publish(mode)  
    elif ch == 'k':
      # pause / resume mowing
      mode = UInt16()
      mode.data = 0x0C
      self.app_cmd_pub.publish(mode)        
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
  rospy.init_node('nowire_hmi')
  teleop = KeyTeleop()
  teleop.run()
