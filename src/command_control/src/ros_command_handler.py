#!/usr/bin/env python
from joystick_handler import JoystickHandler, IDLE, MANUAL, AUTONOMOUS
import rospy

from geometry_msgs.msg import Twist


class ROSCommandHandler:
    def __init__(self):
	self._autonomy_cmd = Twist()
	self._joystick_handler = JoystickHandler()
	self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
	rospy.init_node('command_control', anonymous=True)
	rospy.Subscriber('/path_tracker/cmd', Twist, self.autonomy_callback)

    def publish(self):
	if self._joystick_handler.get_drive_mode() == IDLE:
	    rospy.loginfo("Command control in Idle drive mode")
	    return
	elif self._joystick_handler.get_drive_mode() == MANUAL:
	    rospy.loginfo("Command control in manual drive mode")
	    msg = self.transform_joystick_cmd_to_ros_msg()
        elif self._joystick_handler.get_drive_mode() == AUTONOMOUS:
	    rospy.loginfo("Command control in autonomous drive mode")
	    msg = self._autonomy_cmd
	else:
	    rospy.logerr("Joystick has an unknown drive mode")
	    return
	self.pub_cmd_vel.publish(msg)

    def transform_joystick_cmd_to_ros_msg(self):
	cmd = self._joystick_handler.get_joystick_command()
	cmd_forward = cmd[0]
	cmd_turn = cmd[1]
	velocity = self._joystick_handler.get_velocity()
	print ('{} {}'.format(cmd_forward,cmd_turn))
	msg = Twist()
	msg.linear.x = cmd_forward * velocity
	msg.angular.z = cmd_turn * -1 * velocity
	return msg

    def autonomy_callback(self, msg):
	self._autonomy_cmd = msg

    def spin(self):
	while not rospy.is_shutdown():
	    self._joystick_handler.read_joystick()
	    rospy.sleep(0.05)
	    self.publish()

