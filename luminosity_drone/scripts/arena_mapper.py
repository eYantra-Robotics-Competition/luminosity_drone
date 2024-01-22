#!/usr/bin/env python3

import rospy
import roslib
from std_msgs.msg import Int8, String
import math
import time
import csv
import threading
from geometry_msgs.msg import PoseArray
import sys
import json

#Defining a class
class Record_Coordinates():

	def __init__(self):
		rospy.init_node('whycon_record_nodes',anonymous=False) # initializing a ros node with name marker_detection
		self.whycon_marker = [0,0,0]
		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
		self.number_of_nodes = 25
		self.block_name_list = ["A1", "B1", "C1", "D1", "E1", "E2", "D2", "C2", "B2", "A2", "A3", "B3", "C3", "D3", "E3", "E4", "D4", "C4", "B4", "A4", "A5", "B5", "C5", "D5", "E5"]
		self.pose_list = []
		self.current_index = None

	# Callback for /whycon/poses
	def whycon_data(self,msg):
		pos_x = round(msg.poses[0].position.x,3)
		pos_y = round(msg.poses[0].position.y,3)
		pos_z = round(msg.poses[0].position.z,3)
		self.whycon_marker = [pos_x,pos_y,pos_z]

	def keypress_thread(self):
		while True:
			text = input("Press Enter to lock.")
			if not text:
				break

	def query_yes_no(self, question, default=None):
		"""Ask a yes/no question via input() and return their answer.

		"question" is a string that is presented to the user.
		"default" is the presumed answer if the user just hits <Enter>.
		It must be "yes" (the default), "no" or None (meaning
		an answer is required of the user).

		The "answer" return value is True for "yes" or False for "no".
		"""
		valid = {"yes": True, "y": True, "ye": True,"no": False, "n": False}
		if default is None:
			prompt = " [Y/N]:  "
		elif default == "yes":
			prompt = " [Y/N]:  "
		elif default == "no":
			prompt = " [Y/N]:  "
		else:
			raise ValueError("invalid default answer: '%s'" % default)

		while True:
			sys.stdout.write(question + prompt)
			choice = input().lower()
			if default is not None and choice == '':
				return valid[default]
			elif choice in valid:
				return valid[choice]
			else:
				sys.stdout.write("Please respond with 'yes' or 'no' ""(or 'y' or 'n').\n")

	def display_and_lock_whycon_poses(self, index):
		while True:
			print("\n\nRecord the WhyCon coordinate for: "+self.block_name_list[index]+". Press ENTER to lock.\n")
			t1 = threading.Thread(target=self.keypress_thread , args=())
			t1.start()
			while t1.is_alive():
				sys.stdout.write('\rwhy_x: '+'{0:06.3f}'.format(self.whycon_marker[0])+'   why_y: '+'{0:06.3f}'.format(self.whycon_marker[1])+'   why_z: '+'{0:06.3f}'.format(self.whycon_marker[2]))
				sys.stdout.flush()
				whycon_marker_locked = list(self.whycon_marker)
				time.sleep(0.2)
			text_input = input("You have chosen the above pose for "+self.block_name_list[index]+". Press ENTER to commit, q to exit or any other key to retake value: ")
			if not text_input:
				self.pose_list.append(whycon_marker_locked)
				break
			elif text_input == 'q':
				sys.exit()

	def input_position_config(self):
		for i in range(self.number_of_nodes):
			self.display_and_lock_whycon_poses(i)
	
	def write_config_to_json(self, file_path = 'arena_mapper.json'):
		pose_dict = dict(zip(self.block_name_list, self.pose_list))
		with open(file_path, mode='w') as outfile:
			json.dump(pose_dict, outfile)
		print("Recording Successful")

if __name__=="__main__":
	try:
		rec = Record_Coordinates()
		rec.input_position_config()
		rec.write_config_to_json()
	except KeyboardInterrupt:
		print("Recording Config Interrupted")
