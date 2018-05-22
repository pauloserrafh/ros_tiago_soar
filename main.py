# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sammy Pfeiffer

# System imports
import sys
import time
# ROS imports
import rospy
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
# Soar imports
PATH_TO_SOAR = "/home/user/SOAR/Soar/out"
sys.path.append(PATH_TO_SOAR)
import Python_sml_ClientInterface as sml

SOAR_GP_PATH = "./tiago.soar"

def wait_for_valid_time(timeout):
	"""Wait for a valid time (non-zero), this is important
	when using a simulated clock"""
	# Loop until:
	# * ros master shutdowns
	# * control+C is pressed (handled in is_shutdown())
	# * timeout is achieved
	# * time is valid
	start_time = time.time()
	while not rospy.is_shutdown():
		if not rospy.Time.now().is_zero():
			return
		if time.time() - start_time > timeout:
			rospy.logerr("Timed-out waiting for valid time.")
			exit(0)
		time.sleep(0.1)
	# If control+C is pressed the loop breaks, we can exit
	exit(0)


def get_status_string(status_code):
	return GoalStatus.to_string(status_code)

class SOARInterface:
	def __init__(self):
		pass

	def action(self, comm):
		command_name = comm
		if command_name == "takeoff":
			# out = robot.act(command_name)
			pass

	def sendOK(self):
		return "succeeded"


def define_prohibitions(): #TODISCOVER WTF IS THIS
	pass

def create_kernel():
	kernel = sml.Kernel.CreateKernelInCurrentThread()
	if not kernel or kernel.HadError():
		print kernel.GetLastErrorDescription()
		exit(1)
	return kernel

def create_agent(kernel, name):
	agent = kernel.CreateAgent("agent")
	if not agent:
		print kernel.GetLastErrorDescription()
		exit(1)
	return agent

def agent_load_productions(agent, path):
	agent.LoadProductions(path)
	if agent.HadError():
		print agent.GetLastErrorDescription()
		exit(1)

class Tiago:
	def __init__(self):
		rospy.init_node('run_motion_python')

		rospy.loginfo("Starting run_motion_python application...")
		wait_for_valid_time(10.0)

		self.client = SimpleActionClient('/play_motion', PlayMotionAction)

		rospy.loginfo("Waiting for Action Server...")
		self.client.wait_for_server()

	def act(self, action):
		goal = PlayMotionGoal()
		goal.motion_name = action
		goal.skip_planning = False
		goal.priority = 0  # Optional

		rospy.loginfo("Sending goal with motion: " + goal.motion_name)
		self.client.send_goal(goal)

		rospy.loginfo("Waiting for result...")
		action_ok = self.client.wait_for_result(rospy.Duration(30.0))

		state = self.client.get_state()

		if action_ok:
			rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
		else:
			rospy.logwarn("Action failed with state: " + str(get_status_string(state)))

	def reset(self):
		goal = PlayMotionGoal()
		goal.motion_name = 'home'
		goal.skip_planning = False
		goal.priority = 0  # Optional

		rospy.loginfo("Sending goal with motion: " + goal.motion_name)
		self.client.send_goal(goal)

		rospy.loginfo("Waiting for result...")
		action_ok = self.client.wait_for_result(rospy.Duration(30.0))

		state = self.client.get_state()

		if action_ok:
			rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
		else:
			rospy.logwarn("Action failed with state: " + str(get_status_string(state)))



if __name__ == '__main__':
	soar_interface = SOARInterface()
	
	# print "******************************\n******************************\nNew goal\n******************************\n******************************\n"
	kernel = create_kernel()
	agent = create_agent(kernel, "agent")
	agent_load_productions(agent,SOAR_GP_PATH)
	agent.SpawnDebugger()

	kernel.CheckForIncomingCommands()

	time.sleep(10)

	pInputLink = agent.GetInputLink()
	pID = agent.CreateIdWME(pInputLink, "helloworld")


	agent.Commit()
	agent.RunSelfTilOutput()
	agent.Commands()
	numberCommands = agent.GetNumberCommands()
	print "Number of commands received by the agent: %s" % (numberCommands)
	if (numberCommands):
		command = agent.GetCommand(0)
		command_name = command.GetCommandName()
		print("command: ")
		print(command)
		print("command_name: ")
		print(command_name)

		robot = Tiago()
		robot.act(action = "wave")
		robot.reset()
	else:
		print("Error. No comamands received.")

	kernel.DestroyAgent(agent)
	kernel.Shutdown()
	#del kernelCommit

