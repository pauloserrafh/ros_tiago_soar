#!/usr/bin/env python

import time
# from PySide import QtCore, QtGui
import sys

PATH_TO_SOAR = "/home/user/SOAR/Soar/out"
sys.path.append(PATH_TO_SOAR)
import Python_sml_ClientInterface as sml

#path = PATH_TO_SOAR
#sys.path.append(path)

SOAR_GP_PATH = "./tiago.soar"

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

if __name__ == '__main__':
	import sys
	soar_interface = SOARInterface()

	print "******************************\n******************************\nNew goal\n******************************\n******************************\n"
	first_time = time.time()
	kernel = create_kernel()
	agent = create_agent(kernel, "agent")
	agent_load_productions(agent,SOAR_GP_PATH)
	agent.SpawnDebugger()

	# p_cmd = 'learn --on'
	# res = agent.ExecuteCommandLine(p_cmd)
	# res = kernel.ExecuteCommandLine(p_cmd, agent.GetAgentName)
	kernel.CheckForIncomingCommands()
	# p_cmd = 'watch --learning 2'
	# res = agent.ExecuteCommandLine(p_cmd)
	# print str(res)

	goal_achieved = False
	time.sleep(10)

	pInputLink = agent.GetInputLink()
	pID = agent.CreateIdWME(pInputLink, "helloworld")

	# wme1 = agent.CreateIntWME(pID, "diff1", a)

	# if(r==1):
	# 	wme1 = agent.CreateIntWME(pID, "diff1", a)
	# if(r==0):
	# 	wme1 = agent.CreateIntWME(pID, "diff1", 1)


		# wme2 = agent.CreateIntWME(pID, "diff2", b)
		# wme3 = agent.CreateIntWME(pID, "altd", c)
		# wme3 = agent.CreateIntWME(pID, "decision", e)

		# wme4 = agent.CreateIntWME(pID, "valtd", d)

	agent.Commit()
	agent.RunSelfTilOutput()
	agent.Commands()
	numberCommands = agent.GetNumberCommands()
	print "Number of commands received by the agent: %s" % (numberCommands)
	command = agent.GetCommand(0)
	command_name = command.GetCommandName()
	print("command: ")
	print(command)
	print("command_name: ")
	print(command_name)

	kernel.DestroyAgent(agent)
	kernel.Shutdown()
	#del kernelCommit
