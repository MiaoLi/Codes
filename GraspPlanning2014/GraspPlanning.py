#!/usr/bin/python

from openravepy import *
from openravepy import misc
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import time
from MiaoFunc import *

objGrasp = 'spray'
robotFile = './Models/allegroTac.robot.xml'

env = Environment() # create openrave environment
env.SetDebugLevel(DebugLevel.Fatal)
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load(robotFile) # load 16 dof allegro hand
robot = env.GetRobots()[0] # get the first robot
obj = env.ReadKinBodyXMLFile('./Models/' + objGrasp + '.kinbody.xml')
env.Add(obj)
hand=env.GetKinBody('AllegroHand')
time.sleep(0.1)



handles = []
handPose = array([[1, 0,0 , 0.1],[0, 1 ,0 , 0],[0, 0, 1 , 0],[0, 0, 0 , 1]]) 


with env: # lock the environment since robot will be used
	print("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\n"+repr(robot.GetDOFValues()))
	robot.SetDOFValues([0.5],[0]) # set joint 0 to value 0.5	
	T = robot.GetLinks()[0].GetTransform() # get the transform of link 1
	print("The transformation of link 1 is:\n"+repr(T))
	Ttmp = robot.GetLinks()[0].GetTransform()
	htmp = PlotTransformation(Ttmp,env,0.05)
	handles.append(htmp)

	# plot all the coordinate frame:
	while False:	
		for i in range(len(robot.GetLinks())):
			Ttmp = robot.GetLinks()[i].GetTransform()
			htmp = PlotTransformation(Ttmp,env,0.05)
			handles.append(htmp)
	
	robot.SetTransform(handPose)
	env.UpdatePublishedBodies()
	time.sleep(0.01)


raw_input('press Enter to exit')