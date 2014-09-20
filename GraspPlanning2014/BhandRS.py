#!/usr/bin/python
"""
miao.li@epfl.ch
This file is used to generate the hand configuration and 
the encoder for hand inverse kinematics

"""

from openravepy import *
import random
import math, time
from numpy import *
from openravepy import misc
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import time
from MiaoFunc import *

vis = False

v = raw_input("Visualization? : [y/n]\n")
if (v == 'y'):
	vis = True

objname = 'spray'

env = Environment() # create openrave environment
if (vis):
	env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('./Models/barretthand.robot.xml') # load 4 dof barrett hand
robot = env.GetRobots()[0] # get the first robot
# obj = env.ReadKinBodyXMLFile('./Models/' + objGrasp + '.kinbody.xml')
# env.Add(obj)


time.sleep(0.1)

nbSample = 1000000


handles = []
BHandData = zeros((nbSample,22))

with env: # lock the environment since robot will be used
		lower,upper = robot.GetActiveDOFLimits()
		for i in range(nbSample):
			print i
			JntTarget = numpy.random.rand(len(lower))*(upper-lower)+lower
			#print JntTarget
			robot.SetDOFValues([JntTarget[0]],[0])
			robot.SetDOFValues([JntTarget[1]],[1])
			robot.SetDOFValues([JntTarget[2]],[2])
			robot.SetDOFValues([JntTarget[3]],[3])
			T0 = robot.GetLinks()[3].GetGlobalMassFrame()
			T1 = robot.GetLinks()[6].GetGlobalMassFrame()
			T2 = robot.GetLinks()[8].GetGlobalMassFrame()

			T0[0:3,3] = T0[0:3,3] + 0.01*T0[0:3,1]+0.025*T0[0:3,0];
			T1[0:3,3] = T1[0:3,3] + 0.01*T1[0:3,1]+0.025*T1[0:3,0];
			T2[0:3,3] = T2[0:3,3] + 0.01*T2[0:3,1]+0.025*T2[0:3,0];

			BHandData[i,0:4] = JntTarget
			BHandData[i,4:7] = T0[0:3,3]
			BHandData[i,7:10] = T1[0:3,3]
			BHandData[i,10:13] = T2[0:3,3]
			BHandData[i,13:16] = T0[0:3,1]
			BHandData[i,16:19] = T1[0:3,1]
			BHandData[i,19:22] = T2[0:3,1]

			htmp1 = PlotTransformation(T0,env,0.05)
			htmp2 = PlotTransformation(T1,env,0.05)
			htmp3 = PlotTransformation(T2,env,0.05)
			handles.append(htmp1)
			handles.append(htmp2)
			handles.append(htmp3)
			handles = []

			env.UpdatePublishedBodies()
			time.sleep(0.0001)

numpy.savetxt('BHandData.txt',BHandData)

raw_input('press Enter to exit')
