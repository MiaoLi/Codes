#!/usr/bin/python

from openravepy import *
import random
import math, time
from numpy import *
from openravepy import misc
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import time, threading
from collections import Counter

vis = False

def vector(v):
	return numpy.array(v)

class PlotSpinner(threading.Thread):
    def __init__(self,handle):
        threading.Thread.__init__(self)
        self.starttime = time.time()
        self.handle=handle
        self.ok = True
    def run(self):
        while self.ok:
            self.handle.SetShow(bool(mod(time.time()-self.starttime,2.0) < 1.0))
            # time.sleep(0.0001)


def encodePair(pos1, pos2, norm1, norm2):
	code1 = vector(pos1) - vector(pos2)
	code1 = numpy.linalg.norm(code1)
	code2 = vector(norm1) - vector(norm2)
	code2 = numpy.linalg.norm(code2)
	return [code1, code2]



def encode(pos1, pos2, pos3, norm1, norm2, norm3): # currently encode 3-fingered hand only
	pair1 = encodePair(pos1,pos2, norm1, norm2)
	pair2 = encodePair(pos1,pos3, norm1, norm3)
	pair3 = encodePair(pos2,pos3, norm2, norm3)
	code = [pair1[0], pair1[1], pair2[0], pair2[1], pair3[0], pair3[1]]
	return code


v = raw_input("Visualization? : [y/n]\n")
if (v == 'y'):
	vis = True

env = Environment() # create openrave environment
if (vis):
	env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('./Models/barretthand.robot.xml') # load 4 dof barrett hand
robot = env.GetRobots()[0] # get the first robot
# obj = env.ReadKinBodyXMLFile('./bunny.kinbody.xml')
# env.Add(obj)

time.sleep(0.1)


steps = 40
maxAngle = 90
length = .04
pointSize = .006



data = []
conf = []


with env: # lock the environment since robot will be used
	with robot:
		
		for i in range(steps):
			print ("sampled points on the first dof: %d\n") % (i+1)
			# for j in range(steps):
			j = i
			for k in range(steps):
				for l in range(steps):
					raw_input('press enter to continue')
					robot.SetDOFValues([i*(maxAngle/(steps-1))/180.0*math.pi],[0])
					robot.SetDOFValues([j*(maxAngle/(steps-1))/180.0*math.pi],[1])
					robot.SetDOFValues([k*(maxAngle/(steps-1))/180.0*math.pi],[2])
					robot.SetDOFValues([l*(100/(steps-1))/180.0*math.pi],[3])
					T0 = robot.GetLinks()[3].GetGlobalMassFrame()
					T1 = robot.GetLinks()[6].GetGlobalMassFrame()
					T2 = robot.GetLinks()[8].GetGlobalMassFrame()
					# print type(x)

					T0[0:3,3] = T0[0:3,3] + 0.01*T0[0:3,1];
					T1[0:3,3] = T1[0:3,3] + 0.01*T1[0:3,1];
					T2[0:3,3] = T2[0:3,3] + 0.01*T2[0:3,1];

					code = encode(T0[0:3,3], T1[0:3,3], T2[0:3,3], T0[0:3,1], T1[0:3,1], T2[0:3,1])

					# code = code.tolist()
					data.append(str(code).strip('[]') + "\n")
					confTmp = str(i*(maxAngle/(steps-1))/180.0*math.pi) + " " + str(j*(maxAngle/(steps-1))/180.0*math.pi) + " "	+ str(k*(maxAngle/(steps-1))/180.0*math.pi) + " " + str(l*(100/(steps-1))/180.0*math.pi)
					# dv = dirVec(T0[0:3,3], T1[0:3,3], T2[0:3,3]) # has to be normalized
					# confTmp = confTmp + " " + str(dv).strip('[]') + str(T2[0:3,3]).strip('[]') + "\n"
					confTmp = confTmp + " " + str(T0[0:3,3]).strip('[]') + str(T1[0:3,3]).strip('[]') + str(T2[0:3,3]).strip('[]') + "\n"
					# print confTmp
					conf.append(confTmp)
					if (vis):
						try:
							handles = []

							handles.append(env.plot3(points=T0[0:3,3],
							                           pointsize=pointSize,
							                           colors=array((1,0,0)),drawstyle=1))
							handles.append(env.drawarrow(p1=T0[0:3,3],p2=T0[0:3,3]+length*T0[0:3,1],linewidth=0.004,color=[1.0,0.0,0.0]))

							handles.append(env.plot3(points=T1[0:3,3],
							                           pointsize=pointSize,
							                           colors=array((0,1,0)),drawstyle=1))
							handles.append(env.drawarrow(p1=T1[0:3,3],p2=T1[0:3,3]+length*T1[0:3,1],linewidth=0.004,color=[1.0,0.0,0.0]))

							handles.append(env.plot3(points=T2[0:3,3],
							                           pointsize=pointSize,
							                           colors=array((0,0,1)),drawstyle=1))
							handles.append(env.drawarrow(p1=T2[0:3,3],p2=T2[0:3,3]+length*T2[0:3,1],linewidth=0.004,color=[0.0,0.0,1.0]))

							handles.append(env.drawlinestrip(points=array((T0[0:3,3],T1[0:3,3])),
                                           linewidth=4.0,
                                           colors=array(((0,0,0)))))
							handles.append(env.drawlinestrip(points=array((T0[0:3,3],T2[0:3,3])),
                                           linewidth=4.0,
                                           colors=array(((0,0,0)))))
							handles.append(env.drawlinestrip(points=array((T1[0:3,3],T2[0:3,3])),
                                           linewidth=4.0,
                                           colors=array(((0,0,0)))))


							env.UpdatePublishedBodies()
							spinner = PlotSpinner(handles[-1])
							spinner.start()

							time.sleep(0.0001)
						finally:
						    if spinner is not None:
						        spinner.ok = False
						        # raw_input("press enter to continue")

		raw_input("press enter to finish subsampling")


f = open("code.dat", "w+")
f.writelines(data)
f.close()

f = open("conf.dat", "w+")
f.writelines(conf)
f.close()

# raw_input("Press Enter to Exit")
