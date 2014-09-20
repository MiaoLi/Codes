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
import time, threading
from MiaoFunc import *
from numpy.linalg import inv

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



fh = open( "FeasibleGrasp.txt" );
FeasibleGrasp = []
for line in fh.readlines():
    y = [value for value in line.split()]
    FeasibleGrasp.append( y )
fh.close()
print(len(FeasibleGrasp))

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('./Models/barretthand.robot.xml') # load 4 dof barrett hand
robot = env.GetRobots()[0] # get the first robot
time.sleep(0.1)

handPose = array([[1, 0,0 , 0],[0, 1 ,0 , 0],[0, 0, 1 , 0],[0, 0, 0 , 1]]) 
Graspidx = 0

handles = []
infocylinder = KinBody.Link.GeometryInfo()
infocylinder._type = KinBody.Link.GeomType.Cylinder
infocylinder._t[0,3] = 0.0
infocylinder._vGeomData = [0.035,0.13]
infocylinder._bVisible = True
infocylinder._fTransparency = 0.4
infocylinder._vDiffuseColor = [0.827451,0.827451,0.827451]
k3 = RaveCreateKinBody(env,'')
k3.InitFromGeometries([infocylinder])
k3.SetName('tempcylinder')
env.Add(k3,True)
time.sleep(0.1)

with env:
	for i in range(100):
		Graspidx = random.randint(0, len(FeasibleGrasp))
		raw_input('press enter to continue')
		handPose = array([[1, 0,0 , 0],[0, 1 ,0 , 0],[0, 0, 1 , 0],[0, 0, 0 , 1]])
		robot.SetTransform(handPose)
		# set the finger joint 
		robot.SetDOFValues([float(FeasibleGrasp[Graspidx][0])],[0])
		robot.SetDOFValues([float(FeasibleGrasp[Graspidx][1])],[1])
		robot.SetDOFValues([float(FeasibleGrasp[Graspidx][2])],[2])
		robot.SetDOFValues([float(FeasibleGrasp[Graspidx][3])],[3])

		# set the grasping points
		Point1 = array([float(FeasibleGrasp[Graspidx][4]), float(FeasibleGrasp[Graspidx][5]), float(FeasibleGrasp[Graspidx][6])])
		Point2 = array([float(FeasibleGrasp[Graspidx][7]), float(FeasibleGrasp[Graspidx][8]), float(FeasibleGrasp[Graspidx][9])])
		Point3 = array([float(FeasibleGrasp[Graspidx][10]), float(FeasibleGrasp[Graspidx][11]), float(FeasibleGrasp[Graspidx][12])])
		VFObj = VirtualFrame(Point1,Point2,Point3)
		
		T0 = robot.GetLinks()[3].GetGlobalMassFrame()
		T1 = robot.GetLinks()[6].GetGlobalMassFrame()
		T2 = robot.GetLinks()[8].GetGlobalMassFrame()
		T0[0:3,3] = T0[0:3,3] + 0.01*T0[0:3,1]+0.025*T0[0:3,0]
		T1[0:3,3] = T1[0:3,3] + 0.01*T1[0:3,1]+0.025*T1[0:3,0]
		T2[0:3,3] = T2[0:3,3] + 0.01*T2[0:3,1]+0.025*T2[0:3,0]
		Finger1 = T0[0:3,3]
		Finger2 = T1[0:3,3]
		Finger3 = T2[0:3,3]
		VFHand = VirtualFrame(Finger1,Finger2,Finger3)

		handPose = dot(VFObj,inv(VFHand))
		robot.SetTransform(handPose)

		try:
			handles = []
			htac1 = env.plot3(points=Point1, pointsize=0.012,colors=[0,0,1],drawstyle=1)
			htac2 = env.plot3(points=Point2, pointsize=0.012,colors=[0,0,1],drawstyle=1)
			htac3 = env.plot3(points=Point3, pointsize=0.012,colors=[0,0,1],drawstyle=1)
			handles.append(htac1)
			handles.append(htac2)
			handles.append(htac3)

			framescale =0.06
			h1 = env.drawlinestrip(points=array((VFObj[0:3,3],VFObj[0:3,3]+VFObj[0:3,0]*framescale)),linewidth=8.0, colors=array(((1,0,0,0.5))))
			h2 = env.drawlinestrip(points=array((VFObj[0:3,3],VFObj[0:3,3]+VFObj[0:3,1]*framescale)),linewidth=8.0, colors=array(((0,1,0,0.5))))
			h3 = env.drawlinestrip(points=array((VFObj[0:3,3],VFObj[0:3,3]+VFObj[0:3,2]*framescale)),linewidth=8.0, colors=array(((0,0,1,0.5))))
			handles.append(h1)
			handles.append(h2)
			handles.append(h3)

			env.UpdatePublishedBodies()
			spinner = PlotSpinner(handles[-1])
			spinner.start()

			time.sleep(0.5)
		finally:
			if spinner is not None:
				spinner.ok = False


time.sleep(100) # sleep 2 seconds