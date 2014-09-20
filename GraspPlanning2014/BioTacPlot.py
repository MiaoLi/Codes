#!/usr/bin/python
"""
This file is used to visulize the tactile sensing on syntouch (BioTac)

"""

from openravepy import *
from openravepy import misc
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import time

import matplotlib.cm as cmx

def PlotTransformation(T, env, framescale):
	#plot the coordinate frame given a transformation
	h1=env.drawlinestrip(points=array((T[0:3,3],T[0:3,3]+T[0:3,0]*framescale)),linewidth=5.0, colors=array(((1,0,0,0.5))))
	h2=env.drawlinestrip(points=array((T[0:3,3],T[0:3,3]+T[0:3,1]*framescale)),linewidth=5.0, colors=array(((0,1,0,0.5))))
	h3=env.drawlinestrip(points=array((T[0:3,3],T[0:3,3]+T[0:3,2]*framescale)),linewidth=5.0, colors=array(((0,0,1,0.5))))
	return [h1,h2,h3]



ElectrodePos=array([[-7.5,20.42,0],
					[-4.24,16.72,6.18],
					[-4.24,13.22,6.18],
					[-7.5,11.42,0],
					[-4.24,8.92,6.18],
					[-7.5,6.02,0],
					[0,25.62,1.5],
					[-4,23.62,4.5],
					[4,23.62,4.5],
					[0,21.62,7.5],
					[7.5,20.42,0],
					[4.24,16.72,6.18],
					[4.24,13.22,6.18],
					[7.5,11.42,0],
					[4.24,8.92,6.18],
					[7.5,6.02,0],
					[0,16.62,7.5],
					[0,9.62,7.5],
					[0,5.82,7.5]
					])*0.001

objGrasp = 'biotac'


env = Environment() # create openrave environment
env.SetDebugLevel(DebugLevel.Fatal)
env.SetViewer('qtcoin') # attach viewer (optional)

obj = env.ReadKinBodyXMLFile('./Models/' + objGrasp + '.kinbody.xml')
env.Add(obj)
time.sleep(0.1)


handles = []
oriPose = array([[1, 0,0 , 0],[0, 1 ,0 , 0],[0, 0, 1 , 0],[0, 0, 0 , 1]])
sensorOri = array([[0,-1,0,0.027],[0,0,-1,0.024],[1,0,0,0.0116],[0,0,0,1]])

ElectrodePosNew = zeros([19,3]) # position in the world frame
for i in range(19):
	ElectrodePosNew[i,:] = sensorOri[0:3,0:3].dot(ElectrodePos[i,:])+sensorOri[0:3,3]

colorlevel = 128
jetcmap = cmx.get_cmap("hot", colorlevel)    #http://wiki.scipy.org/Cookbook/Matplotlib/Show_colormaps
colormap = jetcmap(numpy.arange(colorlevel))  

tactileMax = 200 # set the this to the ral record value
lower = zeros(19)
upper = ones(19)*tactileMax

nbSample =1000

with env:
	#htmp1 = PlotTransformation(oriPose,env,0.01)
	#handles.append(htmp1)
	#htmp2 = PlotTransformation(sensorOri,env,0.01)
	#handles.append(htmp2)
	for i in range(nbSample):
		
		tactielValue = numpy.random.rand(len(lower))*(upper-lower)+lower # simulate value
		
		ElectrodeColor = zeros([19,4])

		for i in range(19):
			ind = int(tactielValue[i]/tactileMax*colorlevel)
			ElectrodeColor[i,:] = colormap[ind,:] 
		htac = env.plot3(points=ElectrodePosNew, pointsize=0.002,colors=ElectrodeColor,drawstyle=1)
		env.UpdatePublishedBodies()
		time.sleep(0.1)

raw_input('press Enter to exit')