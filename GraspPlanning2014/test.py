#!/usr/bin/python

"""
We want to compare several different methods for grasp planing:
--- two-step planning
--- one-shot planning
"""

from openravepy import *
import time

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('data/lab1.env.xml') # load a simple scene
robot = env.GetRobots()[0] # get the first robot

time.sleep(100)