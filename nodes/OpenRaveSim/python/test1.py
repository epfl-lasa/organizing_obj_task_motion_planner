from openravepy import *
from types import *
from datetime import datetime
import random
import math, time
import numpy as np
import sys
if not __openravepy_build_doc__:
  from openravepy import *
  from numpy import * 
  import time, threading
  from collections import Counter
  from numpy.linalg import inv
  from scipy.optimize import *
  from math import radians
  import rospy
  from copy import copy, deepcopy
  from GraspFun import *
  import pdb
#   sys.path.insert(0, 'pioneer3at')


#############################################################################################################
# set up environment
#############################################################################################################
env = Environment() # create the environment
env.SetDebugLevel(DebugLevel.Fatal)
env.SetViewer('qtcoin') # start the viewer
# pdb.set_trace() #breakpoint
# env.Load('/pioneer3at/model.sdf') # load a scene

