import pybullet as p
import time
import pybullet_data

phisicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF

planeId = p.loadURDF("plane.urdf")
quadID = p.loadURDF("./quadruped.urdf")
GRAVITY = -9.8

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

while (p.isConnected()):
  keys = p.getKeyboardEvents()
  time.sleep(0.1)