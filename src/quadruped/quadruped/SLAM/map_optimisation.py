'''
This scriptwill contain a ros node which will subscribe to the sensor fusion node
and optimise and place the local map created by that node into the global octomap

Subscribers
    /map/grid < quadruped_interfaces.msg.GridMap >

Publishers
    /map/octree/changes < quadruped_interfaces.msg.OctreeChanges >
    /map/octree/heightmap < quadruped_interfaces.msg.OctreeHeightMap >

Author
    ashutoshshrm529 <ashutoshshrm529@gmail.com>

'''
