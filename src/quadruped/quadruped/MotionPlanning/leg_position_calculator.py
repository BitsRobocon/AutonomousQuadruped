'''
This scriptwill contain a ros node which will subscribe to both the direction of
movement topic and overall map topic, and plan the position and direction of force
to be applied by each leg and publish it.

Subscribers
    /path/direction < quadruped_interfaces.msg.MovementDirection >
    /map/octree/heightmap < quadruped_interfaces.msg.OctreeHeightMap >

Publishers
    /leg/position < quadruped_interfaces.msg.LegPosition > # can be changed to one publisher for each leg

Author


'''
