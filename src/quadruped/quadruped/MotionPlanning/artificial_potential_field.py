'''
This scriptwill contain a ros node which will create the artificial potential field
map using data gained from the octree generated. This will then find an optimal
direction of motion and publish it to the leg_position_calculator node

Subscribers
    /map/octree/changes  < quadruped_interfaces.msg.OctreeChanges >

Publishers
    /path/direction < quadruped_interfaces.msg.MovementDirection >

Author
    bhavika-g <bhavika.gopalani@gmail.com>

'''
