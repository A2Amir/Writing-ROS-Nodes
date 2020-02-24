# Writing ROS Nodes

In [the previous lesson]() I've created a Catkin workspace and added the simple arm package to it, 
In this lesson, I will be writing notes in Python that publish and subscribe to topics and write a Ross service that can be called from other nodes or from the command line.

The first node that I will be writing is called simple mover. The simple mover node does nothing more than publish joint angle commands to simple arm.

The next node called arm mover. The arm mover node provides a service called safe move, which allows the arm to be moved to any position within its workspace, which has been deemed to be safe. The safe zone is bounded by minimum and maximum joint angles and it's configurable via the Ross parameter server.

The last node I will write in this lesson, is the look away node.  This node subscribes to a topic where camera data is being published. When the camera detects an image with uniform color meaning that it's looking at the sky, the node will call the safe move service to move the arm to a new position.
