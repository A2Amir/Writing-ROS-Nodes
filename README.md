# Writing ROS Nodes

In [the previous lesson](https://github.com/A2Amir/Catkin-Workspace-for-ROS) I've created a Catkin workspace and added the simple arm package to it, 
In this lesson, I will be writing nodes in Python that publish and subscribe to topics and write a Ross service that can be called from other nodes or from the command line.

* The first node that I will be writing is called simple mover. The simple mover node does nothing more than publish joint angle commands to simple arm.

* The next node called arm mover. The arm mover node provides a service called safe move, which allows the arm to be moved to any position within its workspace, which has been deemed to be safe. The safe zone is bounded by minimum and maximum joint angles and it's configurable via the Ross parameter server.

* The last node I will write in this lesson, is the look away node.  This node subscribes to a topic where camera data is being published. When the camera detects an image with uniform color meaning that it's looking at the sky, the node will call the safe move service to move the arm to a new position.

# 1. ROS Publishers

Before seeing the code for **simple_mover**, it may be helpful to see how ROS Publishers work in Python. Publishers allow a node to send messages to a topic, so that data from the node can be used in other parts of the ROS system. In Python, ROS publishers typically have the following definition format, although other parameters and arguments are possible:

    pub1 = rospy.Publisher("/topic_name", message_type, queue_size=size)

* **Synchronous publishing** means that a publisher will attempt to publish to a topic but may be blocked if that topic is being published to by a different publisher. In this situation, the second publisher is blocked until the first publisher has serialized all messages to a buffer and the buffer has written the messages to each of the topic's subscribers. This is the default behavior of a rospy Publisher **if the queue_size parameter is not used or set to None**.

* **Asynchronous publishing** means that a publisher can store messages in a queue until the messages can be sent. If the number of messages published exceeds the size of the queue, the oldest messages are dropped. **The queue size can be set using the queue_size parameter**.


Once the publisher has been created as above, a message with the specified data type can be published as follows:
	  
    pub1.publish(message)
 
Note:Choosing a good **queue_size** is somewhat subjective, setting queue_size=0 actually creates an infinite queue. This could lead to memory leakage if the messages are published faster than they are picked up. providing a little room for messages to queue without being too large, is a good choice.

#### Simple Mover: The Code

Below is the Explanation of the code for **[the simple_mover node](https://github.com/A2Amir/Writing-ROS-Nodes/blob/master/Code/simple_mover.py)**, followed by a step-by-step explanation of what is happening. 

First, open a new terminal, next: 

	1. cd ~/catkin_ws/src/simple_arm/
	2. mkdir scripts
	3. cd scripts
	4. touch simple_mover
	5. nano simple_mover

6. I have opened the simple_mover script with the nano editor, now copy 
	and paste the code below from [this file](https://github.com/A2Amir/Writing-ROS-Nodes/blob/master/Code/simple_mover.py) into the script 	
	and use ctrl-x followed by y then enter to save the script.

	7. chmod u+x simple_mover
 
#### The code: Explained
