# q_learning_project
## Team: Elizabeth Singer, Adam Weider, and Katie Hughes

# Writeup
## Objectives Description
Our goal was to use a Q learning algorithm and reinforcement learning to teach the turtlebot to find the correct placement of dumbbells in front of corresponding numbered blocks. The next set of objectives was to use perception to identify the colored dumbbells and numbered blocks and robot manipulation to execute the strategy learned in the Q learning stage. We also used navigation to properly align the robot, pick up the blocks, and place them in the right places. Essentially, the robot used Q learning to learn how to find the right combination of dumbbells and blocks, perception to see which dumbbells and blocks it was looking at, navigation to get to the right blocks, and robot arm manipulation to pick up and put down the dumbbells in the place perceived by perception according to the strategy derived using q learning. 

## High-level description
(1 paragraph): At a high-level, describe how you used reinforcement learning to solve the task of determining which dumbbells belong in front of each numbered block.

## Q-learning algorithm description
### Selecting and executing actions for the robot (or phantom robot) to take
We select actions for the phantom robot to take by using the action matrix, which tells which actions are legal given a specific state of the robot. Once we have a list of legal actions, we randomly choose one and publish the desired action. We include some rospy.sleep() calls in order to ensure that the reward is received before proceeding.
### Updating the Q-matrix
We update the q-matrix via the algorithm described on the project page. Once a particular action has been performed and a reward has been received, we look at the potential rewards from the new position (by looking up another row in the q-matrix) and take the maximum value. We chose to use the parameters alpha = 1 and gamma = 0.5. We increment the value of the current index in the q-matrix by the quantity alpha * (reward + gamma * maximum_value - current_index). 
### Determining when to stop iterating through the Q-learning algorithm
We stop iterating through the q-learning algorithm after a certain threshold of iterations without an update. Through some trial and error we determined that the best threshold value is around 50. 
### Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot 
To determine the most optimal path after the q-matrix has converged, we locate the current state of the robot, and look up the possible actions that can be taken from that state (which is a row in the q-matrix). We find which index has the maximum value, which corresponds to which action has the maximum future reward. If there is more than one maximum we randomly choose one of them. Then, we perform that action. This process repeats until the robot ends up in its final state and receives the reward. 

## Robot perception description
Describe how you accomplished each of the following components of the perception elements of this project in 1-3 sentences, any online sources of information/code that helped you to recognize the objects, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):
### Identifying the locations and identities of each of the colored dumbbells
### Identifying the locations and identities of each of the numbered blocks

## Robot manipulation and movement
Describe how you accomplished each of the following components of the robot manipulation and movement elements of this project in 1-3 sentences, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):

### Gif - manipulator
![Example showing manipulator orienting toward dumbbell, picking it up, moving it, and putting it down.](Manipulate.gif)

### Moving to the right spot in order to pick up a dumbbell
The perception code uses color perception to identify the correct color dumbbell to approach using a combination of the camera and the laser scanner and proportional control. Once within half a meter of the appropriate colored dumbbell, the lift method in movearm.py uses the laser scanner to align the robot to point directly at the center of the dumbbell using proportional angular/rotational control and then uses proportional distance control to approach the dumbbell.

### Picking up the dumbbell
Once properly aligned and at a sufficient distance from the dumbbell, the manipulator arm is placed into a position where it can grab the center of the dumbbell. Using proportional control and forward motion the robot approaches until the gripper is properly around the handle of the dumbbell. Angular and distance control are used for this precise movement. The gripper is closed and the manipulator raises the dumbbell to a position where it can be carried without falling. 

### Moving to the desired destination (numbered block) with the dumbbell
### Putting the dumbbell back down at the desired destination
Once oriented in front of the correct block a calculated sequence of motions are used to lower the dumbbell to the ground, open the manipulator arm, and then move back slowly to leave the dumbbell in an upright position in front of the appropriate block. 

## Challenges
(1 paragraph): Describe the challenges you faced and how you overcame them.

## Future work
(1 paragraph): If you had more time, how would you improve your implementation?

## Takeaways
(at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.

## gif
In your writeup, include a gif of your robot successfully executing the task once your Q matrix has converged.

## rosbag
Record a run of your q-learning algorithm in a rosbag. Please record the following topics: /cmd_vel, /gazebo/set_model_state, /q_learning/q_matrix, /q_learning/reward, /q_learning/robot_action, /scan, and any other topics you generate and use in your particle filter project. Please do not record all of the topics, since the camera topics make the rosbags very large. For ease of use, here's how to record a rosbag:
$ rosbag record -O filename.bag topic-names
Please refer to the ROS Resources page for further details on how to record a rosbag.

# Implementation plan

## Q-learning algorithm

### Executing the Q-learning algorithm

- Implementation: Set up the data structures for the Q-matrix and implement the
following tasks: initialize Q-matrix, select random action, perform random action,
receive reward for action, update Q-matrix, determine when Q-matrix has converged.
These will be implemented in a ROS node making use of the phantom robot node using
the sub/pub messages for determining the outcomes from the learning phase.

- Testing: A self-contained python node will be created to test each of the phases
of the Q-learning algorithm. Through printing states to the screen and running test
cases where the learning updates can be calculated by hand, the functions can be
tested and shown to be correct.

### Determining when the Q-matrix has converged

- Implementation: The convergence of the Q-matrix will be tested by checking the
Frobenius norm of the Q-matrix (sum of the squares of the matrix), or the Frobenius
norm of the change in the Q-matrix from one step to another (sum of the squares
of the delta from one update). Once the Frobenius norm ceases to change, or changes
by less than a threshold, the Q-matrix has converged.

- Testing: A simple loop can be created that changes a matrix by an amount that
decreases over time geometrically. Once the changes are small enough, the convergence
threshold should be met and the convergence can be tested. We can also print a
small subset of Q-matrix values to the screen to observe convergence.

### Choosing actions to maximize reward

- Implementation: Given the state of the world, the Q-matrix will show which
actions are feasible, and which have the highest expected future reward. We can
simply select the one action of highest future reward, or randomly select actions
proportional to their expected future reward. If there are actions of equal future
reward, we can randomly select from among them.

- Testing: Given a Q-matrix, we can test that we select an appropriate action.
We can first do this with the phantom robot movement, and then once the movement
node is created, we can test with the turtlebot.

## Robot perception

### Identifying and locating the colored dumbbells

- Implementation: We can use the color image sensor to detect the direction in
which a given dumbbell lies relative to the robot (the color values should stand
out against those of the world background and the blocks). Then, we can inspect
the values of those contrasting color measurements to determine the identity of
the dumbbell in view.

- Testing: We can use Gazebo to place the robot near specific dumbbells and verify
that the correct direction and color are determined from the image sensor
measurements.

### Identifying and locating the numbered blocks

- Implementation: We can use a similar technique to that described above for
the dumbbells, though instead of image sensor measurements, we will instead use
an opencv2 classifier (as demonstrated in the class 11 lecture notes) for detecting
the numbers on the blocks.

- Testing: As was true of the implementation, testing should also be similar to
that described for identifying and locating the dumbbells.

## Robot manipulation and movement

### Handling the dumbbells with the OpenMANIPULATOR arm

- Implementation: We will use the provided routines to command the manipulator
arm to the required angles to grip the dumbbells, and then to close the claw,
and then to lift the dumbbells. These required positions will be fixed since the
dumbbells are the same size and do not change. If the orientation of the robot
needs to be set properly in order to grab the dumbbells, we will implement a
proportional feedback routine to position the dumbbell in the center of the camera
so that it can be grabbed. The parameters of the positioning algorithm will need
to be learned from trial and error in Gazebo.

- Testing: Using Gazebo, we will place the dumbbells in front of the robot and
determine the necessary angles to command in order to address, grab, and lift the
dumbbells. We can similarly set up the dumbbells in front of the robot to test the
tracking algorithm to position the robot in place so that it can grab the dumbbells.


### Navigating to where the robot will handle the dumbbells

- Implementation: We can use the laser scanner and color image sensor to determine
the location of the dumbbells in the world, and either determine the color of the
dumbbell at a distance (we can test this) or move to that location and use the
color sensor to determine which dumbbell is at that location. Once the dumbbell
locations are known, we can use the Q-matrix to determine where to place them
using the manipulator functions from above.

- Testing: Using our movement framework, instruct the robot to navigate to a
specific dumbbell or block, and visually inspect the results. It should be clear
if something is amiss (e.g. navigating to an incorrect target, missing the target
entirely, colliding with the target, etc).

# Timeline of milestones

### Saturday, February 20th
- Q-learning algorithm

### Wednesday, February 24th
- Robot perception

### Saturday, February 27th
- Robot manipulation and movement

### Sunday, February 28th
- Writeup, recording, rosbag, and partner survey
