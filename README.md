# q_learning_project
## Team: Elizabeth Singer, Adam Weider, and Katie Hughes

# Writeup
## Objectives Description
- Our goal was to use a Q learning algorithm and reinforcement learning to teach the turtlebot to find the correct placement of dumbbells in front of corresponding numbered blocks. The next set of objectives was to use perception to identify the colored dumbbells and numbered blocks and robot manipulation to execute the strategy learned in the Q learning stage. We also used navigation to properly align the robot, pick up the blocks, and place them in the right places. Essentially, the robot used Q learning to learn how to find the right combination of dumbbells and blocks, perception to see which dumbbells and blocks it was looking at, navigation to get to the right blocks, and robot arm manipulation to pick up and put down the dumbbells in the place perceived by perception according to the strategy derived using q learning. 

## High-level description
- We use a q-learning algorithm to determine the correct orientation of the dumbbells. The algorithm progressively fills in a matrix, where the rows represent the possible states (which dumbbell is at which block) and the columns represent possible actions the robot can take (moving a particular dumbbell to a particular block). We created an action matrix to determine which actions are allowed at each state. This limited the possible state transitions so that only moves of a single dumbbell from the origin to an empty block are allowed. In the bulk of our algorithm, we begin from the initial state where every dumbbell is at the origin. Using the action matrix, we chose a random allowed action from this state, transition to that state, and receive a reward from performing that action. A function of the reward, plus any rewards that are obtainable from the new state, is stored in the q-matrix. As the algorithm progresses, and the robot tries out many different random combinations of allowed actions, the actions that lead to future rewards are incremented in the q-matrix. This allows us to “trace out” the optimal path by examining the maximum values of the q-matrix at the robot’s current state. Through trial and error, the robot learns which actions will be rewarded the most in the future. 

## Q-learning algorithm description
### Selecting and executing actions for the robot (or phantom robot) to take
- The current script used for the q-learning algorithm is q-algorithm.py. 
- Selecting and executing actions for the robot (or phantom robot) to take
  - We select actions for the phantom robot to take by using the action matrix, which tells which actions are legal given a specific state of the robot. Once we have a list of legal actions, we randomly choose one and publish the desired action. We include some rospy.sleep() calls in order to ensure that the reward is received before proceeding.
  - The action matrix is created in initialize_action_matrix. The selection of a random action given the current state is done at the beginning of the fill_qmatrix function. This is also where we publish the movements to the phantom robot. I use a variety of helper functions to translate between the state and action numbers and the orientations of the dumbbells. These are the functions find_state, locations_from_state, find_action, and inverse_action. I also have helper functions to check if a particular dumbbell configuration is allowed (valid_state), to apply an action to a state and get the resulting state (apply_action), and check if the state is an “end state” where every dumbbell is at a block, and there are no more legal moves (end_state). 
### Updating the Q-matrix
- We update the q-matrix via the algorithm described on the project page. Once a particular action has been performed and a reward has been received, we look at the potential rewards from the new robot state (by looking up another appropriate row in the q-matrix) and take the maximum value. We chose to use the parameters alpha = 1 and gamma = 0.5. We increment the value of the current index in the q-matrix by the quantity alpha * (reward + gamma * maximum_value - current_index).
- Updating the q-matrix also happens in the fill_qmatrix function. We check if the updated value of the q-matrix is the same as it was before, and if it is not, we change it and publish the new q-matrix.  
### Determining when to stop iterating through the Q-learning algorithm
- We stop iterating through the q-learning algorithm after a certain threshold of iterations without an update. Through some trial and error we determined that the best threshold value is around 75. 
- The parameters that determine when to stop are threshold and self.count, both used in fill_qmatrix. The count variable increments every time an update is not made and is set to 0 otherwise. The algorithm stops when self.count is equal to the threshold. 
### Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot 
- To determine the most optimal path after the q-matrix has converged, we locate the current state of the robot, and look up the possible actions that can be taken from that state (which is a row in the q-matrix). We find which index has the maximum value, which corresponds to which action has the maximum future reward. If there is more than one maximum we randomly choose one of them. Then, we perform that action. This process repeats until the robot ends up in its final state and receives the reward.

- I print out an optimal set of robot actions at the end of the fill_qmatrix function. These are calculated from the q-matrix, so by subscribing to the q_matrix topic in later scripts, we can recalculate these in the movement and perception functions. 

## Robot perception description
Describe how you accomplished each of the following components of the perception elements of this project in 1-3 sentences, any online sources of information/code that helped you to recognize the objects, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):

### Identifying the locations and identities of each of the colored dumbbells

- describe how you accomplished each of the following components of the perception elements of this project in 1-3 sentences, any online sources of information/code that helped you to recognize the objects
- describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code)
- add pictures, Youtube videos, and/or embedded animated gifs

### Identifying the locations and identities of each of the numbered blocks

- describe how you accomplished each of the following components of the perception elements of this project in 1-3 sentences, any online sources of information/code that helped you to recognize the objects,
- describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code)
- add pictures, Youtube videos, and/or embedded animated gifs

## Robot manipulation and movement
Describe how you accomplished each of the following components of the robot manipulation and movement elements of this project in 1-3 sentences, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):

### Gif - manipulator
![Example showing manipulator orienting toward dumbbell, picking it up, moving it, and putting it down.](Manipulate.gif)

### Moving to the right spot in order to pick up a dumbbell
- The perception code uses color perception to identify the correct color dumbbell to approach using a combination of the camera and the laser scanner and proportional control. Once within half a meter of the appropriate colored dumbbell, the lift method in movearm.py uses the laser scanner to align the robot to point directly at the center of the dumbbell using proportional angular/rotational control and then uses proportional distance control to approach the dumbbell.
- Functions involved: face_dumbbell is a method within the robot node that uses proportional angular control to orient the robot so that the handle of the dumbbell is exactly centered in front of the robot. Laser scan messages and twist messages are used to update the range data in this method and to update and control the positioning of the robot. 
- add pictures, Youtube videos, and/or embedded animated gifs

### Picking up the dumbbell
- Once properly aligned and at a sufficient distance from the dumbbell, the manipulator arm is placed into a position where it can grab the center of the dumbbell. Using proportional control and forward motion the robot approaches until the gripper is properly around the handle of the dumbbell. Angular and distance control are used for this precise movement. The gripper is closed and the manipulator raises the dumbbell to a position where it can be carried without falling. 
- Functions involved: pickup_db runs lift_dumbbell, open_gripper, face_dumbbell, reach_dumbbell, approach_dumbbell, close_gripper, and lift_dumbbell. Proccess_scan is run every time a scan is received from the lidar and approach dumbbell and face_dumbbell are called from process scan if the appropriate flags are set requesting these actions. These flags are set within pickup_db so they in effect, call face_dumbbell and approach_dumbbell via the process scan function. 
  - open_gripper sets a gripper joint goal and uses the move_group to achieve that goal. 
  - close_gripper sets a gripper joint goal and uses the move_group to achieve that goal.
  - reach_dumbbell sets an arm joint goal to position the arm joint to reach out and be ready to grab the dumbbell and uses the move_group to achieve that goal. 
  - lift_dumbbell sets an arm joint goal to raise the dumbbell into the air and uses a move_group to affect that motion. 
  - face_dumbbell is called by proccess_scan when laser scan data is received if the self.turning flag is true.
  - approach _dumbbell is called in proccess_scan if the self.moving flag is set to true. Approach_dumbbell continues to move forward until the gripper is at the right distance to be able to grab the dumbbell and it publishes twist messages onto cmd_vel to control movement. 
  - Proccess_scan 
 - add pictures, Youtube videos, and/or embedded animated gifs

### Moving to the desired destination (numbered block) with the dumbbell
- Describe how you accomplished each of the following components of the robot manipulation and movement elements of this project in 1-3 sentences
- describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code)
- add pictures, Youtube videos, and/or embedded animated gifs
### Putting the dumbbell back down at the desired destination
- Once oriented in front of the correct block a calculated sequence of motions are used to lower the dumbbell to the ground, open the manipulator arm, and then move back slowly to leave the dumbbell in an upright position in front of the appropriate block. 
- putdown_db is a method that calls set_dumbbell and open_gripper. Set_dumbbell ses an arm joint goal to properly lower the dumbbell to the ground using move_group to affect that goal. It then calls open_gripper to release the dumbbell and then it publishes a twist message to slowly back away from the dumbbell, leaving it in place. 
- add pictures, Youtube videos, and/or embedded animated gifs

## Challenges
- One thing I (Katie) struggled with in the q-learning algorithm was receiving the appropriate reward for the action performed. With the first attempt, the entire algorithm ran to completion before a single reward was read in. WIth a second attempt, a rospy.sleep() statement was added between the movement publication and the update of the q-matrix. This read in rewards, but this still sometimes read in the wrong reward for the action. The current working implementation sleeps before the movement publication, after the movement publication, within the callback function to receive the reward, and after receiving a nonzero reward. This makes the algorithm slower, but appears to eliminate the problem of rewards not matching up. 
- Elizabeth
- Adam

## Future work
- The q-learning algorithm currently takes on the order of 5-10 minutes to converge (~300 total iterations). This is because of the sleep statements, which were added to ensure the correct reward was read in. Reducing the sleep times from 1 to 0.5 seconds created additional problems with getting the correct reward. If we had more time, maybe we could figure out a better way to read in the correct reward so that the algorithm does not take so long. 

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
