# q_learning_project

**Team: Elizabeth Singer, Adam Weider, and Katie Hughes**

## Setup

See [SETUP](docs/SETUP.md) for instructions on setting up and running the project.

## Objectives Description

Our goal was to use a Q learning algorithm and reinforcement learning to teach
the turtlebot to find the correct placement of dumbbells in front of
corresponding numbered blocks. The next set of objectives was to use perception
to identify the colored dumbbells and numbered blocks and robot manipulation to
execute the strategy learned in the Q learning stage. We also used navigation
and proportional control to properly align the robot, pick up the blocks, and
place them in the right places. Essentially, the robot used Q learning to learn
how to find the right combination of dumbbells and blocks, perception to see
which dumbbells and blocks it was looking at, navigation to get to the right
blocks, and robot arm manipulation to pick up and put down the dumbbells in the
place perceived by perception according to the strategy derived using q
learning.

## High-level description

We use a q-learning algorithm to determine the correct orientation of the
dumbbells. The algorithm progressively fills in a matrix, where the rows
represent the possible states (which dumbbell is at which block) and the columns
represent possible actions the robot can take (moving a particular dumbbell to a
particular block). We created an action matrix to determine which actions are
allowed at each state. This limited the possible state transitions so that only
moves of a single dumbbell from the origin to an empty block are allowed. In the
bulk of our algorithm, we begin from the initial state where every dumbbell is
at the origin. Using the action matrix, we chose a random allowed action from
this state, transition to that state, and receive a reward from performing that
action. A function of the reward, plus any rewards that are obtainable from the
new state, is stored in the q-matrix. As the algorithm progresses, and the robot
tries out many different random combinations of allowed actions, the actions
that lead to future rewards are incremented in the q-matrix. This allows us to
“trace out” the optimal path by examining the maximum values of the q-matrix at
the robot’s current state. Eventually, the robot learns which actions will be
rewarded the most in the future.

### The Q-matrix converges, and then TurtleBot3 performs the optimal three actions

![ShortQLearn](https://user-images.githubusercontent.com/68019178/109866278-b578de00-7c2a-11eb-8e42-add33781c6b7.gif)

## Q-learning algorithm

### Selecting and executing actions for the robot (or phantom robot) to take

- We select actions for the phantom robot to take by using the action matrix,
  which tells which actions are legal given a specific state of the robot. Once
  we have a list of legal actions, we randomly choose one and publish the
  desired action. We include some rospy.sleep() calls in order to ensure that
  the reward is received before proceeding.

- The action matrix is created in initialize_action_matrix. The selection of a
  random action given the current state is done at the beginning of the
  fill_qmatrix function. This is also where we publish the movements to the
  phantom robot. We use a variety of helper functions to translate between the
  state and action numbers and the orientations of the dumbbells. These are the
  functions find_state, locations_from_state, find_action, and inverse_action.
  We also have helper functions to check if a particular dumbbell configuration
  is allowed (valid_state), to apply an action to a state and get the resulting
  state (apply_action), and check if the state is an “end state” where every
  dumbbell is at a block, and there are no more legal moves (end_state).

### Updating the Q-matrix

- We update the q-matrix via the algorithm described on the project page. Once a
  particular action has been performed and a reward has been received, we look
  at the potential rewards from the new robot state (by looking up another
  appropriate row in the q-matrix) and take the maximum value. We chose to use
  the parameters alpha = 1 and gamma = 0.5. We increment the value of the
  current index in the q-matrix by the quantity alpha * (reward + gamma *
  maximum_value - current_index).

- Updating the q-matrix also happens in the fill_qmatrix function. We check if
  the updated value of the q-matrix is the same as it was before, and if it is
  not, we change it and publish the new q-matrix.

### Determining when to stop iterating through the Q-learning algorithm

- We stop iterating through the q-learning algorithm after a certain threshold
  of iterations without an update. Through some trial and error we determined
  that the best threshold value is around 75.

- The parameters that determine when to stop are threshold and self.count, both
  used in fill_qmatrix. The count variable increments every time an update is
  not made and is set to 0 otherwise. The algorithm stops when self.count is
  equal to the threshold.

### Maximizing reward after the Q-Matrix has converged

- To determine the most optimal path after the q-matrix has converged, we locate
  the current state of the robot, and look up the possible actions that can be
  taken from that state (which is a row in the q-matrix). We find which index
  has the maximum value, which corresponds to which action has the maximum
  future reward. If there is more than one maximum we randomly choose one of
  them. Then, we perform that action. This process repeats until the robot ends
  up in its final state and receives the reward.

- We evaluate an optimal set of actions after the q-matrix has converged in the
  fill_qmatrix function. I calculate the set of 3 actions and publish them to a
  new topic called q_learning/optimal actions. The published message holds a
  list of RobotMoveDBToBlock messages, in the order they should be executed. The
  following scripts subscribe to this topic to receive the actions.

## Robot perception

### Identifying the locations and identities of each of the colored dumbbells

- The colored dumbbells are identified using CV2 operations for masking colors in
  images provided by the robot camera. The center pixel coordinates of the
  current dumbbell color in the image are computed. If no coordinates are found,
  the dumbbell is assumed to be out of frame, and the robot rotates to have the
  dumbbell enter into frame. Once the center of the color is determined, the
  x-offset from the center of the image is used to compute an error factor for
  the alignment of the TurtleBot and the dumbbell. Proportional control is
  applied to this error for finding the appropriate angular velocity at which
  to face the dumbbell of interest.

- `scripts/perception/color.py`
  - `locate_color(range, img)` - Locate the center of the given HSV color range
    `range` in the image `img`. CV2 operations are used to mask the image,
    calculate the resulting moments, and compute the pixel center of the color
    range. If the center cannot be computed, `None` is returned.

- `scripts/robot_action.py`
  - Lines 559-578 of `update(msg, model)` - Dumbbell alignment error is computed
    using `locate_color` applied to the color range for the target dumbbell. If
    the center of the color range cannot be found (i.e. if no objects with such
    colors are present in the image), the bot begins turning slowly to allow the
    dumbbell to enter into view. If the alignment error is under a set threshold
    close to zero, the bot switches states to begin approaching the dumbbell.
    Otherwise, proportional control is used to compute an appropriate angular
    velocity for rotating the bot to face the dumbbell.

### Identifying the locations and identities of each of the numbered blocks

- When TurtleBot has a dumbbell grasped, it rotates around to the general
  location of the blocks. Two orientations are specified at which to perform
  OCR, one between the left and center block, and one between the center and
  right block. Two orientations are used as, from the distance of the dumbbells,
  the blocks cannot all fit inside the field of view of the camera; having two
  perspectives guarantees all blocks are subject to an OCR pass. Like when
  facing toward the dumbbells, the center pixel coordinates of the bounding box
  on the recognized character are found, and the x-offset from the center of the
  image is used to compute angular error. Proportional control is used to
  compute the appropriate angular velocity for facing the target block.

- `scripts/perception/ocr.py`
  - `locate_number(pipeline, num, img)` - Use the given Keras OCR `pipeline` to
    detect the most front-facing instance of the number `num` in image `img`.
    The result with the bounding box of maximum width is selected as the
    instance of the number oriented most towards the front. If no instance of
    the number are found in the image, `None` is returned.

- `scripts/robot_action.py`
  - Lines 447-471 of `update(msg, model)` - Block alignment error is computed
    using `locate_number` applied to the number for the target block. If no
    instance of the number is found, the bot switches states to rotate towards
    the next vantage point from which to perform OCR. If an instance is found,
    the direction to the block is computed from the pixel coordinates of the
    number, and then the bot switches states to begin rotating, aligning its own
    direction with that to the block.

## Robot manipulation and movement

### Moving to the right spot in order to pick up a dumbbell

- The perception code uses color perception to identify the correct color
  dumbbell to approach using a combination of the camera and the laser scanner
  and proportional control. Once within half a meter of the appropriate colored
  dumbbell, the lift method in the Robot node of control_arm.py uses the laser
  scanner to align the robot to point directly at the center of the dumbbell
  using proportional angular/rotational control and then uses proportional
  distance control to approach the dumbbell. Once it’s in the right place it
  publishes the action to be taken by the movement node. We use ROS subscribers
  and publishers to coordinate the handoff between getting to the right location
  in general, and then publishing a lift command for the Robot node (in
  control_arm.py) on /cmd_arm to which this node subscribes. We then process the
  message, which is “up” indicating to move the arm to prepare to lift the
  dumbbell and slowly approach forward while maintaining alignment.

- Functions involved: face_dumbbell is a method within the robot node that uses
  proportional angular control to orient the robot so that the handle of the
  dumbbell is exactly centered in front of the robot. Laser scan messages and
  twist messages are used to update the range data in this method and to update
  and control the positioning of the robot. process_scan() is used when lidar
  scans arrive, and if the turning flag is set, then face_dumbbell is called.
  command_received is used by the control_arm.py (robot) node to monitor the
  /cmd_arm topic where the movement node will publish requests to lift/grab or
  set-down the dumbbell. The /res_arm topic is used to signal when the action is
  complete with a “done” message.  command_received() is a method within the
  robot node that is called when the /cmd_arm topic has a message. If the
  message is “up” then the pickup_db method is called, and if the message is
  “down” then the putdown_db method is called. Any other message signals an
  error.

### Picking up the dumbbell

- Once properly aligned and at a sufficient distance from the dumbbell, the
  manipulator arm is placed into a position where it can grab the center of the
  dumbbell. Using proportional control and forward motion the robot approaches
  until the gripper is properly around the handle of the dumbbell. Angular and
  distance control are used for this precise movement. The gripper is closed and
  the manipulator raises the dumbbell to a position where it can be carried
  without falling, however the dumbbell is not well-suited to carry and travel
  with this gripper.

- Functions involved: pickup_db runs lift_dumbbell (get it out of the way),
  open_gripper (prepare for grab), face_dumbbell (turn to orient facing the
  dumbbell), reach_dumbbell (extend arm to grab position), approach_dumbbell
  (slowly get gripper around handle), close_gripper (grab dumbbell), and
  lift_dumbbell (Raise overhead). Proccess_scan is run every time a scan is
  received from the lidar and approach dumbbell and face_dumbbell are called
  from process scan if the appropriate flags are set requesting these actions.
  These flags are set within pickup_db so they in effect, call face_dumbbell and
  approach_dumbbell via the process scan function and the associated flags.

  - open_gripper sets a gripper joint goal and uses the move_group to achieve
    that goal.

  - close_gripper sets a gripper joint goal and uses the move_group to achieve
    that goal.

  - reach_dumbbell sets an arm joint goal to position the arm joint to reach out
    and be ready to grab the dumbbell and uses the move_group to achieve that
    goal.

  - lift_dumbbell sets an arm joint goal to raise the dumbbell into the air and
    uses a move_group to affect that motion.

  - face_dumbbell is called by proccess_scan when laser scan data is received if
    the self.turning flag is true.

  - approach _dumbbell is called in proccess_scan if the self.moving flag is set
    to true. Approach_dumbbell continues to move forward until the gripper is at
    the right distance to be able to grab the dumbbell and it publishes twist
    messages onto cmd_vel to control movement.

  - For Proccess_scan, everytime a Lidar scan is received, if self.turning (the
    turning flag) is set to true, then face_dumbbell(data) is called to orient
    towards the dumbbell. If self.moving (the moving flag) is set to true, then
    approach underscore dumbbell(data) is called. These two functions set the
    appropriate parameters in self.twist and the twist message is published.

### Moving to the desired destination (numbered block) with the dumbbell

- Unfortunately OCR passes require a significant amount of time, on the order of
  seconds, while image data is received on the order of hundreds of milliseconds.
  Thus, angular error cannot be computed at a sufficient speed to adjust the
  alignment of the bot in real time as it approaches the target block. To
  compensate for this, we have the bot stop when about halfway to the general
  location of the block. The bot repeats the OCR pass and alignment step performed
  previously, thus mitigating angular error which may have developed when moving
  toward the block. Finally, the bot continues to approach the block, adjusting
  only linear velocity via error calculated with closest obstacle distance from
  LiDAR range data.

- `scripts/robot_action.py`
  - Lines 496-526 of `update(msg, model)` - A distance at which to perform
    re-alignment with the block is computed according to the number of remaining
    alignments for the trajectory toward the block (we use a starting value of
    1, so re-alignment only happens once, though the value could be customized
    for different behaviors). If the bot is under the alignment distance to the
    block, then it switches states to run an OCR pass, compute the rotation to
    the number on the block, and re-align with the block. Otherwise, if the
    bot is within a set stopping distance to the block, the bot switches states
    to stop movement and begin placing the dumbbell down. Otherwise, error is
    calculated using the LiDAR scan ranges, and proportional control is applied
    to this error to compute an appropriate linear velocity for approaching the
    block.

### Putting the dumbbell back down at the desired destination

- Once oriented in front of the correct block a calculated sequence of motions
  are used to lower the dumbbell to the ground, open the manipulator arm, and
  then move back slowly to leave the dumbbell in an upright position in front of
  the appropriate block. This action is signaled by the movement node over the
  /cmd_arm topic with the message “down”.

- Functions involved: putdown_db is a method that calls set_dumbbell and
  open_gripper. Set_dumbbell ses an arm joint goal to properly lower the
  dumbbell to the ground using move_group to affect that goal. It then calls
  open_gripper to release the dumbbell and then it publishes a twist message to
  slowly back away from the dumbbell, leaving it in place. Command_received is
  the method that monitors the topic /cmd_arm and makes hte call to putdown_db
  when the message is “down”. Once the action is complete, the /res_arm topic is
  published with the message “done” signaling that the action is completed.

### TurtleBot3 picks up and sets down dumbbells

Note: The GIFs are much slower than real time, for some reason. The robot does
not actually go this slow. (It may depend on the computer you are viewing on)

![MoveLiftSet (1)](https://user-images.githubusercontent.com/68019178/109749259-e102b680-7b9f-11eb-882b-97a5dbe9b70a.gif)

![FirstLiftmoveSet](https://user-images.githubusercontent.com/68019178/109749277-e7912e00-7b9f-11eb-9497-275fcb495c96.gif)

## Challenges

- One thing we struggled with in the q-learning algorithm was receiving the
  appropriate reward for the action performed. With the first attempt, the
  entire algorithm ran to completion before a single reward was read in. WIth a
  second attempt, a rospy.sleep() statement was added between the movement
  publication and the update of the q-matrix. This read in rewards, but this
  still sometimes read in the wrong reward for the action. The current working
  implementation sleeps before the movement publication, after the movement
  publication, within the callback function to receive the reward, and after
  receiving a nonzero reward. This makes the algorithm slower, but appears to
  eliminate the problem of rewards not matching up.

- The arm manipulation functions had a number of challenges. First, the
  dumbbells barely seem to fit within the gripper, and finding a set of goal
  positions for which the dumbbells could be reliably grabbed, lifted, and
  carried was difficult. One of the challenges was getting the alignment
  perfect, so that the grabber would fit around the handle without knocking the
  dumbbell over, and this required precision alignment and a slow approach under
  proportional alignment and distance control. We think that the dumbbell in
  future projects could be better fit to be grabbed, as it took a lot of time to
  get a method for grabbing and raising the dumbbell without it dropping. These
  were overcome by carefully observing the way the dumbbell would fall, and
  improving the goal positions. Also, the manipulator is placed in four
  different positions for orienting toward, approaching, grabbing/raising, and
  then setting down the dumbbell. This helped to make the operation repeatable.
  It was difficult to reliably orient precisely in front of the dumbbell. To do
  this, we had to look at the scan data and not focus on the closest angle in
  the scan, but on all angles that returned distances less than .5m, and then
  use the average of these angles as the center of the dumbbell, which was then
  used in proportional control to reorient and face the dumbbell. To keep this
  from being too slow, once the orientation was within 2 degrees, the movement
  could continue. To coordinate control between nodes, we used ROS topics for
  /cmd_arm and /rsp_arm so that the nodes could be self-contained and not need
  to access internal operations or methods.

## Future work

- The q-learning algorithm currently takes on the order of 5-10 minutes to
  converge (~300 total iterations). This is because of the sleep statements,
  which were added to ensure the correct reward was read in. Reducing the sleep
  times from 1 to 0.5 seconds created additional problems with getting the
  correct reward. If we had more time, maybe we could figure out a better way to
  read in the correct reward so that the algorithm does not take so long.

- The dumbbell seems too difficult to control with the current method, if it
  were possible to find a more stable orientation for lifting and carrying the
  weights, this may help repeatability. However, it is likely that a different
  dumbbell design would help here. It’s possible that moving the robot forward,
  while lifting the dumbbell might make this more stable, and given more time,
  we would explore how robot motion along with arm motion during the lift would
  help.  It was clear that backing away from the dumbbell after depositing it
  was necessary to enable it to stay vertical.

## Takeaways

- We would tell a future group working on a similar project that by clearly
  defining the individual roles of the nodes that each partner would primarily
  design, progress can be made in parallel. This was key for us getting this
  project done because there are a lot of moving parts and it is a long project!
  Additionally, Creating customized ROS topics for communication between
  projects makes compartmentalizing the code easier. Though we had to figure out
  how to create customized messages, doing so was rewarding but you have to
  remember to do a catkin make!

- In terms of tips for tools that are helpful, we would tell a future group
  working on a similar project that becoming good with git and branches would be
  helpful, because it added some extra struggles for our group to try to all get
  on the same page without a lot of experience with github. Additionally, We
  would tell them that using the GUI was very helpful for identifying the target
  positions. It saves a lot of time, because it gave a good initial guess to
  start with.

- We would also tell a future group working on a similar project that Q learning
  can take a long time to converge and there are quirks about the messaging
  between the phantom robot and the rewards that made it difficult to have a
  fast and efficient algorithm. Despite some of the challenges, we learned that
  Q learning is great for learning tasks when there is a clearly defined reward
  and the number of possible states of the system is relatively small so that
  the robot can learn by trial and error. If the dimension of the problem is
  much larger, then a different kind of reinforcement learning would be needed.
  But this form was relatively simple and straightforward to implement. The
  phantom manipulator definitely sped up the learning process.

### TurtleBot3 executes optimal actions following convergence of the Q-matrix

[Video here](https://www.youtube.com/watch?v=0v7KDNA4S6s)
