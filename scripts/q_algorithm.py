#!/usr/bin/env python3

import rospy

from q_learning_project.msg import QLearningReward
from q_learning_project.msg import QMatrix
from q_learning_project.msg import RobotMoveDBToBlock

from std_msgs.msg import Header
import time
import numpy as np
import random
np.set_printoptions(threshold=np.inf)


class QLearn(object):
    def __init__(self):
        rospy.init_node('q_learning_algorithm')
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.action_reward)
        self.matrix_pub = rospy.Publisher("q_learning/q_matrix", QMatrix, queue_size=10)
        self.move_pub = rospy.Publisher("q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)
        #Creating the Q Matrix structure
        self.q = []
        #There are 9 possible actions (all initialized to 0):
        a = [0]*9
        #There are 64 possible states
        for x in range(0, 64):
            self.q.append(a)
        self.q = np.array(self.q)
        matrix = QMatrix()
        matrix.header = Header(stamp=rospy.Time.now())
        matrix.q_matrix = self.q
        self.matrix_pub.publish(matrix)
        # initialize these values to help later with determining convergence
        self.count=0
        self.reward=0
        print("Q Matrix Initialized")
        # creating the action matrix:
        self.actions = np.zeros((64,64), dtype=int)
        for si in range(0, 64):
            for sf in range(0, 64):
                # si is the initial state, sf is the final state
                if si == sf:
                    # you cannot transition into the same state
                    self.actions[si][sf] = -1
                    continue
                ri, gi, bi = self.locations_from_state(si)
                initial = np.array([ri, gi, bi])
                rf, gf, bf = self.locations_from_state(sf)
                final = np.array([rf, gf, bf])
                diff = final - initial
                if np.count_nonzero(diff) > 1:
                    #More than one dumbbell has moved
                    self.actions[si][sf] = -1
                    continue
                if np.count_nonzero(initial)>np.count_nonzero(final):
                    #A dumbbell got moved to the origin
                    self.actions[si][sf] = -1
                    continue
                if not (self.valid_state(rf, gf, bf)):
                    #Can't transition to an invalid state
                    self.actions[si][sf] = -1
                    continue
                moved_dumbbell = np.nonzero(diff)[0][0] # the nonzero index of diff
                source = initial[moved_dumbbell]
                destination = final[moved_dumbbell]
                if source != 0:
                    # moving between blocks is not allowed
                    self.actions[si][sf] = -1
                    continue
                #print("Start: ", ri, gi, bi, "finish:", rf, gf, bf)
                required_action = self.find_action(moved_dumbbell, destination)
                self.actions[si][sf] = required_action
        print("Action Matrix Initialized")
        #self.test_state()
        #self.test_action()
        #print(self.actions)

    def fill_qmatrix(self):
        threshold = 50
        alpha = 1
        gamma = 0.5
        s = 0
        while self.count<threshold:
            print(self.count)
            #print('state:', s)
            # selecting an action at random:
            all_actions = self.actions[s]
            possible_actions = all_actions[all_actions>=0]
            a = random.choice(possible_actions)
            #print('selected action:', a)
            color, block = self.inverse_action(a)
            #print("Move", self.dumbbell_color(color), "to", block)
            move = RobotMoveDBToBlock()
            move.robot_db = self.dumbbell_color(color)
            move.block_id = block
            self.move_pub.publish(move)
            rospy.sleep(1.0)
            # I receive reward in self.reward automatically via callback
            new_state = self.apply_action(s, a)
            #once I am in new_state, what is the max there?
            mx = np.amax(self.q[new_state])
            print("REWARD:", self.reward)
            update = self.q[s][a] + alpha*(self.reward + gamma*mx - self.q[s][a])
            if self.q[s][a] != update:
                print('updating')
                self.q[s][a] = update
                matrix = QMatrix()
                matrix.header = Header(stamp=rospy.Time.now())
                matrix.q_matrix = self.q
                self.matrix_pub.publish(matrix)
                self.count = 0
                print(self.q)
            else:
                print('no update')
                self.count += 1
            # need to check if the new state is at the end
            if self.end_state(new_state):
                print('nowhere else to go')
                s = 0
            else:
                s = new_state
        print(self.q)

    def action_reward(self, data):
        """ Callback to receive the reward from robot movement """
        print("action reward:", data.reward)
        self.reward = data.reward

    def find_state(self, red, green, blue):
        """ Simple way to assign all 64 states to a unique number.
            Assumes a dumbbell at the origin is assigned 0
            and a dumbbell at a block is assigned the block's number.
        """
        state = 1*red + 4*green + 16*blue
        return state

    def locations_from_state(self, state):
        """ Inverse of the 'find state' function """
        blue = state // 16
        green = (state - 16*blue) // 4
        red = state - 16*blue - 4*green
        return red, green, blue

    def test_state(self):
        """ Helper function to test state functions"""
        for b in range(0, 4):
            for g in range(0, 4):
                for r in range(0, 4):
                    print("red:", r, "green:", g, "blue:", b, end='\t')
                    num = self.find_state(r,g,b)
                    print("State num:", num)
                    red, green, blue = self.locations_from_state(num)
                    print("red:", red, "green:", green, "blue:", blue, "\n")

    def find_action(self, color, block):
        """ Assumes red=0, green=1, blue=2, block = its number """
        action = color*3+(block-1)
        return action

    def inverse_action(self, action):
        """ Returns the color and block location from an action"""
        color = action // 3
        block = action-color*3 + 1
        return color, block

    def dumbbell_color(self, color):
        """ Mapping b/w dumbbells numbers and colors """
        if color == 0:
            return "red"
        elif color == 1:
            return "green"
        elif color == 2:
            return "blue"
        else:
            return "UNKNOWN COLOR??"

    def test_action(self):
        """ Helper to test the action functions """
        for color in range(0, 3):
            for block in range(1, 4):
                print("Moving the", self.dumbbell_color(color), "to block", block)
                action = self.find_action(color, block)
                print('Action number', action)
                c, b = self.inverse_action(action)
                print('Inverting: moving', self.dumbbell_color(c), "to", b, '\n')

    def valid_state(self, r, g, b):
        """ Given dumbbell locations, is this state allowed? """
        colors = [r, g, b]
        n1 = colors.count(1)
        n2 = colors.count(2)
        n3 = colors.count(3)
        if n1>1 or n2>1 or n3>1:
            return False
        else:
            return True

    def apply_action(self, state, action):
        """ Given a state and an action, return the next state"""
        r, g, b = self.locations_from_state(state)
        color, block = self.inverse_action(action)
        if color==0:
            r = block
        elif color==1:
            g = block
        elif color==2:
            b = block
        else:
            print("WHATT???")
        new_state = self.find_state(r, g, b)
        return new_state

    def end_state(self, state):
        """ Given a state, are there no more legal moves? """
        red, green, blue = self.locations_from_state(state)
        if red != 0 and green != 0 and blue != 0:
            return True
        else:
            return False

    def run(self):
        rospy.spin()


if __name__=="__main__":

    node = QLearn()
    node.fill_qmatrix()
    node.run()
