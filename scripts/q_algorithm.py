#!/usr/bin/env python3

import rospy

from q_learning_project.msg import QLearningReward
from q_learning_project.msg import QMatrix
from q_learning_project.msg import RobotMoveDBToBlock
from q_learning_project.msg import Actions

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
        self.actions_pub = rospy.Publisher("q_learning/optimal_actions", Actions, queue_size=10)
        #Creating the Q Matrix structure
        self.q = np.zeros((64,9), dtype=int)
        self.publish_qmatrix()
        # initialize these values to help later with determining convergence
        self.count=0
        self.reward=0
        print("Q Matrix Initialized")
        # creating the action matrix:
        self.actions = np.zeros((64,64), dtype=int)
        self.initialize_action_matrix()

    def publish_qmatrix(self):
        matrix = QMatrix()
        matrix.header = Header(stamp=rospy.Time.now())
        matrix.q_matrix = self.q
        self.matrix_pub.publish(matrix)

    def initialize_action_matrix(self):
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
                required_action = self.find_action(moved_dumbbell, destination)
                self.actions[si][sf] = required_action
        print("Action Matrix Initialized")

    def fill_qmatrix(self):
        """ This contains the logic of the q-learning algorithm"""
        threshold = 75
        alpha = 1
        gamma = 0.5
        s = 0
        while self.count<threshold:
            print('------\nIterations without update:', self.count, '/', threshold)
            # selecting an action at random:
            all_actions = self.actions[s]
            possible_actions = all_actions[all_actions>=0]
            a = random.choice(possible_actions)
            color, block = self.inverse_action(a)
            print("Move", self.dumbbell_color(color), "to block", block)
            rospy.sleep(1.0)
            move = RobotMoveDBToBlock()
            move.robot_db = self.dumbbell_color(color)
            move.block_id = block
            self.move_pub.publish(move)
            rospy.sleep(1.0)
            # I receive reward in self.reward automatically via callback
            new_state = self.apply_action(s, a)
            #once I am in new_state, what is the max there?
            mx = np.amax(self.q[new_state])
            print("Reward:", self.reward)
            update = self.q[s][a] + alpha*(self.reward + gamma*mx - self.q[s][a])
            if self.q[s][a] != update and update <= 100:
                print('Updating the q-matrix')
                self.q[s][a] = update
                self.publish_qmatrix()
                self.count = 0
                rospy.sleep(1.0)
            else:
                self.count += 1
            # need to check if the new state is at the end
            if self.end_state(new_state):
                s = 0
            else:
                s = new_state
        print('\nQ-Learning algorithm completed!')
        try:
            print('The actions that give the highest reward are: ')
            optimal_actions = Actions()
            actions = np.arange(9)
            state = 0
            for x in range(0, 3):
                action = random.choice(actions[(self.q[state])>0])
                color, block = self.inverse_action(action)
                print(x+1,') Move',  self.dumbbell_color(color), "to block", block)
                optimal_action = RobotMoveDBToBlock()
                optimal_action.robot_db = self.dumbbell_color(color)
                optimal_action.block_id = block
                optimal_actions.actions.append(optimal_action)
                state = self.apply_action(state, action)
            self.actions_pub.publish(optimal_actions)
        except:
            print("Something went wrong. Try running again?")

    def action_reward(self, data):
        """ Callback to receive the reward from robot movement """
        self.reward = data.reward
        rospy.sleep(1.0)

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
