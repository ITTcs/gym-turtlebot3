#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert, Eduardo #

from geometry_msgs.msg import Pose

class Respawn():
    def __init__(self):
        self.goal_position = Pose()
        self.goal_x_list = None
        self.goal_y_list = None
        self.len_goal_list = None
        self.index = None
        self.last_index = None
        self.init_goal_x = None
        self.init_goal_y = None
        self.goal_position.position.x = None
        self.goal_position.position.y = None

    def initIndex(self):
        self.index = 0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.last_index = self.index

    def setGoalList(self, goal_list):
        self.goal_x_list = [p[0] for p in goal_list]
        self.goal_y_list = [p[1] for p in goal_list]
        self.len_goal_list = len(self.goal_x_list)
        self.init_goal_x = self.goal_x_list[0]
        self.init_goal_y = self.goal_y_list[0]
        self.initIndex()

    def getPosition(self, position_check=False):

        if position_check:
            self.index = (self.last_index + 1) % self.len_goal_list
            self.last_index = self.index

            self.goal_position.position.x = self.goal_x_list[self.index]
            self.goal_position.position.y = self.goal_y_list[self.index]

        print(f'New goal position: {self.goal_position.position.x:.2f}, {self.goal_position.position.y:.2f}')

        return self.goal_position.position.x, self.goal_position.position.y
