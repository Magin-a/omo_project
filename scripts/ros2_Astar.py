#!/usr/bin/env python3
import signal
import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from warnings import warn

def signal_handler(signal, frame):
    print('\nYou pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class Astar(Node):
  def __init__(self, grid_size, robot_radius, start, goal, scene_num):      
    self.grid_size = grid_size#0.08
    self.robot_radius = robot_radius#2.0
    self.allow_diagonal_movement = True
    self.scene_num = scene_num

    self.x_width = 50
    self.y_width = 50
    self.maze = np.zeros((int(self.x_width / self.grid_size), int(self.y_width / self.grid_size)), dtype = int)

    self.start_position = start#[0,0] #시작점 pose
    self.goal_position = goal#[10,0] #목표점 pose

    self.start_node = Node_Astar(None, self.calc_grid_pos(self.start_position))
    self.goal_node = Node_Astar(None, self.calc_grid_pos(self.goal_position))

    self.open_list = []
    self.closed_list = []

    self.obstacles = []  # Obstacle(x, y, r)
    self.obs_grid = []

    self.open_list.append(self.start_node) #출발점 추가

    #이동하는 칸은 십자가 모양
    self.adjacent_squares = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    if self.allow_diagonal_movement:
      self.adjacent_squares += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
            
    self.outer_iteration = 0
    self.max_iteration = (len(self.maze[len(self.maze) - 1]) // 2) ** 2

    self.set_scenario()

  def set_scenario(self):
    # map(기본 사각형)
    for i in range(-4, 5):
      self.obstacles.append(Obstacle(-1, i, 1))
      self.obstacles.append(Obstacle(8, i, 1))
    for i in range(-1, 8):
      self.obstacles.append(Obstacle(i, 4, 1))
      self.obstacles.append(Obstacle(i, -4, 1))
      
    for i in range(-2, 5):
      self.obstacles.append(Obstacle(1, i, 1))

    for i in range(3, 7):
      self.obstacles.append(Obstacle(i, 0, 1))

    for i in range(-4, 0):
      self.obstacles.append(Obstacle(6, i, 1))

    for i in range(3, 8):
      self.obstacles.append(Obstacle(i, 2, 1))

    for i in range(-4, -2):
      self.obstacles.append(Obstacle(3, i, 1))

    self.update_obstacles()

  def calc_grid_pos(self, node):
    return int(((len(self.maze) / 2 * self.grid_size) - node[1]) / self.grid_size), int(node[0] / self.grid_size)

  def update_obstacles(self):
    for obs in self.obstacles:
      obs.pos = self.calc_grid_pos((obs.x, obs.y))
      if obs.r > self.grid_size:
        k = (obs.r - self.grid_size) / self.grid_size
        for i in range(-int(k), int(k) + 1):
          for j in range(-int(k), int(k) + 1):
            if not self.check_range((obs.pos[0] + i, obs.pos[1] + j)):
              self.maze[int(obs.pos[0]) + i][int(obs.pos[1]) + j] = 9
      if not self.check_range(obs.pos):
        self.maze[int(obs.pos[0])][int(obs.pos[1])] = 9

  def check_range(self, node):
    return (
      node[0] > (len(self.maze) - 1)
      or node[0] < 0
      or node[1] > (len(self.maze[len(self.maze) - 1]) - 1)
      or node[1] < 0
    )

  #exist start_point
  def astar_condition(self):
    return len(self.open_list) > 0


  def get_the_current_node(self):
    self.current_node = self.open_list[0]
    self.current_index = 0
    for ind, item in enumerate(self.open_list):
      if item.F < self.current_node.F:
       self.current_node = item
       self.current_index = ind

      elif item.F == self.current_node.F:
        if item.H < self.current_node.H:
          self.current_node = item
          self.current_index = ind

    if self.outer_iteration > self.max_iteration:
      warn("giving up on pathfinding too many iterations")
      return return_path(self.current_node)
    
    self.open_list.pop(self.current_index)
    self.closed_list.append(self.current_node)
  

  def check_find_the_goal(self):
    if self.current_node.position == self.goal_node.position:
      return return_path(self.current_node)
  
  
  def generate_children(self):
    self.children = []
    for new_position in self.adjacent_squares:
      self.node_position = (
        self.current_node.position[0] + new_position[0],
        self.current_node.position[1] + new_position[1],
      )
      if self.check_range(self.node_position):
        continue
      if self.maze[int(self.node_position[0])][int(self.node_position[1])] != 0:
        continue
      if self.check_radius():
        continue
      self.new_node = Node_Astar(self.current_node, self.node_position)
      self.children.append(self.new_node)
  
  def check_radius(self):
    for idx, obs in enumerate(self.obstacles):
      node_x = self.node_position[1] * self.grid_size
      node_y = ((len(self.maze) / 2 * self.grid_size) - self.node_position[0] * self.grid_size)
      
      dist = math.hypot(node_x - obs.x, node_y - obs.y)
      
      if dist < self.robot_radius:
        if obs.x == 10 and node_x == 9 and node_y == 0:
          print('\n')
          print('node_grid', node_x, node_y)
          print(dist)
          print('obstacle', obs.x, obs.y)
        return True #장애물에 걸린 경우
      
    return False

  def loop_through_children(self):
    self.heuristic_weight = 1.0
    
    for child in self.children:
      if len([closed_child for closed_child in self.closed_list if closed_child == child]) > 0:
        continue
      if child.position[0] == self.current_node.position[0] or child.position[1] == self.current_node.position[1]:
        child.G = self.current_node.G + 10
      else:
        child.G = self.current_node.G + 14
      
      child.H = (abs(child.position[0] - self.goal_node.position[0]) + abs(child.position[1] - self.goal_node.position[1])) * 10
      
      child.F = child.G + self.heuristic_weight * child.H
      
      same_ = False
      for idx, open_node in enumerate(self.open_list):
        if open_node.position == child.position:
          same_ = True
          same_idx = idx
          
          if open_node.F > child.F:
            self.open_list.pop(same_idx)
            self.open_list.append(child)
            
      if same_ == False:
        self.open_list.append(child)

class Obstacle:
  def __init__(self, x, y, r):
    self.x = x
    self.y = y
    self.r = r
    self.pos = (0, 0)

class Node_Astar:
  def __init__(self, parent=None, position=None):
    super().__init__()
    self.parent = parent
    self.position = position
    self.G = 0
    self.H = 0
    self.F = 0

  def __eq__(self, other):
    return self.position == other.position

  def __str__(self):
    return str(self.position)

class ROS2(Node):
    def __init__(self):
      super().__init__('ros2_Astar')
      self.global_path_pub = self.create_publisher(Path, 'path', 10)

    def path_publish(self, total_path):
        pub_msg = Path()
        pub_msg.header.frame_id = 'odom'
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(total_path.shape[1]):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.pose.position.x = total_path[0][i]
            pose_stamped.pose.position.y = total_path[1][i]

            #print(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            pub_msg.poses.append(pose_stamped)

        self.global_path_pub.publish(pub_msg)

def return_path(current_node):
  path = []
  current = current_node
  
  while current is not None:
    path.append(current.position)
    current = current.parent
    
  return path[::-1]  # return reversed path

def main(args=None):
    grid_size = 0.017
    robot_radius = 0.2
    start_position = (0, 0)
    goal_position = (7, -3)

    scene_num = 1

    rclpy.init(args=args)
    ros = ROS2()
    astar = Astar(grid_size, robot_radius, start_position, goal_position, scene_num)
    start_time = time.time()

    while astar.astar_condition(): #path 배열에 data 존재여부
      astar.get_the_current_node()
      path_node = astar.check_find_the_goal()
      astar.generate_children()
      astar.loop_through_children()

      if path_node is not None:
        end_time = time.time()
        print('\nterminate_time: {}s'.format(end_time - start_time))
        break
    
    if path_node is None:
      print('There is no Path')
      sys.exit()

    result = np.ones((len(path_node), 2))

    for i in range(len(path_node)):
      result[i][1] = ((len(astar.maze) / 2 * astar.grid_size) - path_node[i][0] * astar.grid_size)
      result[i][0] = path_node[i][1] * astar.grid_size

    rx, ry = [], []

    for i in range(len(result)):
      rx.append(result[i][0])
      ry.append(result[i][1])
    
    r = rx + ry

    path_len = len(rx)
    path = np.reshape(r, (2, path_len))

    print("Path pub Start!!")
    while rclpy.ok():
      ros.path_publish(path)

    ros.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()