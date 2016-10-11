#!/usr/bin/python

# 2.12 Lab 5 RRT for obstacle-free trajectory planning (adapted from Steve LaValle's code)
# Peter Yu Oct 2016

import sys, random, math, pygame
from math import sqrt,cos,sin,atan2
from planner import fk, fk1, joint_limits, a1, a2
import numpy as np
import sensor_msgs.msg
import rospy
from me212helper.marker_helper import createLineStripMarker, createPointMarker
from visualization_msgs.msg import Marker
import collision

#constants
XDIM = a1+a2 + 0.1
ZDIM = a1+a2 + 0.1
EPSILON = 0.01        # (rad)
TARGET_RADIUS = 0.005 # (meter)
NUMNODES = 5000

obstacle_segs = [ [[0.2,0.2], [0.4,0.2]] ]  # line segs ((x1,z1)--(x2,z2))
target_x = [0.15, 0.15]
#obstacle_segs = []

def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_toward(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

class Node:
    def __init__(self, q, parent_id = None): # q: joint position (q1, q2), x: location of the TCP (x, z)
        self.q = q
        self.parent_id = parent_id

def find_nearest_node(nodes, q):
    min_index = 0
    for i, p in enumerate(nodes):
        if dist(q, p.q) < dist(q, nodes[min_index].q):
            min_index = i
    return min_index

def in_workspace(q):
    x = fk(q)
    return x[0] >= 0 and x[0] <= XDIM and x[1] >= 0 and x[1] <= ZDIM 

def backtrace(nodes, new_node):
    plan = []
    curr_node = new_node
    while True:
        plan.append(curr_node.q)
        if curr_node.parent_id is None:
            break
        curr_node = nodes[ curr_node.parent_id ]
        
    plan.reverse()
    return plan



def rrt(target_x, q0, NIter = 10000, pub = None, vis_pub= None):
    
    nodes = [ Node(q0) ]
    
    if vis_pub is not None:
        vis_pub.publish(createPointMarker([[target_x[0], 0, target_x[1]]], 2, namespace="", rgba=(0,0,1,1), frame_id = '/arm_base'))
    
    for i in xrange(NIter):
        print 'i', i
        # pick a node in work space randomly
        q_rand = [ np.random.uniform(joint_limits[0][0], joint_limits[0][1]), 
                   np.random.uniform(joint_limits[1][0], joint_limits[1][1]) ]
        nearest_node_index = find_nearest_node (nodes, q_rand)
        new_q = step_from_toward(nodes[nearest_node_index].q, q_rand)
        
        #print 'new_q', new_q
        #print 'in_workspace(new_q)', in_workspace(new_q)
        #print 'in_collision(new_q)', in_collision(new_q)
        
        if pub is not None:
            js = sensor_msgs.msg.JointState(name=['joint1', 'joint2'], position = (new_q[0] - np.pi/2, new_q[1]))
            pub.publish(js)
        
        if vis_pub is not None:
            xz = fk(new_q)
            vis_pub.publish(createPointMarker([[xz[0], 0, xz[1]]], i+6, namespace="", rgba=(0,1,0,1), frame_id = '/arm_base'))
        
        if in_workspace(new_q) and not collision.in_collision(new_q, obstacle_segs):
            new_node = Node(new_q, nearest_node_index)
            nodes.append(new_node)
            
            if dist(fk(new_q), target_x) < TARGET_RADIUS:
                plan = backtrace(nodes, new_node)
                
                return plan
    
    return None # no plan found

def main():
    rospy.init_node("test_rrt")
    exec_joint_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10) 
    
    use_real_arm = rospy.get_param('/real_arm', False)
    
    rospy.sleep(0.5)
    
    vis_pub.publish(Marker(action=3)) # delete all
    vis_pub.publish(createLineStripMarker([[obstacle_segs[0][0][0], 0, obstacle_segs[0][0][1]] , [obstacle_segs[0][1][0], 0, obstacle_segs[0][1][1]]], 
                    marker_id = 5, rgba = (1,0,0,1), pose=[0,0,0,0,0,0,1], frame_id = '/arm_base'))
    
    rospy.sleep(0.5)
    plan = rrt(target_x = target_x, q0 = (np.pi/2 - 0.01, 0.0), pub = exec_joint_pub, vis_pub= vis_pub)
    
    for p in plan:
        if not use_real_arm:
            js = sensor_msgs.msg.JointState(name=['joint1', 'joint2'], position = (p[0] - np.pi/2, p[1]))
            exec_joint_pub.publish(js)
        else:
            exec_joint1_pub.publish(std_msgs.msg.Float64(p[0] - np.pi/2))
            exec_joint2_pub.publish(std_msgs.msg.Float64(p[1]))
        
        rospy.sleep(0.01)
        
    print plan
    

# if python says run, then we should run
if __name__ == '__main__':
    main()


