#!/usr/bin/python

from planner import fk1, fk
import numpy as np

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def in_collision(q, obstacle_segs):
    arm_segments = [( (0,0), fk1(q) ), ( fk1(q), fk(q))]
    
    ## return True if there are segments from arm_segments and segments from obstacle_segs intersect

if __name__=="__main__":
    
    obstacle_segs = [ [[0.2,0.2], [0.4,0.2]] ]  # line segs ((x1,z1)--(x2,z2))
    print in_collision( [0,0], obstacle_segs)
    print in_collision( [-np.pi /4*3, -np.pi /4*3], obstacle_segs)
