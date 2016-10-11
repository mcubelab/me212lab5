import numpy as np

joint_limits = (
    (-np.pi, np.pi),  # make joint 1 smaller from real
    (-np.pi, np.pi))

# length of two links
a1 = 0.1778
a2 = 0.235

def in_joint_range(q):
    for i, qi in enumerate(q):
        if qi < joint_limits[i][0] or qi > joint_limits[i][1]:
            return False
    return True
    
def select_best_q(candidates, q0, weight = [1,1]):
    # we prefer minimum travel
    min_v = None
    best_q = None
    for q in candidates:
        v = np.sum(((np.array(q) - np.array(q0))*np.array(weight))**2)
        if (min_v == None or min_v > v) and in_joint_range(q):
            min_v = v
            best_q = q
    return best_q

def ik(target_TCP_poselist, q0):
    x, z = target_TCP_poselist[0], target_TCP_poselist[2]
    ik_candidate = []
    
    xz2 = x**2 + z**2  ## In Python, x**y: x to the power y
    
    # candidate 1
    q_2 = ## ??
    q_1 = ## ??
    
    if not np.isnan([q_1, q_2]).any():
        ik_candidate.append([q_1, q_2])
    
    # candidate 2
    q_2 = ## ??
    q_1 = ## ??
    
    if not np.isnan([q_1, q_2]).any():
        ik_candidate.append([q_1, q_2])
    
    return select_best_q(ik_candidate, q0)

def ikv(target_TCP_vel, q0):
    J = Jacobian(q0)
    qdot = np.linalg.solve(J, target_TCP_vel).tolist()  # "J \ target_TCP_vel" in matlab
    return qdot

def Jacobian(q):
    q1 = q[0]
    q2 = q[1]
    s1 = np.sin(q1)
    s12 = np.sin(q1+q1)
    c1 = np.cos(q1)
    c12 = np.cos(q1+q1)
    return np.array([[-a1 * s1 - a2 * s12 ,  -a2 * s12  ], 
                     [a1 * c1 + a2 * c12  ,  a2 * c12 ]])

# return end point of the second link
def fk(q):
    return (a1 * np.cos(q[0]) + a2 * np.cos(q[0]+q[1]) ,
            a1 * np.sin(q[0]) + a2 * np.sin(q[0]+q[1]))

# return end point of the first link
def fk1(q):
    return (a1 * np.cos(q[0]),
            a1 * np.sin(q[0]))


