import numpy as np
import casadi as ca
import time


def MPC(self_state, goal_state, obstacles):
    opti = ca.Opti()
    ## parameters for optimization
    T = 0.2
    N = 10  # MPC horizon
    v_max = 0.5
    omega_max = 0.6
    safe_distance = 0.55
    # Significantly increased angle weight (Q[2,2]) to prioritize orientation
    Q = np.array([[1.2, 0.0, 0.0],[0.0, 1.2, 0.0],[0.0, 0.0, 20.0]])
    # Reduced angular velocity cost (R[1,1]) to allow easier turning
    # Slightly increased from 0.01 to 0.05 to provide damping and prevent overshoot
    R = np.array([[0.2, 0.0], [0.0, 0.05]])
    
    # Extract target state: x, y, yaw (from goal_state columns 0, 1, 3)
    # goal_state structure: [x, y, z, yaw]
    goal_x = goal_state[:,0]
    goal_y = goal_state[:,1]
    goal_yaw = goal_state[:,3]
    
    # Construct goal matrix for MPC (N x 3)
    goal = np.column_stack((goal_x, goal_y, goal_yaw))
    
    tra = goal_state[:,2]
    opt_x0 = opti.parameter(3)
    opt_controls = opti.variable(N, 2)
    v = opt_controls[:, 0]
    omega = opt_controls[:, 1]

    ## state variables
    opt_states = opti.variable(N+1, 3)
    x = opt_states[:, 0]
    y = opt_states[:, 1]
    theta = opt_states[:, 2]

    f = lambda x_, u_: ca.vertcat(u_[0]*ca.cos(x_[2]), u_[0]*ca.sin(x_[2]), u_[1])

    ## init_condition
    opti.subject_to(opt_states[0, :] == opt_x0.T)

    # Position Boundaries
    # Here you can customize the avoidance of local obstacles 

    # Admissable Control constraints
    # Allow slight backward motion to prevent deadlock when close to target but misaligned
    opti.subject_to(opti.bounded(-0.1, v, v_max)) 
    opti.subject_to(opti.bounded(-omega_max, omega, omega_max)) 

    # System Model constraints
    for i in range(N):
        x_next = opt_states[i, :] + T*f(opt_states[i, :], opt_controls[i, :]).T
        opti.subject_to(opt_states[i+1, :]==x_next)

    #### cost function
    obj = 0 
    
    # Check if we are very close to the final goal (using initial state)
    dist_to_goal = np.sqrt((self_state[0,0] - goal[N-1,0])**2 + (self_state[0,1] - goal[N-1,1])**2)
    
    # U-shaped angle boost profile:
    # 1. Far away (> 3.0m): Normal weight (boost = 1.0)
    # 2. Approaching (3.0m -> 2.0m): Drop weight to prioritize position (1.0 -> 0.01)
    # 3. Near (2.0m -> 0.1m): Keep low weight (0.01) to ensure reaching goal
    # 4. Final adjustment (0.1m -> 0.0m): Ramp up weight to fix orientation (0.01 -> 6.0)
    
    ang_boost = 1.0 # Default
    
    if dist_to_goal > 3.0:
        ang_boost = 1.0
    elif dist_to_goal > 2.0:
        # Linear drop from 1.0 (at 3.0m) to 0.0 (at 2.0m)
        # dist range: 1.0
        # fraction = (dist - 2.0) / 1.0  (0.0 at 2.0m, 1.0 at 3.0m)
        fraction = (dist_to_goal - 2.0) / 1.0
        ang_boost = 0.0 + (1.0 - 0.0) * fraction
    elif dist_to_goal > 0.1:
        # Keep at 0.0 (completely ignore orientation)
        ang_boost = 0.0
    else:
        # Linear ramp up from 0.0 (at 0.1m) to 1.0 (at 0.0m)
        # dist range: 0.1
        # fraction = (0.1 - dist) / 0.1 (0.0 at 0.1m, 1.0 at 0.0m)
        fraction = (0.1 - dist_to_goal) / 0.1
        # Clip fraction to avoid overshoot if dist < 0
        fraction = max(0.0, min(1.0, fraction))
        ang_boost = 0.0 + (1.0 - 0.0) * fraction
        
    ang_boost = float(ang_boost)

    # Dynamic weights
    # If near goal: 
    #   - pos_weight: keep normal (or reduce slightly to allow rotation adjustment)
    #   - angle_weight: increase massively
    # If far from goal:
    #   - pos_weight: high to reach goal
    #   - angle_weight: moderate
    
    # Base weights
    w_pos_step = 0.1
    w_ang_step = 0.1 * Q[2,2] # 2.0
    w_pos_term = 2.0
    w_ang_term = 5.0 * Q[2,2] # 100.0
    
    for i in range(N):
        # Position error cost
        pos_error = opt_states[i, 0:2] - goal[i:i+1, 0:2]
        obj = obj + w_pos_step * ca.mtimes([pos_error, Q[0:2, 0:2], pos_error.T])
        
        # Heading error cost
        ang_diff = opt_states[i, 2] - goal[i, 2]
        ang_cost = 2 * (1 - ca.cos(ang_diff))
        obj = obj + w_ang_step * ang_boost * ang_cost
    
        # Control cost
        obj = obj + ca.mtimes([opt_controls[i, :], R, opt_controls[i, :].T]) 
        # "Don't move forward if not aligned" penalty
        # We relax this when ang_boost is low (approaching goal) to allow moving to position even if angle is off
        # When ang_boost is high (at goal), this penalty kicks in to force rotation before moving (if moving is needed)
        # or to force v=0 while rotating
        
        align_penalty_weight = 0.05
        if ang_boost < 0.1: # In the "ignore angle" phase
             align_penalty_weight = 0.0 # Disable alignment penalty
             
        obj = obj + align_penalty_weight * ca.power(v[i], 2) * (1 - ca.cos(ang_diff))
        
    # Terminal cost
    pos_error_N = opt_states[N, 0:2] - goal[N-1:N, 0:2]
    obj = obj + w_pos_term * ca.mtimes([pos_error_N, Q[0:2, 0:2], pos_error_N.T])
    
    ang_diff_N = opt_states[N, 2] - goal[N-1, 2]
    ang_cost_N = 2 * (1 - ca.cos(ang_diff_N))
    obj = obj + w_ang_term * ang_boost * ang_cost_N

    opti.minimize(obj)
    opts_setting = {'ipopt.max_iter':200, 'ipopt.print_level':0, 'print_time':0, 'ipopt.tol':1e-5, 'ipopt.acceptable_tol':1e-6, 'ipopt.acceptable_obj_change_tol':1e-6}
    opti.solver('ipopt',opts_setting)
    opti.set_value(opt_x0, self_state[:,:3])

    try:
        sol = opti.solve()
        u_res = sol.value(opt_controls)
        state_res = sol.value(opt_states)
    except:
        state_res = np.repeat(self_state[:3],N+1,axis=0)
        u_res = np.zeros([N,2])

    return state_res, u_res
