#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float32MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist, TransformStamped
import transforms3d
from nav_msgs.msg import Path, Odometry, OccupancyGrid
import numpy as np
from MPC import MPC
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class Local_Planner(Node):
    def __init__(self):
        super().__init__('local_planner')
        self.declare_parameter('replan_period', 0.01)
        self.replan_period = float(self.get_parameter('replan_period').value)
        self.curr_state = np.zeros(5)
        self.z = 0
        self.N = 10
        self.goal_state = np.zeros([self.N,4])
        self.ref_path_close_set = False
        self.target_state = np.array([-1,4,np.pi/2])
        self.target_state_close = np.zeros(3)
        self.desired_global_path = [ np.zeros([300,4]) , 0]
        self.have_plan = False
        self.is_close = False
        self.is_get = False
        self.is_grasp = False
        self.is_all_task_down = False
        self.robot_state_set = False
        self.ref_path_set = False
        self.ob=[]
        self.is_end=0
        self.ob_total = []
        self.__timer_replan = self.create_timer(self.replan_period, self.__replan_cb)
        self.final_goal_yaw = None # Store user's desired final orientation
        
        self.__sub_curr_state = self.create_subscription(Float32MultiArray, '/curr_state', self.__curr_pose_cb, 10)
        self.__sub_obs = self.create_subscription(Float32MultiArray, '/obs', self.__obs_cb, 10)
        self.__sub_goal_state = self.create_subscription(Float32MultiArray, '/global_path', self._global_path_callback, 10)
        self.__sub_waypoints = self.create_subscription(Path, '/waypoints', self.__waypoints_cb, 10)
        self.__pub_local_path = self.create_publisher(Path, '/local_path', 10)
        self.__pub_local_plan = self.create_publisher(Float32MultiArray, '/local_plan', 10)
        self.control_cmd = Twist()
        self.times = 0
        self.obstacle_markerarray = MarkerArray()
        self.ob_pub = self.create_publisher(MarkerArray, '/ob_draw', 10)
        

    def distance_sqaure(self,c1,c2):
        distance = (c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1])
        return distance

    def draw_ob(self):
        self.obstacle_markerarray.markers=[]
        num = 0
        for i in range(len(self.ob)):
            t_ob = Marker()
            t_ob.header.frame_id = "world"
            t_ob.id = num
            t_ob.type = t_ob.CYLINDER
            t_ob.action = t_ob.ADD
            t_ob.pose.position.x = self.ob[i][0]
            t_ob.pose.position.y = self.ob[i][1]
            t_ob.pose.position.z=0.2
            t_ob.scale.x = 0.1
            t_ob.scale.y = 0.1
            t_ob.scale.z = 0.4
            t_ob.color.a= 1
            t_ob.color.r = 0
            t_ob.color.g = 1
            t_ob.color.b = 0
            self.obstacle_markerarray.markers.append(t_ob)
            num = num +1
        self.ob_pub.publish(self.obstacle_markerarray)

    def _scan_callback(self, data):
        self.ob = []
        phi = data.angle_min
        point_last = np.array([100, 100])
        for r in data.ranges:
            point = np.array([self.curr_state[0]+r*np.cos(phi+self.curr_state[2]),self.curr_state[1]+r*np.sin(phi+self.curr_state[2])])
            if (r >= data.range_min and r <= data.range_max and r<=1.0 and self.distance_sqaure(point,point_last) > 0.04 ):
                self.ob.append( point )
                point_last = point
            phi += data.angle_increment
        self.draw_ob()

    def __obs_cb(self, data):
        self.ob = []
        if(len(data.data)!=0):
            size = len(data.data)//3
            for i in range(size):
                self.ob.append(( (data.data[3*i]//0.3)*0.3, (data.data[3*i+1]//0.3)*0.3) )
            dic = list(set([tuple(t) for t in self.ob]))
            self.ob = [list(v) for v in dic]
            self.draw_ob()

    def __replan_cb(self):
        if self.robot_state_set and self.ref_path_set:
            target = []
            self.choose_goal_state()        ##  gobal planning
            dist = 1
            goal = np.array([self.target_state[0], self.target_state[1], self.target_state[2]])
            start_time = self.get_clock().now()
            states_sol, input_sol = MPC(np.expand_dims(self.curr_state, axis=0),self.goal_state,self.ob) ##  gobal planning
            end_time = self.get_clock().now()
            # self.get_logger().info('[local_planner] solved in {:.3f} sec'.format((end_time-start_time).nanoseconds/1e9))

            if(self.is_end == 0):
                self.__publish_local_plan(input_sol,states_sol)
            self.have_plan = True
        elif self.robot_state_set==False and self.ref_path_set==True:
            # print("no pose")
            pass
        elif self.robot_state_set==True and self.ref_path_set==False:
            # print("no path")
            pass
        else:
            # print("no path and no pose")
            pass
        

    def __publish_local_plan(self,input_sol,state_sol):
        local_path = Path()
        local_plan = Float32MultiArray()
        local_path.header.stamp = self.get_clock().now().to_msg()
        local_path.header.frame_id = "world"

        # Log the offset for debugging
        dx = self.goal_state[0,0] - self.curr_state[0]
        dy = self.goal_state[0,1] - self.curr_state[1]
        dth = self.goal_state[0,3] - self.curr_state[2]
        # Normalize dth
        while dth > np.pi: dth -= 2*np.pi
        while dth < -np.pi: dth += 2*np.pi
        
        target_yaw = self.goal_state[0, 3]
        self.get_logger().info(f"Curr: ({self.curr_state[0]:.2f}, {self.curr_state[1]:.2f}, {self.curr_state[2]:.2f}) Goal: ({self.goal_state[0,0]:.2f}, {self.goal_state[0,1]:.2f}, {target_yaw:.2f}) Error(Goal-Curr): {dth:.3f} rad")

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i,0]
            this_pose_stamped.pose.position.y = state_sol[i,1]
            this_pose_stamped.pose.position.z = self.z+0.5 #self.desired_global_path[0][0,2]
            this_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            this_pose_stamped.header.frame_id="world"
            local_path.poses.append(this_pose_stamped)
            
            for j in range(2):
                local_plan.data.append(input_sol[i][j])

        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)
        try:
            # self.get_logger().info(f"[local_planner] publish local_plan first=({local_plan.data[0]:.3f},{local_plan.data[1]:.3f})")
            pass
        except Exception:
            pass

    def distance_global(self,c1,c2):
        distance = np.sqrt((c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1]))
        return distance
    

    def find_min_distance(self,c1):
        number =  np.argmin( np.array([self.distance_global(c1,self.desired_global_path[0][i]) for i in range(self.desired_global_path[1])]) )
        return number

    def choose_goal_state(self):
        num = self.find_min_distance(self.curr_state)
        scale = 1
        num_list = []
        for i in range(self.N):  
            num_path = min(self.desired_global_path[1]-1,int(num+i*scale))
            num_list.append(num_path)
        # If we are near the end, force the goal state to be the final point for all N steps
        # This ensures we don't just look at intermediate points when we are basically there
        # num is the index of the closest point on path
        
        if(num  >= self.desired_global_path[1]):
            self.is_end = 1
        for k in range(self.N):
            self.goal_state[k] = self.desired_global_path[0][num_list[k]]
        # If close to final waypoint, enforce final yaw across horizon
        size = self.desired_global_path[1]
        if size > 0:
            final_yaw = self.desired_global_path[0][size-1, 3]
            dist_to_end = self.distance_global(self.curr_state, self.desired_global_path[0][size-1])
            if dist_to_end < 0.7:
                for k in range(self.N):
                    self.goal_state[k][3] = final_yaw
        # print(self.goal_state)

    def __curr_pose_cb(self, data):
        self.robot_state_set = True
        self.curr_state[0] = data.data[0]
        self.curr_state[1] = data.data[1]
        self.curr_state[2] = data.data[3]
        self.curr_state[3] = data.data[4]
        self.curr_state[4] = data.data[5]
 
        self.z = data.data[2]

    def __waypoints_cb(self, msg):
        if len(msg.poses) > 0:
            q = msg.poses[0].pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.final_goal_yaw = np.arctan2(siny_cosp, cosy_cosp)
            self.get_logger().info(f"Received new goal with desired yaw: {self.final_goal_yaw:.3f}")
            if self.ref_path_set:
                self._apply_final_yaw()

    def _global_path_callback(self, data):
        if(len(data.data)!=0):
            self.ref_path_set = True
            size = len(data.data)//3
            self.get_logger().info(f"Received global path with {size} points")
            self.desired_global_path[1]=size
            for i in range(size):
                # Read data sequentially (Assuming input is Start -> Goal) from global_planning_node (stride 3)
                self.desired_global_path[0][i,0]=data.data[3*i]
                self.desired_global_path[0][i,1]=data.data[3*i+1]
                self.desired_global_path[0][i,2]=data.data[3*i+2]
                
                # Compute yaw from adjacent points with lookahead window for smoothing
                lookahead = 5 # Look 5 points ahead (~0.5m) to smooth grid artifacts
                next_idx = min(i + lookahead, size - 1)
                
                if next_idx > i:
                    dx = data.data[3*next_idx] - data.data[3*i]
                    dy = data.data[3*next_idx+1] - data.data[3*i+1]
                    self.desired_global_path[0][i,3] = np.arctan2(dy, dx)
                else:
                    # Use previous yaw for the last point(s)
                    self.desired_global_path[0][i,3] = self.desired_global_path[0][i-1,3] if i > 0 else 0.0
            
            if self.final_goal_yaw is not None:
                self._apply_final_yaw()

    def _apply_final_yaw(self):
        size = self.desired_global_path[1]
        if size <= 0 or self.final_goal_yaw is None:
            return
        force_window = 5
        blend_window = 15
        for k in range(min(force_window, size)):
            idx = size - 1 - k
            self.desired_global_path[0][idx, 3] = self.final_goal_yaw
        for k in range(force_window, min(blend_window, size)):
            idx = size - 1 - k
            alpha = (k - force_window) / (blend_window - force_window)
            yaw_path = self.desired_global_path[0][idx, 3]
            yaw_final = self.final_goal_yaw
            diff = yaw_final - yaw_path
            while diff > np.pi: diff -= 2*np.pi
            while diff < -np.pi: diff += 2*np.pi
            self.desired_global_path[0][idx, 3] = yaw_path + diff * (1.0 - alpha)

    def _global_path_callback2(self, data):
        if(len(data.data)!=0):
            self.ref_path_set = True
            size = len(data.data)//5
            self.desired_global_path[1]=size
            for i in range(size):
                # Read data sequentially (Assuming input is Start -> Goal)
                self.desired_global_path[0][i,0]=data.data[5*i]
                self.desired_global_path[0][i,1]=data.data[5*i+1]
                # Use input Z (index 2) instead of GPR predicted Z (index 3)
                self.desired_global_path[0][i,2]=data.data[5*i+2] 
                
                # Compute yaw from adjacent points
                if i < size - 1:
                    dx = data.data[5*(i+1)] - data.data[5*i]
                    dy = data.data[5*(i+1)+1] - data.data[5*i+1]
                    self.desired_global_path[0][i,3] = np.arctan2(dy, dx)
                else:
                    # Use previous yaw for the last point
                    self.desired_global_path[0][i,3] = self.desired_global_path[0][i-1,3] if i > 0 else 0.0
            
    



def main():
    rclpy.init()
    node = Local_Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
