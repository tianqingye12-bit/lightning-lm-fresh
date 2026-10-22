"""ROS2 adapter. Only AMCL publishes map->odom; truth never drives that transform."""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .core import World, load_scenario
from .grid import load_grid


def orientation(message, yaw):
    message.z, message.w = math.sin(yaw/2), math.cos(yaw/2)


class Simulator(Node):
    def __init__(self):
        super().__init__('navigation_simulator')
        scenario_path = self.declare_parameter('scenario','').value
        map_path = self.declare_parameter('navigation_map','').value
        rate = float(self.declare_parameter('rate',.5).value)
        if not .1 <= rate <= 2.:
            raise ValueError('Simulation rate must be 0.1..2')
        self.world = World(load_scenario(scenario_path))
        self.navigation_hash = load_grid(map_path).fingerprint()
        self.configuration_hash = self.declare_parameter('navigation_config_hash','').value
        self.running = False
        self.started = False
        self.sim_time = 1.  # A nonzero /clock allows Nav2 to initialize before motion starts.
        self.last_command = -float('inf')
        self.command = (0.,0.)
        self.last_scan = -float('inf')
        self.clock_pub = self.create_publisher(ClockMsg,'/clock',10)
        self.scan_pub = self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry,'/odom',10)
        self.truth_pub = self.create_publisher(PoseStamped,'/simulation/ground_truth',10)
        self.state_pub = self.create_publisher(String,'/simulation/state',10)
        self.markers_pub = self.create_publisher(MarkerArray,'/simulation/markers',10)
        latched = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.world_pub = self.create_publisher(OccupancyGrid,'/simulation/world',latched)
        self.create_subscription(Twist,'/cmd_vel',self.receive_command,10)
        self.create_service(Trigger,'/simulation/start',self.start)
        self.create_service(Trigger,'/simulation/stop',self.stop_simulation)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        laser = TransformStamped()
        laser.header.frame_id='base_link'; laser.child_frame_id='base_laser'
        laser.transform.translation.z=.2; laser.transform.rotation.w=1.
        self.static_tf.sendTransform(laser)
        grid = self.world.grid
        msg = OccupancyGrid()
        msg.header.frame_id='map'; msg.info.resolution=float(grid.resolution)
        msg.info.height,msg.info.width=map(int,grid.cells.shape)
        msg.info.origin.position.x,msg.info.origin.position.y=grid.origin[:2]
        orientation(msg.info.origin.orientation,grid.origin[2])
        msg.data=grid.cells.ravel().tolist()
        self.world_pub.publish(msg)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(self.world.cfg['robot']['dt']/rate,self.tick,clock=self.wall_clock)
        self.get_logger().info('Paused for /simulation/start; trial runner starts it automatically. Sensors and /clock are active.')

    def receive_command(self,msg):
        self.command=(msg.linear.x,msg.angular.z)
        self.last_command=self.sim_time

    def start(self,request,response):
        response.success=not self.started
        response.message='Started' if response.success else 'Already started; restart the launch for a fresh trial'
        if response.success:
            self.running=self.started=True
            self.last_command=-float('inf')
            self.command=(0.,0.)
            self.world.rng=np.random.default_rng(self.world.cfg['seed'])
        return response

    def stop_simulation(self,request,response):
        self.running=False
        self.command=(0.,0.)
        self.last_command=-float('inf')
        response.success=True; response.message='Motion stopped; relaunch to run another trial'
        return response

    def tick(self):
        dt=self.world.cfg['robot']['dt']
        command=self.command if self.sim_time-self.last_command < self.world.cfg['robot']['command_timeout'] else (0.,0.)
        v,w=self.world.step(*command) if self.running else (0.,0.)
        self.sim_time+=dt
        seconds=int(self.sim_time)
        clock=ClockMsg(); clock.clock.sec=seconds
        clock.clock.nanosec=int(round((self.sim_time-seconds)*1e9))
        if clock.clock.nanosec == 1_000_000_000:
            clock.clock.sec+=1; clock.clock.nanosec=0
        self.clock_pub.publish(clock)
        stamp=clock.clock
        x,y,yaw=self.world.odom
        odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id='odom'; odom.child_frame_id='base_link'
        odom.pose.pose.position.x=float(x); odom.pose.pose.position.y=float(y)
        orientation(odom.pose.pose.orientation,float(yaw))
        odom.twist.twist.linear.x=v; odom.twist.twist.angular.z=w
        odom.pose.covariance[0]=odom.pose.covariance[7]=.01; odom.pose.covariance[35]=.005
        self.odom_pub.publish(odom)
        transform=TransformStamped(); transform.header=odom.header; transform.child_frame_id='base_link'
        transform.transform.translation.x=float(x); transform.transform.translation.y=float(y)
        transform.transform.rotation=odom.pose.pose.orientation
        self.tf.sendTransform(transform)
        truth=PoseStamped(); truth.header.stamp=stamp; truth.header.frame_id='map'
        truth.pose.position.x=float(self.world.pose[0]); truth.pose.position.y=float(self.world.pose[1])
        orientation(truth.pose.orientation,float(self.world.pose[2])); self.truth_pub.publish(truth)
        if self.sim_time-self.last_scan+1e-9 >= self.world.cfg['laser']['period']:
            self.last_scan=self.sim_time
            angles,ranges=self.world.scan()
            scan=LaserScan(); scan.header.stamp=stamp; scan.header.frame_id='base_laser'
            scan.angle_min=float(angles[0]); scan.angle_max=float(angles[-1])
            scan.angle_increment=float(angles[1]-angles[0]); scan.scan_time=float(self.world.cfg['laser']['period'])
            scan.time_increment=0.; scan.range_min=.02; scan.range_max=float(self.world.cfg['laser']['range'])
            scan.ranges=ranges.astype(np.float32).tolist(); self.scan_pub.publish(scan)
            state=dict(stamp=self.sim_time,elapsed=self.world.elapsed,pose=self.world.pose.tolist(),
                       distance=self.world.distance,collisions=self.world.collisions,contact=self.world.contact,
                       running=self.running,world_hash=self.world.world_hash,protocol_hash=self.world.protocol_hash,
                       navigation_map_hash=self.navigation_hash,navigation_config_hash=self.configuration_hash)
            message=String(); message.data=json.dumps(state); self.state_pub.publish(message)
            markers=MarkerArray()
            geometry=[(self.world.pose[:2],np.array([self.world.cfg['robot']['radius']]*2),0.)]
            geometry.extend(self.world.boxes(self.world.elapsed))
            for i,(center,half,box_yaw) in enumerate(geometry):
                marker=Marker(); marker.header=truth.header; marker.ns='simulation'; marker.id=i
                marker.type=Marker.CYLINDER if i==0 else Marker.CUBE; marker.action=Marker.ADD
                marker.pose.position.x=float(center[0]); marker.pose.position.y=float(center[1]); marker.pose.position.z=.2
                orientation(marker.pose.orientation,box_yaw)
                marker.scale.x=float(2*half[0]); marker.scale.y=float(2*half[1]); marker.scale.z=.4
                marker.color.r,marker.color.g,marker.color.b,marker.color.a=(.1,.8,.2,.9) if i==0 else (.9,.4,.1,.9)
                markers.markers.append(marker)
            self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=Simulator(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
