"""Bounded Nav2 action trial with time-matched estimated pose versus independent truth."""
import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .core import load_scenario, World, wrap


class Trial(Node):
    def __init__(self, output):
        super().__init__('navigation_trial',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.latest=None; self.pending=deque(maxlen=200)
        self.errors=[]; self.rows=0
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.create_subscription(String,'/simulation/state',self.receive,10)
        self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.lifecycle=self.create_client(GetState,'/bt_navigator/get_state')
        self.start_client=self.create_client(Trigger,'/simulation/start')
        self.stop_client=self.create_client(Trigger,'/simulation/stop')
        self.stop=self.create_publisher(Twist,'/cmd_vel',10)
        self.stream=(output/'samples.csv').open('x',newline='')
        self.writer=csv.writer(self.stream)
        self.writer.writerow(['elapsed_s','truth_x','truth_y','truth_yaw','estimate_x','estimate_y',
                              'estimate_yaw','position_error_m','yaw_error_rad','distance_m','collisions'])

    def receive(self,msg):
        state=json.loads(msg.data)
        self.latest=state
        if state['running']:
            self.pending.append(state)

    def record(self,finish=False):
        while self.pending:
            state=self.pending[0]
            try:
                tf=self.tf.lookup_transform('map','base_link',Time(seconds=state['stamp']))
                p,q=tf.transform.translation,tf.transform.rotation
                if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)):
                    raise TransformException('Nonfinite estimated pose')
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                truth=state['pose']
                error=math.hypot(p.x-truth[0],p.y-truth[1])
                yaw_error=wrap(yaw-truth[2])
                values=[p.x,p.y,yaw,error,yaw_error]
                self.errors.append((error,yaw_error))
            except TransformException:
                if not finish and self.latest['stamp']-state['stamp'] < 1.:
                    break  # Allow delayed TF messages to reach this subscriber.
                values=['']*5
            self.writer.writerow([state['elapsed'],*state['pose'],*values,state['distance'],state['collisions']])
            self.rows+=1
            self.pending.popleft()
        self.stream.flush()

    def spin(self):
        rclpy.spin_once(self,timeout_sec=.05)
        self.record()

    def wait_future(self,future,timeout):
        deadline=time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:
            self.spin()
        if not future.done():
            raise TimeoutError('ROS action/service response timed out')
        return future.result()

    def ready(self,timeout):
        deadline=time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<deadline:
            self.spin()
            if self.latest and self.latest['running']:
                raise RuntimeError('Simulation already started; restart the launch for a fresh trial')
            if not (self.latest and self.action.server_is_ready() and self.lifecycle.service_is_ready()):
                continue
            state=self.wait_future(self.lifecycle.call_async(GetState.Request()),min(2.,max(.1,deadline-time.monotonic())))
            if state.current_state.id != 3:
                continue
            if self.tf.can_transform('map','base_link',Time()):
                return
        raise TimeoutError('Nav2/AMCL not ready within startup timeout; inspect launch logs and /clock')


def execute(args, node, scenario):
    node.ready(args.startup_timeout)
    world=World(scenario)
    if (node.latest['protocol_hash'] != world.protocol_hash or node.latest['world_hash'] != world.world_hash):
        raise ValueError('Trial scenario differs from the running simulation')
    if not node.start_client.wait_for_service(timeout_sec=2.):
        raise TimeoutError('/simulation/start not available')
    response=node.wait_future(node.start_client.call_async(Trigger.Request()),3.)
    if not response.success:
        raise RuntimeError(response.message)
    metadata={k:node.latest[k] for k in ('world_hash','protocol_hash','navigation_map_hash','navigation_config_hash')}
    metadata.update(goal_timeout_s=args.goal_timeout,wall_timeout_s=args.wall_timeout,truth_tolerance_m=args.truth_tolerance,
                    truth_yaw_tolerance_rad=.4)
    results=[]
    current=None
    try:
        for goal in scenario['goals']:
            node.spin()
            before=dict(node.latest)
            msg=NavigateToPose.Goal(); msg.pose.header.frame_id='map'
            msg.pose.header.stamp=node.get_clock().now().to_msg()
            msg.pose.pose.position.x=float(goal[0]); msg.pose.pose.position.y=float(goal[1])
            msg.pose.pose.orientation.z=math.sin(goal[2]/2); msg.pose.pose.orientation.w=math.cos(goal[2]/2)
            current=node.wait_future(node.action.send_goal_async(msg),10.)
            if not current.accepted:
                results.append(dict(goal=goal,status='rejected',success=False)); break
            future=current.get_result_async()
            wall_start=time.monotonic()
            timeout=False
            while rclpy.ok() and not future.done():
                node.spin()
                if (node.latest['elapsed']-before['elapsed'] > args.goal_timeout
                        or time.monotonic()-wall_start > args.wall_timeout):
                    timeout=True
                    node.wait_future(current.cancel_goal_async(),5.)
                    node.wait_future(future,5.)
                    break
            result=future.result()
            # Receive at least one post-result truth sample, bounded even if simulator dies.
            settle=time.monotonic()+1.
            stamp=node.latest['stamp']
            while node.latest['stamp']<=stamp and time.monotonic()<settle: node.spin()
            state=node.latest
            error=math.hypot(state['pose'][0]-goal[0],state['pose'][1]-goal[1])
            yaw_error=abs(wrap(state['pose'][2]-goal[2]))
            succeeded=not timeout and result.status==GoalStatus.STATUS_SUCCEEDED
            collisions=state['collisions']-before['collisions']
            success=succeeded and error<=args.truth_tolerance and yaw_error<=.4 and collisions==0
            results.append(dict(goal=goal,status='timeout' if timeout else str(result.status),success=success,
                nav2_succeeded=succeeded,truth_goal_error_m=error,truth_yaw_error_rad=yaw_error,
                elapsed_s=state['elapsed']-before['elapsed'],path_length_m=state['distance']-before['distance'],
                collision_contacts=collisions))
            current=None
            if not success: break
    finally:
        if current is not None and current.accepted:
            try: node.wait_future(current.cancel_goal_async(),3.)
            except (TimeoutError,RuntimeError): pass
        node.stop.publish(Twist())
    errors=node.errors
    return dict(format_version=1,complete=True,scope='2d_navigation_map_evaluation_amcl',
        **metadata,goals_planned=len(scenario['goals']),goals_attempted=len(results),goals=results,
        success=bool(len(results)==len(scenario['goals']) and all(r['success'] for r in results)),
        success_rate=sum(r['success'] for r in results)/len(scenario['goals']),
        elapsed_s=node.latest['elapsed'],path_length_m=node.latest['distance'],
        collision_contacts=node.latest['collisions'],localization_samples=len(errors),
        truth_samples=node.rows,localization_coverage=len(errors)/max(node.rows,1),
        position_rmse_m=math.sqrt(sum(e[0]**2 for e in errors)/len(errors)) if errors else None,
        yaw_rmse_rad=math.sqrt(sum(e[1]**2 for e in errors)/len(errors)) if errors else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',required=True); parser.add_argument('--output',required=True)
    parser.add_argument('--startup-timeout',type=float,default=90.)
    parser.add_argument('--goal-timeout',type=float,default=180.)
    parser.add_argument('--wall-timeout',type=float,default=900.)
    parser.add_argument('--truth-tolerance',type=float,default=.35)
    args=parser.parse_args(remove_ros_args()[1:])
    if not all(math.isfinite(v) and v>0 for v in (args.startup_timeout,args.goal_timeout,args.wall_timeout,args.truth_tolerance)):
        parser.error('Timeouts and truth tolerance must be positive and finite')
    scenario=load_scenario(args.scenario)
    output=Path(args.output).resolve(); output.mkdir(parents=True,exist_ok=False)
    (output/'INCOMPLETE').write_text('Trial running; summary.json is not yet final.\n')
    rclpy.init(); node=None; report=None
    try:
        node=Trial(output)
        report=execute(args,node,scenario)
    except (Exception,KeyboardInterrupt) as exc:
        report=dict(format_version=1,complete=False,success=False,error=f'{type(exc).__name__}: {exc}')
    finally:
        if node is not None:
            node.stop.publish(Twist())
            if node.stop_client.wait_for_service(timeout_sec=1.):
                try: node.wait_future(node.stop_client.call_async(Trigger.Request()),2.)
                except (TimeoutError,RuntimeError): pass
            node.record(finish=True)
            if report is not None and report.get('complete'):
                report['truth_samples']=node.rows
                report['localization_samples']=len(node.errors)
                report['localization_coverage']=len(node.errors)/max(node.rows,1)
                report['position_rmse_m']=math.sqrt(sum(e[0]**2 for e in node.errors)/len(node.errors)) if node.errors else None
                report['yaw_rmse_rad']=math.sqrt(sum(e[1]**2 for e in node.errors)/len(node.errors)) if node.errors else None
            node.stream.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        (output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        if report.get('complete'): (output/'INCOMPLETE').unlink()
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['success'] else 1)
