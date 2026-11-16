"""Adapt the installed Humble Nav2 defaults, retaining its BT/plugin inventory."""
import copy


def nav2_parameters(defaults, scenario, map_path):
    params=copy.deepcopy(defaults)
    def node(name):
        return params.setdefault(name,{}).setdefault('ros__parameters',{})
    x,y,yaw=scenario['start']
    robot,laser=scenario['robot'],scenario['laser']
    node('amcl').update(base_frame_id='base_link',odom_frame_id='odom',global_frame_id='map',
                        scan_topic='/scan',set_initial_pose=True,always_reset_initial_pose=True,
                        initial_pose=dict(x=float(x),y=float(y),z=0.,yaw=float(yaw)),
                        min_particles=150,max_particles=600,max_beams=60,update_min_d=.1,update_min_a=.1,
                        laser_max_range=float(laser['range']),laser_min_range=.02,transform_tolerance=.5)
    node('map_server')['yaml_filename']=str(map_path)
    node('bt_navigator').update(robot_base_frame='base_link',global_frame='map',odom_topic='/odom')
    control=node('controller_server')
    control.update(controller_frequency=10.,min_y_velocity_threshold=.001)
    control['progress_checker'].update(required_movement_radius=.2,movement_time_allowance=20.)
    control['general_goal_checker'].update(xy_goal_tolerance=.2,yaw_goal_tolerance=.25)
    control['FollowPath'].update(max_vel_x=float(robot['max_linear']),max_speed_xy=float(robot['max_linear']),
                               max_vel_theta=float(robot['max_angular']),vx_samples=12,vy_samples=1,
                               vtheta_samples=16,sim_time=1.5,debug_trajectory_details=False,
                               trans_stopped_velocity=.03)
    for name in ('local_costmap','global_costmap'):
        cost=params[name][name]['ros__parameters']
        cost.update(robot_base_frame='base_link',robot_radius=float(robot['radius']),resolution=.1,
                    update_frequency=5. if name=='local_costmap' else 1.,publish_frequency=1.,
                    always_send_full_costmap=False)
        cost.pop('footprint',None)
        if name=='local_costmap':
            cost.update(width=6,height=6,plugins=['obstacle_layer','inflation_layer'])
        else:
            cost.update(plugins=['static_layer','obstacle_layer','inflation_layer'],track_unknown_space=True)
        cost['obstacle_layer']=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,
            observation_sources='scan',scan=dict(topic='/scan',data_type='LaserScan',clearing=True,
                marking=True,inf_is_valid=True,observation_persistence=0.,
                obstacle_min_range=.02,obstacle_max_range=float(laser['range'])-.1,
                raytrace_min_range=.0,raytrace_max_range=float(laser['range'])))
        cost['inflation_layer'].update(inflation_radius=max(.5,float(robot['radius'])+.25),cost_scaling_factor=3.)
    node('planner_server')['GridBased'].update(use_astar=True,allow_unknown=False,tolerance=.2)
    node('behavior_server').update(robot_base_frame='base_link',global_frame='odom')
    node('velocity_smoother').update(smoothing_frequency=10.,feedback='OPEN_LOOP',
        max_velocity=[float(robot['max_linear']),0.,float(robot['max_angular'])],
        min_velocity=[-float(robot['max_linear']),0.,-float(robot['max_angular'])],velocity_timeout=.5)
    # The installed template can contain nested costmaps and BT auxiliary nodes.
    def sim_time(value):
        if isinstance(value,dict):
            if 'ros__parameters' in value:
                value['ros__parameters']['use_sim_time']=True
            for child in value.values(): sim_time(child)
    sim_time(params)
    return params
