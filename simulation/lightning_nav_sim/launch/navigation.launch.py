"""Independent-world simulation + Humble Nav2/AMCL + optional RViz."""
from pathlib import Path
import copy
import hashlib
import json
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lightning_nav_sim.configuration import nav2_parameters
from lightning_nav_sim.core import World, load_scenario
from lightning_nav_sim.grid import load_grid


def start(context):
    scenario_path=Path(LaunchConfiguration('scenario').perform(context)).expanduser().resolve()
    map_path=Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    scenario=load_scenario(scenario_path)
    world=World(scenario)  # Fail before starting Nav2 if world/start geometry is invalid.
    candidate=load_grid(map_path)
    if candidate.collides(scenario['start'],scenario['robot']['radius']):
        raise ValueError('Start footprint must be free in the candidate navigation map too')
    # Validate the entire fixed goal set against truth, not against the candidate map:
    # candidate map failures (e.g. a stale vehicle at a goal) are part of the experiment.
    for pose in scenario['goals']:
        if world.grid.collides(pose,scenario['robot']['radius']):
            raise ValueError('A goal footprint intersects the truth world; fix the scenario')
    nav2=Path(get_package_share_directory('nav2_bringup'))
    own=Path(get_package_share_directory('lightning_nav_sim'))
    params=nav2_parameters(yaml.safe_load((nav2/'params/nav2_params.yaml').read_text()),scenario,map_path)
    comparable=copy.deepcopy(params)
    comparable['map_server']['ros__parameters']['yaml_filename']='<candidate-map>'
    config_hash=hashlib.sha256(json.dumps(comparable,sort_keys=True).encode()).hexdigest()
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='lightning_nav_sim_',delete=False) as stream:
        yaml.safe_dump(params,stream); parameter_path=Path(stream.name)
    def cleanup(_context):
        parameter_path.unlink(missing_ok=True)
        return []
    simulator=Node(package='lightning_nav_sim',executable='nav_sim_world',output='screen',
        parameters=[dict(scenario=str(scenario_path),navigation_map=str(map_path),
                         navigation_config_hash=config_hash,
                         rate=float(LaunchConfiguration('rate').perform(context)))])
    return [
        simulator,
        RegisterEventHandler(OnProcessExit(target_action=simulator,on_exit=[
            EmitEvent(event=Shutdown(reason='Simulation stopped'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2/'launch/bringup_launch.py')),
            launch_arguments=dict(map=str(map_path),params_file=str(parameter_path),use_sim_time='True',
                slam='False',autostart='True',use_composition='False',use_respawn='False').items()),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(own/'config/navigation.rviz')],
             parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz'))),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario',description='Absolute path to scenario.yaml'),
        DeclareLaunchArgument('map',description='Candidate Nav2 map YAML; never used as the physics world'),
        DeclareLaunchArgument('rate',default_value='0.5',description='Simulation seconds per wall second'),
        DeclareLaunchArgument('rviz',default_value='true'),
        OpaqueFunction(function=start),
    ])
