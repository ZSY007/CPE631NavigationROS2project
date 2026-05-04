import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def _resolve_map_file(context, pkg_share: str):
    map_override_raw = LaunchConfiguration('map').perform(context)
    map_file_raw = map_override_raw if map_override_raw else LaunchConfiguration('map_file').perform(context)
    map_file_stripped = map_file_raw.strip()
    if (
        len(map_file_stripped) >= 2
        and map_file_stripped[0] == map_file_stripped[-1]
        and map_file_stripped[0] in ("'", '"')
    ):
        map_file_stripped = map_file_stripped[1:-1].strip()

    if map_file_stripped.startswith('file://'):
        map_file_stripped = map_file_stripped[len('file://'):]

    map_file_expanded = os.path.expandvars(os.path.expanduser(map_file_stripped))

    if not map_file_expanded:
        resolved_map_file = ''
    elif os.path.isabs(map_file_expanded):
        resolved_map_file = os.path.normpath(map_file_expanded)
    else:
        candidate_from_cwd = os.path.normpath(os.path.join(os.getcwd(), map_file_expanded))
        if os.path.exists(candidate_from_cwd):
            resolved_map_file = candidate_from_cwd
        else:
            candidate_from_pkg = os.path.normpath(os.path.join(pkg_share, map_file_expanded))
            resolved_map_file = candidate_from_pkg if os.path.exists(candidate_from_pkg) else candidate_from_cwd

    return [SetLaunchConfiguration('map_file_resolved', resolved_map_file)]


def _select_nav2_params(
    context,
    dynamic_params: str,
    dynamic_astar_params: str,
    dynamic_dstar_params: str,
    social_params: str,
    social_astar_params: str,
    social_dstar_params: str,
    social_smac_params: str,
    dstar_plus_params: str,
):
    social_navigation = LaunchConfiguration('social_navigation').perform(context).lower()
    planner_variant = LaunchConfiguration('planner_variant').perform(context).lower()
    if planner_variant == 'dstarplus':
        params_file = dstar_plus_params
    elif social_navigation in ('1', 'true', 'yes', 'on'):
        if planner_variant == 'astar':
            params_file = social_astar_params
        elif planner_variant == 'dstar':
            params_file = social_dstar_params
        elif planner_variant == 'smac':
            params_file = social_smac_params
        else:
            params_file = social_params
    elif planner_variant == 'astar':
        params_file = dynamic_astar_params
    elif planner_variant == 'dstar':
        params_file = dynamic_dstar_params
    else:
        params_file = dynamic_params
    return [SetLaunchConfiguration('nav2_params_resolved', params_file)]


def generate_launch_description():
    pkg_share = get_package_share_directory('cpe631_ros2')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    prefix_path = os.environ.get('AMENT_PREFIX_PATH', '/opt/ros/jazzy').split(':')[0] or '/opt/ros/jazzy'
    world_file = os.path.join(pkg_share, 'worlds', 'cafe.world')
    table_model = os.path.join(pkg_share, 'models', 'cafe_table', 'model.sdf')
    tb3_bridge = os.path.join(pkg_share, 'param', 'turtlebot3_burger_bridge_local.yaml')
    nav2_dynamic_params = os.path.join(pkg_share, 'param', 'nav2_dynamic_conservative.yaml')
    nav2_dynamic_astar_params = os.path.join(pkg_share, 'param', 'nav2_dynamic_astar.yaml')
    nav2_dynamic_dstar_params = os.path.join(pkg_share, 'param', 'nav2_dynamic_dstar.yaml')
    nav2_social_params = os.path.join(pkg_share, 'param', 'nav2_social_dynamic.yaml')
    nav2_social_astar_params = os.path.join(pkg_share, 'param', 'nav2_social_astar.yaml')
    nav2_social_dstar_params = os.path.join(pkg_share, 'param', 'nav2_social_dstar.yaml')
    nav2_social_smac_params = os.path.join(pkg_share, 'param', 'nav2_social_smac.yaml')
    nav2_dstar_plus_params = os.path.join(pkg_share, 'param', 'nav2_dstar_plus.yaml')
    social_pose_bridge = os.path.join(pkg_share, 'param', 'social_pose_bridge.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'cafe.yaml')
    # Reuse the known-good RViz layout / tools from the standard navigation launch.
    rviz_config = os.path.join(pkg_share, 'rviz', 'navigation.rviz')

    mapping_arg = DeclareLaunchArgument('mapping', default_value='false')
    navigation_arg = DeclareLaunchArgument('navigation', default_value='false')
    enable_peds_arg = DeclareLaunchArgument('enable_peds', default_value='true')
    social_navigation_arg = DeclareLaunchArgument('social_navigation', default_value='false')
    planner_variant_arg = DeclareLaunchArgument('planner_variant', default_value='navfn')
    enable_gz_ped_debug_arg = DeclareLaunchArgument('enable_gz_ped_debug', default_value='false')
    enable_legacy_actors_arg = DeclareLaunchArgument('enable_legacy_actors', default_value='false')
    enable_rviz_arg = DeclareLaunchArgument('enable_rviz', default_value='true')
    gz_gui_arg = DeclareLaunchArgument('gz_gui', default_value='true')
    map_arg = DeclareLaunchArgument('map', default_value='')
    map_file_arg = DeclareLaunchArgument('map_file', default_value=default_map)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    model_arg = DeclareLaunchArgument('model', default_value='burger')
    use_composition_arg = DeclareLaunchArgument('use_composition', default_value='False')
    nav2_log_level_arg = DeclareLaunchArgument('nav2_log_level', default_value='info')

    mapping = LaunchConfiguration('mapping')
    navigation = LaunchConfiguration('navigation')
    enable_peds = LaunchConfiguration('enable_peds')
    social_navigation = LaunchConfiguration('social_navigation')
    enable_gz_ped_debug = LaunchConfiguration('enable_gz_ped_debug')
    enable_legacy_actors = LaunchConfiguration('enable_legacy_actors')
    enable_rviz = LaunchConfiguration('enable_rviz')
    gz_gui = LaunchConfiguration('gz_gui')
    nav2_params_resolved = LaunchConfiguration('nav2_params_resolved')
    map_file_resolved = LaunchConfiguration('map_file_resolved')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    use_composition = LaunchConfiguration('use_composition')
    nav2_log_level = LaunchConfiguration('nav2_log_level')

    resolve_map_file = OpaqueFunction(function=_resolve_map_file, args=[pkg_share])
    select_nav2_params = OpaqueFunction(
        function=_select_nav2_params,
        args=[
            nav2_dynamic_params,
            nav2_dynamic_astar_params,
            nav2_dynamic_dstar_params,
            nav2_social_params,
            nav2_social_astar_params,
            nav2_social_dstar_params,
            nav2_social_smac_params,
            nav2_dstar_plus_params,
        ],
    )
    social_condition = IfCondition(PythonExpression([
        "'", navigation, "'.lower() == 'true' and '",
        enable_peds, "'.lower() == 'true' and '",
        social_navigation, "'.lower() == 'true'",
    ]))
    # Condition for pedestrian pose publishing: whenever peds are enabled
    peds_condition = IfCondition(PythonExpression([
        "'", mapping, "'.lower() == 'false' and '",
        enable_peds, "'.lower() == 'true'",
    ]))
    gz_ped_debug_condition = IfCondition(PythonExpression([
        "'", mapping, "'.lower() == 'false' and '",
        enable_peds, "'.lower() == 'true' and '",
        enable_gz_ped_debug, "'.lower() == 'true'",
    ]))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': TextSubstitution(text=f'-r -v 4 {world_file}')
        }.items(),
        condition=IfCondition(gz_gui),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': TextSubstitution(text=f'-r -s -v 4 {world_file}')
        }.items(),
        condition=UnlessCondition(gz_gui),
    )

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_share, 'models'),
            os.path.join(pkg_share, 'models', 'cafe', 'materials', 'textures'),
            os.path.join(pkg_share, 'models', 'cafe_table', 'materials', 'textures'),
            os.path.join(pkg_share, 'models', 'person_walking', 'materials', 'textures'),
            os.path.join(pkg_share, 'models', 'person_standing', 'materials', 'textures'),
            os.path.join(tb3_share, 'models'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    plugin_path_value = ':'.join([
        os.path.join(prefix_path, 'opt', 'gz_sim_vendor', 'lib', 'gz-sim-8', 'plugins'),
        os.path.join(prefix_path, 'opt', 'gz_sim_vendor', 'lib'),
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    ])

    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_path_value,
    )

    set_ign_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=':'.join([
            plugin_path_value,
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
        ]),
    )

    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=model,
    )

    # VM/headless runs can crash Ogre2 before odom/tf are published. Force
    # software GL and the classic Ogre renderer so batch runs survive reboot.
    set_libgl_software = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1',
    )

    set_mesa_gl_version = SetEnvironmentVariable(
        name='MESA_GL_VERSION_OVERRIDE',
        value='3.3',
    )

    set_mesa_glsl_version = SetEnvironmentVariable(
        name='MESA_GLSL_VERSION_OVERRIDE',
        value='330',
    )

    set_gz_render_engine = SetEnvironmentVariable(
        name='GZ_RENDER_ENGINE',
        value='ogre',
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_share, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    spawn_table = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cafe_table',
            '-file', table_model,
        ],
        output='screen',
    )

    spawn_actor_condition = IfCondition(PythonExpression([
        "'", mapping, "' == 'false' and '",
        enable_peds, "' == 'true' and '",
        social_navigation, "' == 'false' and '",
        enable_legacy_actors, "' == 'true'",
    ]))

    spawn_actor_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'person_walking_actor_1',
            '-file', os.path.join(pkg_share, 'models', 'person_walking_actor_1', 'model.sdf'),
        ],
        output='screen',
        condition=spawn_actor_condition,
    )

    spawn_actor_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'person_walking_actor_2',
            '-file', os.path.join(pkg_share, 'models', 'person_walking_actor_2', 'model.sdf'),
        ],
        output='screen',
        condition=spawn_actor_condition,
    )

    spawn_actor_3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'person_walking_actor_3',
            '-file', os.path.join(pkg_share, 'models', 'person_walking_actor_3', 'model.sdf'),
        ],
        output='screen',
        condition=spawn_actor_condition,
    )

    spawn_standing = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'person_standing',
            '-file', os.path.join(pkg_share, 'models', 'person_standing', 'model.sdf'),
            '-x', '0.0',
            '-y', '6.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
        condition=spawn_actor_condition,
    )

    bridge_tb3 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': tb3_bridge}],
        output='screen',
    )

    bridge_social_poses = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_social_pose_bridge',
        parameters=[{'config_file': social_pose_bridge}],
        output='screen',
        condition=gz_ped_debug_condition,
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        condition=IfCondition(mapping),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        condition=IfCondition(navigation),
        launch_arguments={
            'slam': 'False',
            'use_localization': 'True',
            'map': map_file_resolved,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': nav2_params_resolved,
            'use_composition': use_composition,
            'log_level': nav2_log_level,
        }.items(),
    )

    ped_pose_extractor = Node(
        package='cpe631_ros2',
        executable='ped_pose_extractor',
        name='ped_pose_extractor',
        output='screen',
        condition=gz_ped_debug_condition,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'input_topic': '/gz/model_poses',
                # Keep this bridge/extractor available for debugging, but do not
                # publish to the planner input. cpe631_peds is the single
                # authoritative /pedestrian_poses source for the experiments.
                'output_topic': '/pedestrian_poses_gz_debug',
                'start_index': 1,
                'pedestrian_count': 4,
            }
        ],
    )

    # Pedestrian manager: spawns model-based pedestrians and directly publishes
    # their poses to /pedestrian_poses, bypassing the gz bridge chain.
    peds_manager = Node(
        package='cpe631_ros2',
        executable='cpe631_peds',
        name='pedestrian_manager',
        output='screen',
        condition=peds_condition,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'output_topic': '/pedestrian_poses',
                'update_period': 0.2,
            }
        ],
    )

    social_nav_node = Node(
        package='cpe631_ros2',
        executable='social_nav_node',
        name='social_nav_node',
        output='screen',
        condition=social_condition,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'prediction_dt': 0.5,
                'q_scale': 0.08,
                'lateral_q_scale': 0.02,
                'gamma': 0.72,
                'publish_rate': 5.0,
                'frame_id': 'map',
                'input_topic': '/pedestrian_poses',
                'output_topic': '/social_costmap',
                'velocity_alpha': 0.35,
                'velocity_tau': 0.35,
                'track_timeout': 1.0,
                'grid_resolution': 0.05,
                'grid_width': 9.45,
                'grid_height': 22.35,
                'grid_origin_x': -5.999,
                'grid_origin_y': -11.076,
                'peak_cost': 85,
                'cutoff_sigma': 2.5,
                'longitudinal_sigma': 0.80,
                'lateral_sigma': 0.35,
                'speed_sigma_scale': 0.35,
                'group_cost_ratio': 0.45,
                'base_lookahead': 1.5,
                'speed_lookahead_gain': 0.7,
                'min_lookahead': 1.5,
                'max_lookahead': 2.8,
                'behind_ped_weight': 0.15,
                'social_relevance_distance': 3.5,
                'social_relevance_soft_margin': 2.0,
                'min_relevance_weight': 0.10,
                'instant_cost_ratio': 0.85,
                'instant_sigma': 0.75,
                'approach_cost_gain': 0.3,
                'lethal_core_radius': 0.25,
                'use_ped_orientation': True,
                'ped_orientation_offset_deg': -90.0,
                'robot_traj_hypotheses': 3,
                'robot_traj_yaw_rate_span': 0.8,
                'robot_prediction_min_speed': 0.12,
                'velocity_uncertainty_gain': 0.35,
                'max_uncertainty_scale': 1.6,
            }
        ],
    )

    dstar_plus_condition = IfCondition(PythonExpression([
        "'", navigation, "'.lower() == 'true' and '",
        LaunchConfiguration('planner_variant'), "'.lower() == 'dstarplus'",
    ]))

    replan_trigger_node = Node(
        package='cpe631_ros2',
        executable='replan_trigger',
        name='replan_trigger',
        output='screen',
        condition=dstar_plus_condition,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_republisher = Node(
        package='cpe631_ros2',
        executable='cpe631_map_republisher',
        name='map_republisher',
        output='screen',
        condition=IfCondition(navigation),
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'input_map_topic': '/map',
                'output_map_topic': '/map_viz',
                'input_metadata_topic': '/map_metadata',
                'output_metadata_topic': '/map_metadata_viz',
                'republish_hz': 1.0,
            }
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        remappings=[
            ('/map', '/map_viz'),
            ('/map_metadata', '/map_metadata_viz'),
        ],
        output='screen',
        condition=IfCondition(enable_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        mapping_arg,
        navigation_arg,
        enable_peds_arg,
        social_navigation_arg,
        planner_variant_arg,
        enable_gz_ped_debug_arg,
        enable_legacy_actors_arg,
        enable_rviz_arg,
        gz_gui_arg,
        map_arg,
        map_file_arg,
        use_sim_time_arg,
        model_arg,
        use_composition_arg,
        nav2_log_level_arg,
        resolve_map_file,
        select_nav2_params,
        set_resource_path,
        set_plugin_path,
        set_ign_plugin_path,
        set_tb3_model,
        set_libgl_software,
        set_mesa_gl_version,
        set_mesa_glsl_version,
        set_gz_render_engine,
        gz_sim,
        gz_sim_headless,
        robot_state_publisher,
        spawn_table,
        spawn_actor_1,
        spawn_actor_2,
        spawn_actor_3,
        spawn_standing,
        bridge_tb3,
        bridge_social_poses,
        slam_toolbox,
        nav2_bringup,
        ped_pose_extractor,
        peds_manager,
        social_nav_node,
        replan_trigger_node,
        map_republisher,
        rviz,
    ])
