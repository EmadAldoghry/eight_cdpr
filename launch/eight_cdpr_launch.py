from ament_index_python.packages       import get_package_share_path

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions                import Node
from launch.substitutions              import Command, LaunchConfiguration
from launch.actions                    import DeclareLaunchArgument, ExecuteProcess
from launch                            import LaunchDescription

import os


# The following function is mandatory
def generate_launch_description():

    # Defining the object, which must be returned
    ld = LaunchDescription()

    package_path = get_package_share_path('eight_cdpr')

    urdf_model_path  = os.path.join(package_path, 'model/eight_cdpr_model.urdf')

    rviz_config_path = os.path.join(package_path, 'rviz/config.rviz')

    #sim_time = LaunchConfiguration('use_sim_time', default = 'false')

    model_arg = DeclareLaunchArgument(name          = 'eight_cdpr_model',
                                      default_value = str(urdf_model_path),
                                      description   = "This is my URDF model definition")

    rviz_arg = DeclareLaunchArgument(name          = 'rvizconfig',
                                     default_value = str(rviz_config_path),
                                     description   = "This is my RViz config file")

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('eight_cdpr_model')]),
                                       value_type = str)

    # Configure the main node
    main_node = Node(package    = "eight_cdpr",
                     executable = "eight_cdpr",
                     output     = "screen",
                     namespace  = None)

    # Configure the robot_state_publisher
    robot_state_node = Node(package    = "robot_state_publisher",
                            executable = "robot_state_publisher",
                            name       = "robot_state_publisher",
                            output     = "screen",
                            parameters = [{'robot_description': robot_description}])

    # Configure the node for the joystick
    joy_node = Node(package    = "joy",
                    executable = "joy_node",
                    output     = "screen",
                    name       = "joy",
                    namespace  = None)

    # Configure RViz for visualization
    rviz_node = Node(package    = "rviz2",
                     executable = "rviz2",
                     output     = "screen",
                     namespace  = None,
                     arguments  = ["-d", LaunchConfiguration("rvizconfig")])


    ld.add_action(model_arg)
    ld.add_action(rviz_arg)
    ld.add_action(main_node)
    ld.add_action(robot_state_node)
    ld.add_action(joy_node)
    ld.add_action(rviz_node)

    return ld
