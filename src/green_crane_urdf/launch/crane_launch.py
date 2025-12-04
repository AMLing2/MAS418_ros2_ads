from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the URDF file
    urdf_file = 'urdf/green_crane.urdf'
    urdf_path = get_package_share_directory('green_crane_urdf') + '/' + urdf_file
    
    # Define the joint state publisher node
    joint_state_publisher_node = Node(
        package='green_crane_urdf',
        executable='urdf_tutorial_cpp',  
        output='screen'
    )
    
    # Define the robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # Define an RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', get_package_share_directory('green_crane_urdf') + '/rviz/urdf.rviz'],
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
