
import launch
import launch_ros.actions

def generate_launch_description():

    image_display = launch_ros.actions.Node(
        package="image_display",
        executable="image_display_node",
        respawn=True,
    )

    return launch.LaunchDescription([
        image_display,
    ])
