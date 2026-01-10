from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Helper: choose a namespace that doesn't collide with existing topics
def _unique_ns(context):
    import rclpy
    from rclpy.node import Node as RclpyNode

    base_ns = LaunchConfiguration("ns").perform(context).strip("/")
    base_ns = base_ns or "oak"

    # Topics this node will publish (relative to ns)
    candidate_rel_topics = [
        "rgb/image",
        "left/image",
        "right/image",
    ]

    rclpy.init(args=None)
    probe = RclpyNode("depthai_ns_probe", namespace="/")
    try:
        existing = {name for name, _ in probe.get_topic_names_and_types()}

        # Try base, then base_2, base_3, ...
        chosen = base_ns
        idx = 2
        def any_conflict(ns):
            return any(f"/{ns}/{rel}" in existing for rel in candidate_rel_topics)

        while any_conflict(chosen):
            chosen = f"{base_ns}_{idx}"
            idx += 1

    finally:
        probe.destroy_node()
        rclpy.shutdown()

    # Build and return the actual Node action with the chosen namespace
    device_mxid   = LaunchConfiguration("device_mxid").perform(context)
    rgb_width   = int(LaunchConfiguration("rgb_width").perform(context))
    rgb_height  = int(LaunchConfiguration("rgb_height").perform(context))
    mono_height = int(LaunchConfiguration("mono_height").perform(context))
    fps         = float(LaunchConfiguration("fps").perform(context))

    return [
        Node(
            package="luxonis_cam_pipeline",
            executable="depthai_multi_cam_node",
            name="depthai_multi_cam_node",
            namespace=chosen,                   # <= unique namespace picked here
            output="screen",
            parameters=[{
                "device_mxid": device_mxid,
                "rgb_width": rgb_width,
                "rgb_height": rgb_height,
                "mono_height": mono_height,
                "fps": fps,
            }],
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("device_mxid",  default_value="192.168.2.20",
                              description="OAK PoE IP (e.g., 192.168.1.50). Empty = auto-discover"),
        DeclareLaunchArgument("rgb_width",  default_value="1280"),
        DeclareLaunchArgument("rgb_height", default_value="720"),
        DeclareLaunchArgument("mono_height", default_value="720"),
        DeclareLaunchArgument("fps",        default_value="30.0"),
        DeclareLaunchArgument("ns",         default_value="oak",
                              description="Base namespace to use; suffix added automatically if needed"),

        # Defer Node creation until we’ve picked a free namespace
        OpaqueFunction(function=_unique_ns),
    ])
