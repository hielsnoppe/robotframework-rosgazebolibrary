from robot.api.deco import keyword
from robot.libraries.BuiltIn import BuiltIn

class Gazebo(object):
    """Robot Framework test library for the Gazebo simulator

    == Table of contents ==

    %TOC%
    """

    ROBOT_LIBRARY_SCOPE = 'SUITE'

    def __init__(self):
        self.ros_lib = BuiltIn().get_library_instance('RosGazeboLibrary.ROS')

    @keyword
    def reset_simulation(self):
        return self.ros_lib.call_service('/gazebo/reset_simulation')

    @keyword
    def reset_world(self):
        return self.ros_lib.call_service('/gazebo/reset_world')

    # Models

    @keyword
    def spawn_sdf_model(self, sdf_path: str, position: tuple, name: str):

        return self.ros_lib.rosrun('gazebo_ros', 'spawn_model', *[
            '-file', sdf_path,
            '-sdf',
            '-model', name,
            '-x', position[0],
            '-y', position[1],
            '-z', position[2],
        ])

    @keyword
    def get_model_state(self, model_name):
        return self.ros_lib.call_service(
            'gazebo/get_model_state', 'gazebo_msgs/GetModelState',
            { 'model_name': model_name }
            )

    # Convenience keywords

    @keyword
    def launch_empty_world(self):
        return self.ros_lib.roslaunch('gazebo_ros', 'empty_world.launch')