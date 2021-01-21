import subprocess
import time

import roslibpy
from robot.api.deco import keyword, not_keyword

from .gazebo import GazeboClient


class Keywords:
    ROBOT_LIBRARY_SCOPE = 'SUITE'
    TOLERANCE = 0.03
    COORDS = ["x", "y", "z"]

    def __init__(self):
        self.proc_list = []
        self.port = -1
        self.client = None
        self.gazebo = None

    @keyword("Launch ${package} ${executable}")
    def launch(self, package, executable):
        """Launch a launch file from a specified ROS package using `roslaunch`"""

        proc = subprocess.Popen(['roslaunch', package, executable])
        self.proc_list.append(proc)
        time.sleep(15)  # adjust as needed

        # https://wiki.ros.org/roslaunch/API%20Usage
        # https://stackoverflow.com/questions/54199557/using-rosbridge-between-two-processes-on-the-same-computer

    @keyword("Run ${package} ${executable}")
    def run(self, package, executable):
        """Run an executable from a specified ROS package using `rosrun`"""

        proc = subprocess.Popen(['rosrun', package, executable])
        self.proc_list.append(proc)

    @keyword("Publish \"${string}\" on ${topic}")
    def publish_string(self, string, topic):

        topic = roslibpy.Topic(self.client, topic, "std_msgs/String")
        topic.publish(roslibpy.Message({'data': string}))

    @keyword(r"Verify link ${link_name} at ${x:-?\d*\.?\d+} ${y:-?\d*\.?\d+} ${z:-?\d*\.?\d+}")
    def verify_link_position(self, link_name, x: float, y: float, z: float):

        self.verify_link_position_relative(link_name, x, y, z, reference_frame="world")

    @keyword(
        r"Verify link ${link_name} at ${x:-?\d*\.?\d+} ${y:-?\d*\.?\d+} ${z:-?\d*\.?\d+} relative to ${reference_frame}"
    )
    def verify_link_position_relative(self, link_name, x: float, y: float, z: float, reference_frame="world"):

        actual = self.gazebo.get_link_state(link_name)['link_state']
        print(actual)
        expected = {"x": x, "y": y, "z": z}

        self._verify_position(actual, expected, self.TOLERANCE)

    @keyword(r"Verify model ${model_name} at ${x:-?\d*\.?\d+} ${y:-?\d*\.?\d+} ${z:-?\d*\.?\d+}")
    def verify_model_position(self, model_name, x: float, y: float, z: float):

        actual = self.gazebo.get_model_state(model_name)
        expected = {"x": x, "y": y, "z": z}

        self._verify_position(actual, expected, self.TOLERANCE)

    @not_keyword
    def _verify_position(self, actual_state_message, expected, tolerance):

        # expected = { "x": x, "y": y, "z": z }
        actual = {axis: actual_state_message["pose"]["position"][axis] for axis in self.COORDS}
        deviation = {axis: abs(expected[axis] - actual[axis]) for axis in self.COORDS}

        if any([d > self.TOLERANCE for d in deviation.values()]):
            raise AssertionError("Position of {} outside the tolerance:\n".format("TODO")
                                 + "Expected:  x={x:.4f}, y={y:.4f}, z={z:.4f} +/-{:.4f}\n".format(tolerance,
                                                                                                   **expected)
                                 + "Actual:    x={x:.4f}, y={y:.4f}, z={z:.4f}\n".format(**actual)
                                 + "Deviation: x={x:.4f}, y={y:.4f}, z={z:.4f}".format(**deviation)
                                 )

    @keyword("Wait for ${dur_in_s}")
    def wait_for(self, dur_in_s):
        wait = True

        result = self.gazebo.get_world_properties()
        curr_time = result['sim_time']

        while wait:
            result = self.gazebo.get_world_properties()
            if abs(result['sim_time'] - curr_time >= float(dur_in_s)):
                wait = False

    @keyword(r"Connect on port ${port:\d+}")
    def connect_on_port(self, port: int):

        self.port = port
        self.client = roslibpy.Ros(host='localhost', port=self.port)
        self.gazebo = GazeboClient(ros_client=self.client)

        self.client.run()

        if not self.client.is_connected:
            raise RuntimeError("Connection failed")

    @keyword("Unpause")
    def unpause(self):

        self.gazebo.unpause_physics()

    @keyword("Disconnect from ROS")
    def disconnect_from_ros(self):

        for proc in self.proc_list:
            proc.kill()

        self.client.terminate()

    # added

    @keyword("Test")
    def test_get_funcs(self):
        actual = self.gazebo.get_physics_properties()
        print(actual)

    @keyword("Get model-state of ${model_name}")
    def get_model_state(self, model_name):
        model_state = self.gazebo.get_model_state(model_name)
        print(model_state)

    @keyword("Get model-properties of ${model_name}")
    def get_model_properties(self, model_name):
        model_property = self.gazebo.get_model_properties(model_name)
        print(model_property)

    @keyword("Get model-info of ${model_name}")
    def get_model_info(self, model_name):
        model_state = self.gazebo.get_model_state(model_name)
        model_property = self.gazebo.get_model_properties(model_name)
        print(model_state)
        print(model_property)

    @keyword("Get ${trait} of model ${model_name}")
    def get_property_of_model(self, trait, model_name):
        model_state = self.gazebo.get_model_state(model_name)
        print('\'' + trait + '\':', model_state[trait])

    @keyword("Get link-state of ${link_name}")
    def get_link_state(self, link_name):
        link_state = self.gazebo.get_link_state(link_name)
        print(link_state)

    @keyword("Get link-properties of ${link_name}")
    def get_link_properties(self, link_name):
        link_property = self.gazebo.get_link_properties(link_name)
        print(link_property)

    @keyword("Get link-info of ${link_name}")
    def get_link_info(self, link_name):
        link_state = self.gazebo.get_link_state(link_name)
        link_property = self.gazebo.get_link_properties(link_name)
        print(link_state)
        print(link_property)

    @keyword("Get world-properties")
    def get_world_properties(self):
        world_property = self.gazebo.get_world_properties()
        print(world_property)

    @keyword("Get physics-properties")
    def test_physics_properties(self):
        physics_property = self.gazebo.get_physics_properties()
        print(physics_property)

    @keyword("Get joint-properties of ${joint_name}")
    def test_joint_properties(self, joint_name):
        joint_property = self.gazebo.get_light_properties(joint_name)
        print(joint_property)

    @keyword("Get light-properties of ${light_name}")
    def test_light_properties(self, light_name):
        light_property = self.gazebo.get_joint_properties(light_name)
        print(light_property)
