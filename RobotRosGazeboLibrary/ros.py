from robot.api.deco import keyword
from robot.libraries import BuiltIn

class ROS(object):
    """Robot Framework test library for the Robot Operating System (ROS)

    This library utilizes Robot Framework's
    [https://robotframework.org/robotframework/latest/libraries/Process.html|Process]
    library for running processes.

    == Table of contents ==

    %TOC%
    """

    ROBOT_LIBRARY_SCOPE = 'SUITE'

    def __init__(self):
        self.process_lib = BuiltIn().get_library_instance('Process')

    @keyword("roslaunch ${package} ${executable}")
    def roslaunch(self, package, executable):
        """
        Launch a launch file from a specified ROS package using `roslaunch`
        
        Uses robot.libraries.Process
        """

        return self.process_lib.start_process('roslaunch', [package, executable])

    @keyword("rosrun ${package} ${executable}")
    def rosrun(self, package, executable):
        """Run an executable from a specified ROS package using `rosrun`"""

        return self.process_lib.start_process('rosrun', [package, executable])

    '''
    @keyword("Publish \"${string}\" on ${topic}")
    def publish_string(self, string, topic):

        topic = roslibpy.Topic(self.client, topic, "std_msgs/String")
        topic.publish(roslibpy.Message({'data': string}))
    '''