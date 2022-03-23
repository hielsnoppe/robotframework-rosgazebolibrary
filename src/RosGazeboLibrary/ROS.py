from robot.api.deco import keyword
from robot.libraries.BuiltIn import BuiltIn

import roslibpy
from roslibpy import Service, ServiceRequest

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

    @keyword("roslaunch")
    def roslaunch(self, package, executable):
        """
        Launch a launch file from a specified ROS package using `roslaunch`
        
        Uses robot.libraries.Process

        See also https://wiki.ros.org/roslaunch/API%20Usage
        """

        return self.process_lib.start_process('roslaunch',
                package, executable,
                stdout='stdout.txt', stderr='stderr.txt')

    @keyword("rosrun")
    def rosrun(self, package, executable, *args):
        """
        Run an executable from a specified ROS package using `rosrun`

        Uses robot.libraries.Process
        """

        return self.process_lib.start_process('rosrun',
                package, executable, *args,
                shell=True)

    @keyword("Call Service")
    def call_service(self, name, message='std_srvs/Empty', data=None):
        return Service(self.client, name, message).call(ServiceRequest(data))


    '''
    rosbridge
    http://wiki.ros.org/rosbridge_suite
    https://stackoverflow.com/questions/54199557/using-rosbridge-between-two-processes-on-the-same-computer
    '''

    @keyword("Start rosbridge")
    def start_rosbridge(self):
        return self.start_rosbridge(9090)

    @keyword("Start rosbridge on ${port}")
    def start_rosbridge(self, port=9090):

        return self.process_lib.start_process('roslaunch',
                'rosbridge_server',
                'rosbridge_websocket.launch',
                'port:=%s' % port
                )

    @keyword
    def rosbridge_connect(self, host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)
        self.client.run()

    @keyword
    def rosbridge_close(self):
        self.client.close()