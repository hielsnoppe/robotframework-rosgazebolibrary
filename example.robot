*** Settings ***
Documentation     An example test suite
...
...               This test suite is an example
Library           RobotRosGazeboLibrary.Keywords
Library           Process
Suite Teardown    Terminate All Processes    kill=True

*** Test Cases ***
Test Handover
	${gazebo_process} =    Start Process    roslaunch    xivt_robotics_demo    fokus_cube.launch        shell=True
    Sleep    7
    ${rosbridge_process} =    Start Process    roslaunch    rosbridge_server    rosbridge_websocket.launch    shell=True
    Sleep    2
	Connect on Port 9090
	Unpause
	Sleep      1
    Verify link robot1::base_link at 0 0 0
	Verify model object at 0.3 0 0
	Get model-info of object
	Publish "go" on /user_input
	Sleep    10
	Verify model object at 0.3 0.63 0
	Sleep       1
	terminate process    ${gazebo_process}
	terminate process    ${rosbridge_process}
    [Teardown]    Disconnect from ROS
