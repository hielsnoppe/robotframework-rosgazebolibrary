*** Settings ***
Documentation     A test test suite
...
...               This test suite is for testing
Library           RobotRosGazeboLibrary.Keywords
Library           Process
Suite Teardown    Terminate All Processes    kill=True

*** Keyword ***
Let the fun begin
    Start Process    roslaunch    xivt_robotics_demo    fokus_cube.launch    shell=True
    Sleep    7
    Start Process    roslaunch    rosbridge_server    rosbridge_websocket.launch    shell=True
    Sleep    2
	Connect on Port 9090

Let the fun stop
    Disconnect from ROS
    Terminate All Processes    kill=True


*** Test Cases ***
Test Handover
    ${handle1} =    Start Process    roslaunch    xivt_robotics_demo    fokus_cube.launch    shell=True
    Sleep    7
    ${handle2} =    Start Process    roslaunch    rosbridge_server    rosbridge_websocket.launch    shell=True
    Sleep    2
	Connect on Port 9090
	Read RTF
	Sleep      1
	Unpause
	Sleep      1
    Get pose:position:x of model object
    Get pose of model object
    Get model-state of object
	Get model-info of object
	Get link-state of robot1::base_link
	Get link-properties of robot1::base_link
	Get physics-properties
	Get world-properties

	Publish "go" on /user_input
	Sleep		2
	terminate process    ${handle1}
	terminate process    ${handle2}
    [Teardown]    Let the fun stop
