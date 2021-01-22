*** Settings ***
Documentation     A test test suite
...
...               This test suite is for testing
Library           RobotRosGazeboLibrary.Keywords
Library           Process
Suite Teardown    Terminate All Processes    kill=True

*** Variables ***


*** Test Cases ***
Test Handover
    ${handle1} =    Start Process    roslaunch    xivt_robotics_demo    fokus_cube.launch    shell=True
    Sleep    7
    ${handle2} =    Start Process    roslaunch    rosbridge_server    rosbridge_websocket.launch    shell=True
    Sleep    2
	Connect on Port 9090
	Unpause
	Sleep      1
    Get is_static of model object
    Get model-state of object
	Get model-info of object
	Get link-state of robot1::base_link
	Get link-properties of robot1::base_link
	Get physics-properties

	Publish "go" on /user_input
	#Wait for 1
	Sleep		2
    terminate process  ${handle1}
    terminate process  ${handle2}
    [Teardown]    Disconnect from ROS
