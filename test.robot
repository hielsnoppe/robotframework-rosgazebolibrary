*** Settings ***
Documentation     A test test suite
...
...               This test suite is for testing
Library           RobotRosGazeboLibrary.Keywords

*** Test Cases ***
Test Handover
	[Setup]    Connect on port 9090
	Unpause
	Sleep      1
    Get model-state of object
	Get model-info of object
	Get link-state of robot1::base_link
	Get link-properties of robot1::base_link
	Get physics-properties

	Publish "go" on /user_input
	#Wait for 1
	Sleep		1

    [Teardown]    Disconnect from ROS
