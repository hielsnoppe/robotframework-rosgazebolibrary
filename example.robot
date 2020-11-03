*** Settings ***
Documentation     An example test suite
...
...               This test suite is an example
Library           RobotRosGazeboLibrary.Keywords

*** Test Cases ***
Test Handover
	[Setup]    Connect on port 9090
	Unpause
	Sleep      1
    Verify link robot1::base_link at 0 0 0
	Verify model object at 0.3 0 0
	Publish "go" on /user_input
	#Wait for 45
	Sleep		45
	Verify model object at 0.3 0.63 0
    [Teardown]    Disconnect from ROS
