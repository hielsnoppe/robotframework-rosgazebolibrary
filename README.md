# Robot ROS Gazebo Library

## Installation

For detailed instructions, see [Installation instructions](https://github.com/robotframework/robotframework/blob/master/INSTALL.rst) for Robot Framework.

In short:

    python3 -m venv .venv
    source ./venv/bin/activate
    
    pip install -r requirements.txt

## Running Tests

Run these commands in separate terminals:

    roslaunch your_package your_example.launch
    roslaunch rosbridge_server rosbridge_websocket.launch
    robot example.robot

## How to use

    Library     RobotROSGazeboLibrary.Keywords

    *** Test Cases ***
    Example Test
        Do This
        Do That