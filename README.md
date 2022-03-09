# Robot ROS Gazebo Library

Keyword library for the [Robot Framework](https://robotframework.org/) for accessing [ROS](https://www.ros.org/) based systems and the [Gazebo simulator](http://gazebosim.org/).

Copyright 2020-2022 [Fraunhofer FOKUS](https://www.fokus.fraunhofer.de/)

The initial version of this software was developed at the [Fraunhofer Institute for Open Communication Systems (FOKUS)](https://www.fokus.fraunhofer.de/).
Initial development was funded by the ITEA3 project [eXcellence in Variant Testing (XIVT)](https://itea4.org/project/xivt.html).

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