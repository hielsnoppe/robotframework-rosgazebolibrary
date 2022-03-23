# Robot Framework ROS Gazebo Library

[![CodeFactor](https://www.codefactor.io/repository/github/hielsnoppe/robotframework-rosgazebolibrary/badge)](https://www.codefactor.io/repository/github/hielsnoppe/robotframework-rosgazebolibrary)

Keyword library for the [Robot Framework](https://robotframework.org/) for accessing [ROS](https://www.ros.org/) based systems and the [Gazebo simulator](http://gazebosim.org/).

Copyright 2020-2022 [Fraunhofer FOKUS](https://www.fokus.fraunhofer.de/)

The initial version of this software was developed at the [Fraunhofer Institute for Open Communication Systems (FOKUS)](https://www.fokus.fraunhofer.de/).
Initial development was funded by the ITEA3 project [eXcellence in Variant Testing (XIVT)](https://itea4.org/project/xivt.html).

## Installation

For detailed instructions, see [Installation instructions](https://github.com/robotframework/robotframework/blob/master/INSTALL.rst) for Robot Framework.

Download this repository and install the library using `pip install -e`.

Installation will become easier as soon as this library is released to the [Python Package Index](https://pypi.org/).

## Documentation

In principle, the following command should create HTML documentation:

    python -m robot.libdoc RobotRosGazeboLibrary docs/index.html

Unfortunately, this does not work at the moment.
Feel free to create a pull request if you know how to fix this.

## Usage Examples

See `examples` folder for example test cases.

## Running Tests (outdated)

Run these commands in separate terminals:

    roslaunch your_package your_example.launch
    roslaunch rosbridge_server rosbridge_websocket.launch
    robot example.robot