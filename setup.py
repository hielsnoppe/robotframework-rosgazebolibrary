from setuptools import setup, find_packages

setup(
    name='robot-ros-gazebo-library',
    version='0.0.1',
    description='',
    author='Niels Hoppe',
    author_email='niels.hoppe@fokus.fraunhofer.de',
    packages=find_packages(include=['RobotRosGazeboLibrary', 'RobotRosGazeboLibrary.*']),
    install_requires=[
        "robotframework==3.2.2",
        "roslibpy==1.1.0"
    ]
)