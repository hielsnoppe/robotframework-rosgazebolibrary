from robot.api.deco import keyword
from robot.libraries.BuiltIn import BuiltIn

class Gazebo(object):
    """Robot Framework test library for the Gazebo simulator

    See also http://gazebosim.org/tutorials/?tut=ros_comm

    == Table of contents ==

    %TOC%
    """

    ROBOT_LIBRARY_SCOPE = 'SUITE'

    def __init__(self):
        self.ros_lib = BuiltIn().get_library_instance('RosGazeboLibrary.ROS')
        self.spawned_models = {}

    # Create and destroy models in simulation
    # http://gazebosim.org/tutorials/?tut=ros_comm#Services:Createanddestroymodelsinsimulation

    @keyword
    def spawn_urdf_model(self, urdf_path: str, position: tuple, model_name: str):
        ''' TODO: Refactor to use service call '''

        self.spawned_models[model_name] = {
            'type': 'URDF',
            'path': urdf_path,
            'position': position
        }

        return self.ros_lib.rosrun('gazebo_ros', 'spawn_model', *[
            '-file', urdf_path,
            '-urdf',
            '-model', model_name,
            '-x', position[0],
            '-y', position[1],
            '-z', position[2],
        ])

    @keyword
    def spawn_sdf_model(self, sdf_path: str, position: tuple, model_name: str):
        ''' TODO: Refactor to use service call '''

        self.spawned_models[model_name] = {
            'type': 'SDF',
            'path': sdf_path,
            'position': position
        }

        return self.ros_lib.rosrun('gazebo_ros', 'spawn_model', *[
            '-file', sdf_path,
            '-sdf',
            '-model', model_name,
            '-x', position[0],
            '-y', position[1],
            '-z', position[2],
        ])

    @keyword
    def delete_model(self, model_name: str):
        ''' Delete a model from simulation
        http://gazebosim.org/tutorials/?tut=ros_comm#DeleteModel
        '''

        self.spawned_models.pop(model_name)

        return self.ros_lib.rosservice_call(
            'gazebo/delete_model', 'gazebo_msgs/DeleteModel',
            { 'model_name': model_name }
            )

    @keyword
    def delete_spawned_models(self):
        ''' Delete all previously spawned models from simulation
        '''
        for model_name in list(self.spawned_models.keys()):
            self.delete_model(model_name)

    # State and property setters
    # http://gazebosim.org/tutorials/?tut=ros_comm#Services:Stateandpropertysetters

    ''' TODO
    def set_link_properties(self, ...):
    def set_physics_properties(self, ...):
    def set_model_state(self, ...):
    def set_model_configuration(self, ...):
    def set_joint_properties(self, ...):
    def set_link_state(self, ...):
    '''

    # State and property getters
    # http://gazebosim.org/tutorials/?tut=ros_comm#Services:Stateandpropertygetters

    @keyword
    def get_model_properties(self, model_name: str):
        '''This service returns the properties of a model in simulation.'''
        return self.ros_lib.rosservice_call(
            'gazebo/get_model_properties', 'gazebo_msgs/GetModelProperties',
            { 'model_name': model_name }
            )

    @keyword
    def get_model_state(self, model_name: str):
        '''This service returns the states of a model in simulation.'''
        return self.ros_lib.rosservice_call(
            'gazebo/get_model_state', 'gazebo_msgs/GetModelState',
            { 'model_name': model_name }
            )

    @keyword
    def get_world_properties(self):
        '''This service returns the properties of the simulation world.'''
        return self.ros_lib.rosservice_call(
            'gazebo/get_model_state', 'gazebo_msgs/GetWorldProperties'
            )
    
    ''' TODO
    def get_joint_properties(self, ...):
    def get_link_properties(self, ...):
    def get_link_state(self, ...):
    '''

    @keyword
    def get_physics_properties(self):
        '''This service returns the properties of the physics engine used in simulation..'''
        return self.ros_lib.rosservice_call(
            'gazebo/get_physics_properties', 'gazebo_msgs/GetPhysicsProperties'
            )

    ''' TODO
    def link_states(self, ...): # investigate
    def model_states(self, ...): # investigate
    '''

    # Force control
    # http://gazebosim.org/tutorials/?tut=ros_comm#Services:Forcecontrol

    ''' TODO
    /gazebo/apply_body_wrench
    /gazebo/apply_joint_effort
    /gazebo/clear_body_wrenches
    /gazebo/clear_joint_forces
    '''

    # Simulation control
    # http://gazebosim.org/tutorials/?tut=ros_comm#Services:Simulationcontrol

    @keyword
    def reset_simulation(self):
        return self.ros_lib.call_service('/gazebo/reset_simulation')

    @keyword
    def reset_world(self):
        return self.ros_lib.call_service('/gazebo/reset_world')

    @keyword
    def pause_physics(self):
        return self.ros_lib.call_service('/gazebo/pause_physics')

    @keyword
    def unpause_physics(self):
        return self.ros_lib.call_service('/gazebo/unpause_physics')

    # Undocumented services
    # Found via `rosservice list`

    '''
    /gazebo/delete_light
    /gazebo/get_light_properties
    /gazebo/get_loggers
    /gazebo/set_light_properties
    /gazebo/set_logger_level
    /gazebo/set_parameters
    /gazebo_gui/get_loggers
    /gazebo_gui/set_logger_level
    '''
    
    # Convenience keywords

    @keyword
    def launch_empty_world(self):
        return self.ros_lib.roslaunch('gazebo_ros', 'empty_world.launch')