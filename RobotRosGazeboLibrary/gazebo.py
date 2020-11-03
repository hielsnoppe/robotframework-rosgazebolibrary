from roslibpy import Service, ServiceRequest

class GazeboClient:

    def __init__(self, *args, **kwargs):

        self.client = kwargs.get("ros_client")

    def get_model_state(self, model_name):

        service = Service(self.client,
                'gazebo/get_model_state',
                'gazebo_msgs/GetModelState'
                )
        request = ServiceRequest({ 'model_name': model_name })

        return service.call(request)

    def get_link_state(self, link_name):

        service = Service(self.client,
                '/gazebo/get_link_state',
                'gazebo_msgs/LinkState'
                )
        request = ServiceRequest({ 'link_name': link_name })

        return service.call(request)

    def get_world_properties(self):

        service = Service(self.client,
                '/gazebo/get_world_properties',
                'gazebo_msgs/GetWorldProperties'
                )
        request = ServiceRequest()

        return service.call(request)

    def unpause_physics(self):

        service = Service(self.client,
                '/gazebo/unpause_physics',
                'std_srvs/Empty'
                )
        request = ServiceRequest()

        return service.call(request)