import pickle
from ExtendedObject import ExtendedObject

obj = ExtendedObject()
obj.get_rtf()
obj.get_model_data()
obj.get_link_data()
obj.get_world_data()
obj.get_physics_data()
obj.get_joint_data()
obj.get_light_data()

pickle.dump(obj, open("../pickle_obj.p", "wb"))
