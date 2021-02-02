import pickle
from .ExtendedSimulationData import ExtendedSimulationData

obj = ExtendedSimulationData()
obj.get_result()
obj.get_rtf()
obj.get_data()

pickle.dump(obj, open("../pickle_obj.p", "wb"))
