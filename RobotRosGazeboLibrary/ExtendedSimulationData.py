from .SimulationData import SimulationData
import xml.dom.minidom
import ast
import csv
from .globals import KW_LIB


def xml_preprocessor(about: str) -> list:
    all_data = xml.dom.minidom.parse("../output.xml")
    data_kws = all_data.getElementsByTagName("kw")
    data_kws = [kw for kw in data_kws if kw.getAttribute('library') == KW_LIB]
    list_msgs = []
    for kw in data_kws:
        full_kw = kw.getElementsByTagName("msg")
        msg_kw = []
        for msg in full_kw:
            if msg.getAttribute("level") == "INFO":
                msg_kw.append(msg)
        try:
            if ast.literal_eval(msg_kw[0].firstChild.data)['about'] == about:
                list_msgs.append(msg_kw)
        except (SyntaxError, IndexError, KeyError):
            pass
    return list_msgs


class ExtendedSimulationData(SimulationData):
    def __init__(self):
        super().__init__()

    def get_data(self):
        output_data = {'model': self.model_data,
                       'link': self.link_data,
                       'world': self.world_data,
                       'physics': self.physics_data,
                       'joint': self.joint_data,
                       'light': self.light_data,
                       }
        for category in output_data:
            work_list = xml_preprocessor(category)
            for item in work_list:
                data_dict = ast.literal_eval(item[0].firstChild.data)
                data_list = data_dict['data']
                tree_dict = data_list[-1]
                data_list.remove(data_list[-1])
                for key in reversed(data_list):
                    tree_dict = {key: tree_dict}
                if category not in ['world', 'physics']:
                    if data_dict['name'] not in output_data[category]:
                        output_data[category][data_dict['name']] = []
                    output_data[category][data_dict['name']].append(tree_dict)
                else:
                    output_data[category].append(tree_dict)

    def get_rtf(self):
        with open("/home/lerner/Semesterprojekt/robottests/robot-ros-gazebo-library/RTF.log") as csv_file:
            reader = csv.reader(csv_file, delimiter=',', quotechar='|')
            reader.__next__()
            for row in reader:
                result_dict = {
                    'rtf': float(row[0]),
                    'sim': float(row[1]),
                    'real': float(row[2]),
                    'paused': True if row[3].lstrip() == 'T' else False
                }
                self.rtf.append(result_dict)
