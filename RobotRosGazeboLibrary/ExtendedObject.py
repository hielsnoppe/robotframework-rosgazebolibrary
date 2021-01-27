from BaseObject import BaseObject
import xml.dom.minidom
import ast
import csv
from globals import KW_LIB


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


class ExtendedObject(BaseObject):
    def __init__(self):
        super().__init__()

    def get_model_data(self):
        work_list = xml_preprocessor('model')
        self.model_data = {}
        for item in work_list:
            data_dict = ast.literal_eval(item[0].firstChild.data)
            data_list = data_dict['data']
            tree_dict = data_list[-1]
            data_list.remove(data_list[-1])
            for key in reversed(data_list):
                tree_dict = {key: tree_dict}
            if data_dict['name'] not in self.model_data:
                self.model_data[data_dict['name']] = []
            self.model_data[data_dict['name']].append(tree_dict)

    def get_link_data(self):
        work_list = xml_preprocessor('link')
        self.link_data = {}
        for item in work_list:
            data_dict = ast.literal_eval(item[0].firstChild.data)
            data_list = data_dict['data']
            tree_dict = data_list[-1]
            data_list.remove(data_list[-1])
            for key in reversed(data_list):
                tree_dict = {key: tree_dict}
            if data_dict['name'] not in self.link_data:
                self.link_data[data_dict['name']] = []
            self.link_data[data_dict['name']].append(tree_dict)

    def get_world_data(self):
        work_list = xml_preprocessor('world')
        self.world_data = []
        for item in work_list:
            data_dict = ast.literal_eval(item[0].firstChild.data)
            data_list = data_dict['data']
            tree_dict = data_list[-1]
            data_list.remove(data_list[-1])
            for key in reversed(data_list):
                tree_dict = {key: tree_dict}
            self.world_data.append(tree_dict)

    def get_physics_data(self):
        work_list = xml_preprocessor('physics')
        self.physics_data = []
        for item in work_list:
            data_dict = ast.literal_eval(item[0].firstChild.data)
            data_list = data_dict['data']
            tree_dict = data_list[-1]
            data_list.remove(data_list[-1])
            for key in reversed(data_list):
                tree_dict = {key: tree_dict}
            self.physics_data.append(tree_dict)

    def get_joint_data(self):
        work_list = xml_preprocessor('joint')
        self.joint_data = {}
        for item in work_list:
            data_dict = ast.literal_eval(item[0].firstChild.data)
            data_list = data_dict['data']
            tree_dict = data_list[-1]
            data_list.remove(data_list[-1])
            for key in reversed(data_list):
                tree_dict = {key: tree_dict}
            if data_dict['name'] not in self.joint_data:
                self.joint_data[data_dict['name']] = []
            self.joint_data[data_dict['name']].append(tree_dict)#

    def get_light_data(self):
        work_list = xml_preprocessor('light')
        self.light_data = {}
        for item in work_list:
            data_dict = ast.literal_eval(item[0].firstChild.data)
            data_list = data_dict['data']
            tree_dict = data_list[-1]
            data_list.remove(data_list[-1])
            for key in reversed(data_list):
                tree_dict = {key: tree_dict}
            if data_dict['name'] not in self.light_data:
                self.light_data[data_dict['name']] = []
            self.light_data[data_dict['name']].append(tree_dict)

    def get_rtf(self):
        self.rtf = []
        with open("/home/lerner/Semesterprojekt/robottests/robot-ros-gazebo-library/RTF.log") as csv_file:
            reader = csv.reader(csv_file, delimiter=',', quotechar='|')
            reader.__next__()
            for row in reader:
                for i in range(0, 3):
                    row[i] = float(row[i])
                row[3] = row[3].lstrip()
                self.rtf.append(row)
