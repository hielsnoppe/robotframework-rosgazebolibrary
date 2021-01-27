from typing import Optional, List, Dict


class BaseObject(object):
    """
    data sfja sśdofbjs jokkjaopkkm
    vdvoir
    """
    def __init__(self):
        self.data = None
        self.rtf: Optional[List[List[float, float, float, str]]] = None
        self.model_data: Optional[Dict[str:List[Dict]]] = None
        self.link_data: Optional[Dict[str:List[Dict]]] = None
        self.world_data: Optional[List[Dict]] = None
        self.physics_data: Optional[List[Dict]] = None
        self.joint_data: Optional[Dict[str:List[Dict]]] = None
        self.light_data: Optional[Dict[str:List[Dict]]] = None
