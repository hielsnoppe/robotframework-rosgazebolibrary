from typing import Optional, List, Dict


class SimulationData(object):
    """
    data sfja sśdofbjs jokkjaopkkm
    vdvoir
    """
    def __init__(self):
        self.result: Optional[Dict[str:Dict[str: int, str: int], str:Dict[str: int, str: int]]] = {}
        self.rtf: Optional[List[Dict[str: float, str: float, str: float, str: bool]]] = []
        self.model_data: Optional[Dict[str:List[Dict]]] = {}
        self.link_data: Optional[Dict[str:List[Dict]]] = {}
        self.world_data: Optional[List[Dict]] = []
        self.physics_data: Optional[List[Dict]] = []
        self.joint_data: Optional[Dict[str:List[Dict]]] = {}
        self.light_data: Optional[Dict[str:List[Dict]]] = {}
