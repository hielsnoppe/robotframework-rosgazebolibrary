import logging
import os
import pickle

from .ExtendedSimulationData import ExtendedSimulationData
from .globals import TEST_DATA_FILE_PATH

try:
    logging_lvl = os.environ['XIVT_LOGGING_LVL'].lower()
    if logging_lvl == 'debug':
        logging.basicConfig(level=logging.DEBUG)
    elif logging_lvl == 'info':
        logging.basicConfig(level=logging.INFO)
    else:
        print(f"Could not match {logging_lvl}.\n"
              f"is it info or debug?")
        logging.basicConfig(level=logging.WARNING)
except KeyError:
    logging.basicConfig(level=logging.WARNING)
    logging.warning("XIVT_LOGGING_LVL is not set, setting logging level to WARNING.")


obj = ExtendedSimulationData()
obj.get_result()
obj.get_data()
obj.get_rtf()

pickle.dump(obj, open(TEST_DATA_FILE_PATH, "wb"))
