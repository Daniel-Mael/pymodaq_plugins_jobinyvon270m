from pathlib import Path
from pymodaq.utils.daq_utils import set_logger

from .utils import Config
config = Config()

with open(str(Path(__file__).parent.joinpath('resources/VERSION')), 'r') as fvers:
    __version__ = fvers.read().strip()
