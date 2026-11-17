import sys
from pathlib import Path

SOFTWARE = Path(__file__).resolve().parents[2]
# xlerobot_table_alignment without installing the plugin, and the robot class for the kinematics parity test.
sys.path[:0] = [str(SOFTWARE / "plugins" / "xlerobot_table_alignment"), str(SOFTWARE / "src")]
