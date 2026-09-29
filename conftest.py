from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
HSL_CORE_PATH = ROOT / "hsl_core"
if str(HSL_CORE_PATH) not in sys.path:
    sys.path.insert(0, str(HSL_CORE_PATH))
