from pathlib import Path
import sys


_THIS_DIR = Path(__file__).resolve().parent
_PACKAGE_ROOT = _THIS_DIR.parent

if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))
