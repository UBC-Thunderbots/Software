import os
import sys
from pathlib import Path

import ruff

package_dir = Path(ruff.__file__).parent

candidates = [
    package_dir.parent.parent / "bin" / "ruff",
    package_dir.resolve().parent.parent / "bin" / "ruff",
]
ruff_bin = None
for path in candidates:
    if path.is_file():
        ruff_bin = path
        break
    
if ruff_bin is None:
    sys.exit("Could not find the ruff binary, looked in: " + ", ".join(map(str, candidates)))

os.execv(ruff_bin, [str(ruff_bin), *sys.argv[1:]])