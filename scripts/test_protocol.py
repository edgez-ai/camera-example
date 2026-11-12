#!/usr/bin/env python3
"""Run the actual shared C image/protocol code with host sanitizers."""
from pathlib import Path
import os
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="camera-protocol-") as directory:
    binary=Path(directory)/"protocol-test"
    subprocess.run([os.environ.get("CC","cc"),"-std=c11","-Wall","-Wextra","-Werror",
        "-fsanitize=address,undefined","-g","-I",str(root/"common"),
        str(root/"tests/protocol_test.c"),str(root/"common/camera_protocol.c"),
        str(root/"nrf54/src/camera_dummy_images.c"),"-o",str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
