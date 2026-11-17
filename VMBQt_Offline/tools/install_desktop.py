#!/usr/bin/env python3
"""Optional user-local desktop shortcut. Run with the project .venv Python."""
from pathlib import Path
import os

root = Path(__file__).resolve().parent.parent
launcher = root / "run.sh"
destination = Path(os.environ.get("XDG_DATA_HOME", Path.home()/".local/share"))/"applications"
destination.mkdir(parents=True, exist_ok=True)
# Desktop Exec supports double-quoted paths; reject newlines/control characters.
if any(c in str(launcher) for c in ('\n','\r','"','`','$','\\')):
    raise SystemExit("Move the project to a simple directory name before installing the shortcut.")
text = f'''[Desktop Entry]
Type=Application
Name=VMBQt Offline Map Editor
Comment=Qt6 Lanelet2 map editor
Exec=bash "{launcher}" %f
Terminal=false
Categories=Development;Graphics;
'''
(destination/"vmbqt.desktop").write_text(text, encoding="utf-8")
print(destination/"vmbqt.desktop")
