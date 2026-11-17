#!/usr/bin/env bash
# Installs only into a project virtualenv; system packages require explicit sudo.
set -euo pipefail
vmbqt_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$vmbqt_dir"
if [[ "$(uname -s)" != Linux ]]; then
  printf '%s\n' 'This helper is for Ubuntu Linux. See README_zh.md.'
  exit 1
fi
if [[ "${1:-}" == --system-deps ]]; then
  sudo apt-get update
  sudo apt-get install -y python3 python3-venv python3-pip \
    libegl1 libgl1 libfontconfig1 libxkbcommon-x11-0 libxcb-cursor0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 \
    libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 libxcb-xkb1 \
    libdbus-1-3 fonts-noto-cjk
fi
python3 -c 'import sys; assert (3,10) <= sys.version_info[:2] < (3,14), "Python 3.10–3.13 required"'
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
printf '%s\n' 'Installed. Start with: bash run.sh examples/demo.vmbqt'
