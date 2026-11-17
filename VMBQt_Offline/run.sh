#!/usr/bin/env bash
set -euo pipefail
vmbqt_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$vmbqt_dir"
if [[ ! -x .venv/bin/python ]]; then
  printf '%s\n' 'Run: bash setup_ubuntu.sh (or create .venv and install requirements.txt)'
  exit 1
fi
exec .venv/bin/python -m vmbqt "$@"
