#!/usr/bin/env bash
# Shortest Path Project -- macOS / Linux launcher.
# Detects python3, installs networkx + matplotlib if missing, runs the program.
set -e

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

if ! command -v python3 &> /dev/null; then
    echo "Error: python3 is not installed."
    echo "Please install Python 3.9 or later:"
    echo "  - macOS:  brew install python   (or download from https://www.python.org/downloads/)"
    echo "  - Linux:  use your distro package manager (e.g. apt install python3 python3-pip)"
    exit 1
fi

if ! python3 -c "import networkx, matplotlib" 2>/dev/null; then
    echo "Installing required packages (networkx, matplotlib) -- one-time setup..."
    if ! python3 -m pip install --user --quiet networkx matplotlib 2>/dev/null; then
        echo "Standard install failed; retrying with --break-system-packages..."
        if ! python3 -m pip install --user --quiet --break-system-packages networkx matplotlib; then
            echo
            echo "Failed to install dependencies automatically."
            echo "Try manually:  pip3 install --user networkx matplotlib"
            echo "Or use a virtual environment:"
            echo "    python3 -m venv .venv && source .venv/bin/activate && pip install networkx matplotlib"
            exit 1
        fi
    fi
fi

python3 shortest_path.py
