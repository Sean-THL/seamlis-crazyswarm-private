#!/usr/bin/env python3
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../../../.."))
SEAMLIS_DIR = os.path.join(REPO_ROOT, "seamlis")

if SEAMLIS_DIR not in sys.path:
    sys.path.insert(0, SEAMLIS_DIR)


from cf_state_subscriber import main  # noqa: E402


if __name__ == "__main__":
    main()
