"""PyInstaller entry point for the Baja Suspension Tool.

Run directly (python launcher.py) or feed to PyInstaller via
suspension_tool.spec to build a standalone executable.
"""

from suspension_tool.gui.app import main

if __name__ == "__main__":
    main()
