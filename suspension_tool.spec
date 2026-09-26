# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller spec for the Baja Suspension Tool.

Build (from the repo root, on the TARGET platform — PyInstaller does not
cross-compile, so build the Windows exe ON Windows):

    pip install -r requirements.txt pyinstaller
    pyinstaller --noconfirm suspension_tool.spec

Output: dist/BajaSuspensionTool/BajaSuspensionTool(.exe) — a one-folder
app (one-folder starts much faster than one-file for VTK-sized bundles;
zip the folder to share it). The GitHub Actions workflow in
.github/workflows/build-windows-exe.yml runs exactly this on a Windows
runner and uploads the zipped folder as an artifact.
"""

from PyInstaller.utils.hooks import collect_data_files, collect_submodules

# pyvista/vtk and PySide6 have community hooks (pyinstaller-hooks-contrib,
# installed with pyinstaller) that pull in the binary plugins; we only add
# the data files hooks occasionally miss.
datas = collect_data_files("pyvista") + collect_data_files("trimesh")

a = Analysis(
    ["launcher.py"],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=[
        "vtkmodules.all",
        "vtkmodules.util.data_model",
        "vtkmodules.util.execution_model",
        "matplotlib.backends.backend_qtagg",
        # trimesh imports its file-format loaders dynamically, so the
        # analyser misses them (and networkx/lxml are lazy deps of the
        # 3MF path) — pull them all in explicitly.
        *collect_submodules("trimesh"),
        "networkx",
        "lxml",
        "lxml.etree",
    ],
    hookspath=[],
    runtime_hooks=[],
    excludes=["tkinter", "PyQt5", "PyQt6"],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="BajaSuspensionTool",
    debug=False,
    strip=False,
    upx=False,
    console=False,          # GUI app: no console window on Windows
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    name="BajaSuspensionTool",
)
