"""Frame-mesh backdrop (Phase 6): load an STL/3MF of the chassis and show
it, static, behind the suspension so hardpoints can be eyeballed against
the real frame.

CAD exports rarely land in our vehicle frame (mm, +X forward, origin on
the ground at the front axle), so the mesh carries a simple user
transform: uniform scale (with a unit auto-guess), a Z rotation in 90-deg
steps, and an XYZ offset. GUI-free; unit-tested where practical.
"""

from dataclasses import dataclass, asdict

import numpy as np
import pyvista as pv


@dataclass
class FrameTransform:
    scale: float = 1.0        # mesh units -> mm
    rot_z_deg: float = 0.0    # rotation about +Z, applied after scaling
    dx: float = 0.0           # mm, applied last
    dy: float = 0.0
    dz: float = 0.0

    def matrix(self) -> np.ndarray:
        a = np.radians(self.rot_z_deg)
        c, s = np.cos(a), np.sin(a)
        m = np.eye(4)
        m[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) * self.scale
        m[:3, 3] = (self.dx, self.dy, self.dz)
        return m

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "FrameTransform":
        known = {f: data[f] for f in
                 ("scale", "rot_z_deg", "dx", "dy", "dz") if f in data}
        return cls(**known)


def guess_scale(extents) -> float:
    """Guess the unit of a frame mesh from its size. A Baja frame is
    roughly 1-3 m long, so: metres -> ~2, inches -> ~80, mm -> ~2000."""
    longest = float(np.max(extents))
    if longest < 10.0:
        return 1000.0     # metres
    if longest < 300.0:
        return 25.4       # inches
    return 1.0            # already mm


def load_frame_mesh(path: str) -> pv.PolyData:
    """Load an STL/3MF into one PolyData (3MF scenes are concatenated)."""
    try:
        import trimesh
    except ImportError as e:
        raise ValueError(
            "Mesh import needs the 'trimesh' package (added to "
            "requirements.txt in a later version than your install). "
            "Fix: run  pip install -r requirements.txt  in the project "
            "folder, then restart the tool.") from e
    loaded = trimesh.load(path)
    if hasattr(loaded, "geometry"):      # a Scene of parts
        parts = list(loaded.geometry.values())
        if not parts:
            raise ValueError(f"no geometry found in {path}")
        mesh = trimesh.util.concatenate(parts)
    else:
        mesh = loaded
    faces = np.asarray(mesh.faces)
    cells = np.column_stack([np.full(len(faces), 3), faces]).ravel()
    return pv.PolyData(np.asarray(mesh.vertices, dtype=float), faces=cells)


# ----------------------------------------------------------------------
# Embedding the mesh in the project file
#
# A frame backdrop used to be stored as a PATH, which breaks the moment
# the project moves to another machine -- the exact situation the file
# format exists to survive. The mesh now travels inside the .MICK.
#
# What gets stored is the SOURCE FILE'S OWN BYTES, deflated and base64'd,
# not a re-export: `load_frame_mesh` already understands every format we
# accept, and round-tripping through a writer of ours would quietly
# change the geometry. The original path is kept alongside so the file
# can still be re-linked to a live CAD export if you want that.
#
# .3mf is a zip already, so deflate barely shrinks it (~2.4 MB stays
# ~2.4 MB, then +33% for base64). ASCII STL compresses by 5-10x.
# ----------------------------------------------------------------------

def embed_mesh_file(path: str) -> dict:
    """Pack a mesh file for storage inside a project file."""
    import base64
    import os
    import zlib
    with open(path, "rb") as fh:
        raw = fh.read()
    return {"name": os.path.basename(path),
            "bytes": len(raw),
            "data": base64.b64encode(zlib.compress(raw, 6)).decode("ascii")}


def extract_mesh_file(blob: dict, dest_dir: str) -> str:
    """Write an embedded mesh back out; returns the new file's path."""
    import base64
    import os
    import zlib
    name = os.path.basename(str(blob.get("name") or "frame.stl")) or "frame.stl"
    out = os.path.join(dest_dir, name)
    with open(out, "wb") as fh:
        fh.write(zlib.decompress(base64.b64decode(blob["data"])))
    return out


def embedded_size_mb(blob: dict | None) -> float:
    """Rough size the embedded mesh adds to a saved file, in MB."""
    if not blob or not blob.get("data"):
        return 0.0
    return len(blob["data"]) / 1024.0 / 1024.0
