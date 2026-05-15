"""Extract per-component bounding boxes from PROC091G.step.

Uses OpenCascade through cadquery-ocp. Walks the assembly tree, applies
location transforms, and computes axis-aligned bounding boxes in the global
frame. Output is a tab-separated table sorted by volume so the small parts
(connectors, ICs, heat sink) surface separately from the PCB and the debug
board.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopLoc import TopLoc_Location

STEP_PATH = Path(r"C:/Users/sideb/chicago/IIT/tfm/SafeRoom/measures/PROC091G (1).step")


def label_name(label: TDF_Label) -> str:
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return "<unnamed>"


def main() -> None:
    t0 = time.time()
    print(f"Opening {STEP_PATH.name} ({STEP_PATH.stat().st_size / 1e6:.1f} MB)...", flush=True)

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("step-doc"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(False)
    reader.SetLayerMode(False)
    status = reader.ReadFile(str(STEP_PATH))
    if status != IFSelect_RetDone:
        print(f"ReadFile failed: {status}", file=sys.stderr)
        sys.exit(1)
    print(f"  parsed in {time.time()-t0:.1f}s, transferring...", flush=True)
    t1 = time.time()
    reader.Transfer(doc)
    print(f"  transferred in {time.time()-t1:.1f}s", flush=True)

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    free_shapes = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free_shapes)
    print(f"Top-level free shapes: {free_shapes.Length()}", flush=True)

    rows: list[tuple[str, float, float, float, float, float, float, float]] = []

    def walk(label: TDF_Label, parent_loc: TopLoc_Location, depth: int, path: str) -> None:
        name = label_name(label)
        my_path = f"{path}/{name}" if path else name

        # Compose local location
        loc = shape_tool.GetLocation_s(label)
        global_loc = parent_loc.Multiplied(loc)

        if shape_tool.IsAssembly_s(label):
            children = TDF_LabelSequence()
            shape_tool.GetComponents_s(label, children)
            for i in range(1, children.Length() + 1):
                comp = children.Value(i)
                # If component is a reference, follow it
                ref = TDF_Label()
                if shape_tool.GetReferredShape_s(comp, ref):
                    comp_loc = shape_tool.GetLocation_s(comp)
                    walk(ref, global_loc.Multiplied(comp_loc), depth + 1, my_path)
                else:
                    walk(comp, global_loc, depth + 1, my_path)
        else:
            shape = shape_tool.GetShape_s(label)
            if shape.IsNull():
                return
            shape = shape.Located(global_loc)
            bb = Bnd_Box()
            try:
                BRepBndLib.Add_s(shape, bb, True)
            except Exception:
                return
            if bb.IsVoid():
                return
            xmin, ymin, zmin, xmax, ymax, zmax = bb.Get()
            dx, dy, dz = xmax - xmin, ymax - ymin, zmax - zmin
            rows.append((my_path, xmin, ymin, zmin, dx, dy, dz, dx * dy * dz))

    for i in range(1, free_shapes.Length() + 1):
        walk(free_shapes.Value(i), TopLoc_Location(), 0, "")

    rows.sort(key=lambda r: r[7], reverse=True)
    print(f"\nLeaf parts found: {len(rows)}\n")
    print(f"{'name':<70} {'X':>9} {'Y':>9} {'Z':>9} {'dX':>8} {'dY':>8} {'dZ':>8}")
    for r in rows[:60]:
        name = r[0][-70:]
        print(f"{name:<70} {r[1]:9.2f} {r[2]:9.2f} {r[3]:9.2f} {r[4]:8.2f} {r[5]:8.2f} {r[6]:8.2f}")

    # Global bbox
    if rows:
        gx0 = min(r[1] for r in rows)
        gy0 = min(r[2] for r in rows)
        gz0 = min(r[3] for r in rows)
        gx1 = max(r[1] + r[4] for r in rows)
        gy1 = max(r[2] + r[5] for r in rows)
        gz1 = max(r[3] + r[6] for r in rows)
        print(f"\nGlobal bbox: X[{gx0:.2f}..{gx1:.2f}] Y[{gy0:.2f}..{gy1:.2f}] Z[{gz0:.2f}..{gz1:.2f}]")
        print(f"Overall size: {gx1-gx0:.2f} x {gy1-gy0:.2f} x {gz1-gz0:.2f} mm")

    print(f"\nTotal time: {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
