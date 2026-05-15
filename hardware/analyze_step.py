"""Detailed analysis of the cleaned IWR6843AOPEVM STEP file.

Walks all leaf shapes preserving names, then:
- lists unique component names
- isolates the PCB substrate (largest flat solid)
- finds the radar AOP chip (~15x15 package, top face)
- finds USB connectors (micro-USB body ~8x5x3mm)
- finds mounting holes (cylindrical holes through PCB)
- finds the heat sink (tall, finned)
"""
from __future__ import annotations

import collections
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
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("d"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(STEP_PATH)) != IFSelect_RetDone:
        raise SystemExit("read failed")
    reader.Transfer(doc)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    print(f"Loaded in {time.time()-t0:.1f}s")

    rows = []
    free = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free)

    def walk(label, ploc, depth, path):
        nm = label_name(label)
        my_path = f"{path}/{nm}" if path else nm
        loc = shape_tool.GetLocation_s(label)
        gloc = ploc.Multiplied(loc)
        if shape_tool.IsAssembly_s(label):
            children = TDF_LabelSequence()
            shape_tool.GetComponents_s(label, children)
            for i in range(1, children.Length() + 1):
                c = children.Value(i)
                ref = TDF_Label()
                if shape_tool.GetReferredShape_s(c, ref):
                    cloc = shape_tool.GetLocation_s(c)
                    walk(ref, gloc.Multiplied(cloc), depth + 1, my_path)
                else:
                    walk(c, gloc, depth + 1, my_path)
        else:
            sh = shape_tool.GetShape_s(label)
            if sh.IsNull():
                return
            sh = sh.Located(gloc)
            bb = Bnd_Box()
            try:
                BRepBndLib.Add_s(sh, bb, True)
            except Exception:
                return
            if bb.IsVoid():
                return
            x0, y0, z0, x1, y1, z1 = bb.Get()
            rows.append((my_path, nm, x0, y0, z0, x1 - x0, y1 - y0, z1 - z0))

    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location(), 0, "")

    print(f"Total leaf solids: {len(rows)}")

    # Unique leaf names
    names = collections.Counter(r[1] for r in rows)
    print(f"\nUnique leaf names: {len(names)}")
    for nm, cnt in names.most_common(30):
        print(f"  {cnt:5d}  {nm}")

    # Unique parent paths (truncate the actual leaf name)
    paths = collections.Counter("/".join(r[0].split("/")[:-1]) for r in rows)
    print(f"\nUnique parent paths: {len(paths)}")
    for p, cnt in paths.most_common(20):
        print(f"  {cnt:5d}  {p}")

    # Largest flat solid = PCB
    flats = [r for r in rows if r[7] < 3.0 and r[5] > 30 and r[6] > 10]
    flats.sort(key=lambda r: r[5] * r[6], reverse=True)
    print("\nFlat candidates (likely PCB substrate or shield):")
    for r in flats[:8]:
        print(f"  {r[1]:<30} X[{r[2]:.2f}..{r[2]+r[5]:.2f}] Y[{r[3]:.2f}..{r[3]+r[6]:.2f}] Z[{r[4]:.2f}..{r[4]+r[7]:.2f}]  {r[5]:.2f} x {r[6]:.2f} x {r[7]:.2f}")

    # AOP-package-like (~15mm square, thin)
    aop = [r for r in rows if 12 < r[5] < 18 and 12 < r[6] < 18 and r[7] < 2.0]
    print(f"\n~15mm square thin packages ({len(aop)}):")
    for r in aop[:10]:
        cx, cy = r[2] + r[5] / 2, r[3] + r[6] / 2
        print(f"  cx={cx:.2f} cy={cy:.2f} z_top={r[4]+r[7]:.2f}  {r[5]:.2f} x {r[6]:.2f} x {r[7]:.2f}")

    # USB micro-B sized bodies (~8x5x3mm)
    usb = [r for r in rows if 7 < r[5] < 9 and 4 < r[6] < 6 and 2.5 < r[7] < 4 or
                              4 < r[5] < 6 and 7 < r[6] < 9 and 2.5 < r[7] < 4]
    print(f"\nUSB-micro-sized bodies ({len(usb)}):")
    for r in usb[:10]:
        print(f"  X0={r[2]:.2f} Y0={r[3]:.2f} Z0={r[4]:.2f}  {r[5]:.2f} x {r[6]:.2f} x {r[7]:.2f}  ({r[1]})")

    # Tall stuff (heat sink, RF shields, capacitors)
    tall = sorted(rows, key=lambda r: r[7], reverse=True)
    print("\nTop 20 tallest solids (Z extent):")
    for r in tall[:20]:
        print(f"  Z0={r[4]:.2f} dZ={r[7]:.2f}  XY={r[5]:.2f}x{r[6]:.2f}  at ({r[2]+r[5]/2:.1f},{r[3]+r[6]/2:.1f})  {r[1]}")

    # Global bbox
    gx0 = min(r[2] for r in rows)
    gy0 = min(r[3] for r in rows)
    gz0 = min(r[4] for r in rows)
    gx1 = max(r[2] + r[5] for r in rows)
    gy1 = max(r[3] + r[6] for r in rows)
    gz1 = max(r[4] + r[7] for r in rows)
    print(f"\nGlobal bbox: X[{gx0:.2f}..{gx1:.2f}] Y[{gy0:.2f}..{gy1:.2f}] Z[{gz0:.2f}..{gz1:.2f}]")
    print(f"Overall: {gx1-gx0:.2f} x {gy1-gy0:.2f} x {gz1-gz0:.2f} mm")

    print(f"\nDone in {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
