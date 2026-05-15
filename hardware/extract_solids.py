"""Enumerate every solid in PROC091G.step with its bounding box.

The STEP exports a single 'PCB' product containing many solids without
component names. We walk each TopoDS_Solid, compute its global bbox, and
print a CSV-ish table so we can cluster by size and identify components
(PCB substrate, heat sink, connectors, IC package).
"""
from __future__ import annotations

import time
from pathlib import Path

from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

STEP_PATH = Path(r"C:/Users/sideb/chicago/IIT/tfm/SafeRoom/measures/PROC091G.step")
OUT_CSV = Path(r"C:/Users/sideb/chicago/IIT/tfm/SafeRoom/hardware/solids.csv")


def main() -> None:
    t0 = time.time()
    reader = STEPControl_Reader()
    print(f"Reading {STEP_PATH.name}...", flush=True)
    if reader.ReadFile(str(STEP_PATH)) != IFSelect_RetDone:
        raise SystemExit("read failed")
    reader.TransferRoots()
    shape = reader.OneShape()
    print(f"  loaded in {time.time()-t0:.1f}s, exploring solids...", flush=True)

    rows = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        s = exp.Current()
        bb = Bnd_Box()
        BRepBndLib.Add_s(s, bb, True)
        if not bb.IsVoid():
            x0, y0, z0, x1, y1, z1 = bb.Get()
            rows.append((x0, y0, z0, x1 - x0, y1 - y0, z1 - z0))
        exp.Next()

    print(f"Solids: {len(rows)}")

    with OUT_CSV.open("w", encoding="utf-8") as f:
        f.write("x0,y0,z0,dx,dy,dz,vol\n")
        for r in rows:
            vol = r[3] * r[4] * r[5]
            f.write(f"{r[0]:.3f},{r[1]:.3f},{r[2]:.3f},{r[3]:.3f},{r[4]:.3f},{r[5]:.3f},{vol:.3f}\n")
    print(f"Wrote {OUT_CSV}")

    # Identify the PCB (largest XY footprint, flat in Z)
    by_xy = sorted(rows, key=lambda r: r[3] * r[4], reverse=True)
    pcb_candidates = [r for r in by_xy[:5] if r[5] < 5.0]
    print("\nPCB candidates (flat, largest XY):")
    for r in pcb_candidates:
        print(f"  X0={r[0]:.2f} Y0={r[1]:.2f} Z0={r[2]:.2f}  dX={r[3]:.2f} dY={r[4]:.2f} dZ={r[5]:.2f}")

    # Largest volume solids
    by_vol = sorted(rows, key=lambda r: r[3] * r[4] * r[5], reverse=True)
    print("\nTop 15 solids by volume:")
    for r in by_vol[:15]:
        print(f"  X0={r[0]:.2f} Y0={r[1]:.2f} Z0={r[2]:.2f}  dX={r[3]:.2f} dY={r[4]:.2f} dZ={r[5]:.2f}  vol={r[3]*r[4]*r[5]:.0f}")

    # Tallest solids (likely heat sink + tall connectors)
    by_z = sorted(rows, key=lambda r: r[5], reverse=True)
    print("\nTop 15 tallest solids:")
    for r in by_z[:15]:
        print(f"  X0={r[0]:.2f} Y0={r[1]:.2f} Z0={r[2]:.2f}  dX={r[3]:.2f} dY={r[4]:.2f} dZ={r[5]:.2f}")

    print(f"\nDone in {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
