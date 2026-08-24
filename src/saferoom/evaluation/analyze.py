#!/usr/bin/env python3
"""
analyze_replay.py — Event-level replay analysis and thesis figures.

Derives two thesis artifacts from an existing replay results JSON
(`figures/replay_results.json`, produced by tools/replay_session.py). This
script does NOT re-run training or replay — it only aggregates the recorded
per-session event metrics.

Outputs (under --outdir, default figures/eval):
  1. latency_histogram.png
       Pooled per-detection latencies for the "rules" and "ml" detectors
       across all fall sessions, overlaid with median lines.
  2. ir_side_split.png + ir_side_split.csv
       Fall sessions split by ACTUAL radar geometry (median X of the fall-
       labeled rows in each session's radar.csv), NOT by session name
       (some izq/dcha names are swapped). x<=0 => "IR-visible", x>0 =>
       "IR-blind". Aggregates event recall / false alarms / latency per
       detector per zone, showing the IR-fusion fail-closed effect.

Usage:
    python tools/analyze_replay.py \
        --json figures/replay_results.json \
        --sessions-root sessions \
        --outdir figures/eval
"""

import argparse
import json
import os
import statistics
import sys

import matplotlib

matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt
import numpy as np

DETECTORS = ["rules", "ml", "rules+ir", "ml+ir"]


def norm_path(p):
    """Normalize Windows-style backslash paths to the current OS separator."""
    return os.path.normpath(p.replace("\\", "/"))


def load_results(json_path):
    with open(json_path, encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, list):
        raise ValueError(f"{json_path} must contain a JSON list")
    return data


def fall_sessions(results):
    return [s for s in results if s.get("n_gt_fall_events", 0) > 0]


def median_fall_x(session, sessions_root):
    """Median X of fall-labeled rows in this session's radar.csv, or None."""
    # Prefer session_dir; fall back to sessions_root/session_id.
    candidates = []
    sd = session.get("session_dir")
    if sd:
        candidates.append(norm_path(sd))
    sid = session.get("session_id")
    if sid:
        candidates.append(os.path.join(sessions_root, sid))

    csv_path = None
    for base in candidates:
        cand = os.path.join(base, "radar.csv")
        if os.path.isfile(cand):
            csv_path = cand
            break
    if csv_path is None:
        print(f"  [warn] radar.csv not found for {sid} (tried: {candidates})",
              file=sys.stderr)
        return None

    # Read only x + label columns.
    xs = []
    try:
        with open(csv_path, encoding="utf-8") as f:
            header = f.readline().rstrip("\n").split(",")
            try:
                ix = header.index("x")
                il = header.index("label")
            except ValueError:
                print(f"  [warn] {csv_path} missing x/label columns", file=sys.stderr)
                return None
            ncols = len(header)
            for line in f:
                parts = line.rstrip("\n").split(",")
                if len(parts) < ncols:
                    continue
                lab = parts[il].strip().strip('"')
                if not lab.startswith("fall"):
                    continue
                try:
                    xs.append(float(parts[ix]))
                except ValueError:
                    continue
    except OSError as e:
        print(f"  [warn] could not read {csv_path}: {e}", file=sys.stderr)
        return None

    if not xs:
        print(f"  [warn] no fall-labeled rows in {csv_path}", file=sys.stderr)
        return None
    return statistics.median(xs)


# --------------------------------------------------------------------------
# Deliverable 1: latency histogram
# --------------------------------------------------------------------------
def build_latency_histogram(falls, outdir):
    pools = {"rules": [], "ml": []}
    for s in falls:
        for det in ("rules", "ml"):
            m = s["detectors"].get(det, {}).get("metrics", {})
            for lat in m.get("latencies_s", []) or []:
                if lat is not None:
                    pools[det].append(float(lat))

    fig, ax = plt.subplots(figsize=(8, 5))
    colors = {"rules": "#1f77b4", "ml": "#d62728"}
    all_lat = pools["rules"] + pools["ml"]
    if all_lat:
        bins = np.linspace(0, max(all_lat) * 1.05 + 1e-6, 25)
    else:
        bins = 25

    for det in ("rules", "ml"):
        vals = pools[det]
        if not vals:
            continue
        ax.hist(vals, bins=bins, alpha=0.55, color=colors[det],
                label=f"{det} (n={len(vals)})", edgecolor="white", linewidth=0.4)
        med = statistics.median(vals)
        ax.axvline(med, color=colors[det], linestyle="--", linewidth=1.8,
                   label=f"{det} median = {med:.3f} s")

    ax.set_xlabel("Detection latency (s)")
    ax.set_ylabel("Number of detections")
    ax.set_title("Fall detection latency — rules vs ML (pooled over fall sessions)")
    ax.legend()
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    out = os.path.join(outdir, "latency_histogram.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out, {k: len(v) for k, v in pools.items()}


# --------------------------------------------------------------------------
# Deliverable 2: IR-side split
# --------------------------------------------------------------------------
def aggregate_zone(sessions_in_zone, detector):
    events = detected = fas = 0
    dur = 0.0
    session_lat_medians = []
    for s in sessions_in_zone:
        m = s["detectors"].get(detector, {}).get("metrics", {})
        events += int(m.get("n_events", 0) or 0)
        detected += int(m.get("n_detected", 0) or 0)
        fas += int(m.get("n_false_alarms", 0) or 0)
        dur += float(s.get("duration_s", 0.0) or 0.0)
        lm = m.get("latency_median_s")
        if lm is not None:
            session_lat_medians.append(float(lm))
    recall = (detected / events) if events else float("nan")
    fa_per_hour = (fas / (dur / 3600.0)) if dur > 0 else float("nan")
    lat_med = statistics.median(session_lat_medians) if session_lat_medians else None
    return {
        "gt_events": events,
        "detected": detected,
        "recall": recall,
        "false_alarms": fas,
        "fa_per_hour": fa_per_hour,
        "latency_median_s": lat_med,
    }


def build_ir_side_split(falls, sessions_root, outdir):
    zones = {"IR-visible": [], "IR-blind": []}
    for s in falls:
        mx = median_fall_x(s, sessions_root)
        if mx is None:
            print(f"  [warn] skipping {s.get('session_id')} from side split "
                  "(no usable radar.csv)", file=sys.stderr)
            continue
        zone = "IR-visible" if mx <= 0 else "IR-blind"
        s["_median_fall_x"] = mx
        zones[zone].append(s)

    zone_sets = {
        "ALL": zones["IR-visible"] + zones["IR-blind"],
        "IR-visible": zones["IR-visible"],
        "IR-blind": zones["IR-blind"],
    }

    # CSV
    csv_path = os.path.join(outdir, "ir_side_split.csv")
    rows = []
    with open(csv_path, "w", encoding="utf-8", newline="") as f:
        f.write("zone,detector,gt_events,detected,recall,false_alarms,"
                "fa_per_hour,latency_median_s\n")
        for zone in ("ALL", "IR-visible", "IR-blind"):
            for det in DETECTORS:
                a = aggregate_zone(zone_sets[zone], det)
                lat = "" if a["latency_median_s"] is None else f"{a['latency_median_s']:.6f}"
                f.write(
                    f"{zone},{det},{a['gt_events']},{a['detected']},"
                    f"{a['recall']:.4f},{a['false_alarms']},"
                    f"{a['fa_per_hour']:.4f},{lat}\n"
                )
                rows.append((zone, det, a))

    # Grouped bar chart: event recall per detector, grouped by zone.
    zone_order = ["ALL", "IR-visible", "IR-blind"]
    agg = {(z, d): aggregate_zone(zone_sets[z], d) for z in zone_order for d in DETECTORS}
    x = np.arange(len(zone_order))
    width = 0.2
    colors = {"rules": "#1f77b4", "ml": "#d62728",
              "rules+ir": "#7fb3d5", "ml+ir": "#f1948a"}

    fig, ax = plt.subplots(figsize=(9, 5.5))
    for i, det in enumerate(DETECTORS):
        vals = []
        for z in zone_order:
            r = agg[(z, det)]["recall"]
            vals.append(0.0 if (r != r) else r)  # NaN -> 0 bar
        bars = ax.bar(x + (i - 1.5) * width, vals, width, label=det, color=colors[det])
        for b, v in zip(bars, vals, strict=False):
            ax.annotate(f"{v:.2f}", (b.get_x() + b.get_width() / 2, v),
                        ha="center", va="bottom", fontsize=7)

    counts = {z: len(zone_sets[z]) for z in zone_order}
    ax.set_xticks(x)
    ax.set_xticklabels([f"{z}\n(n={counts[z]})" for z in zone_order])
    ax.set_ylabel("Event recall")
    ax.set_ylim(0, 1.12)
    ax.set_title("Fall event recall by detector and IR zone\n"
                 "(IR fusion fail-closed: ml+ir recall collapses in the IR-blind zone)")
    ax.legend(title="Detector", ncol=4, loc="upper center", fontsize=8)
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    png_path = os.path.join(outdir, "ir_side_split.png")
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    return csv_path, png_path, zone_sets, agg


# --------------------------------------------------------------------------
def print_summary(falls, lat_counts, zone_sets, agg):
    print("\n" + "=" * 78)
    print("REPLAY ANALYSIS SUMMARY")
    print("=" * 78)
    print(f"Fall sessions analyzed: {len(falls)}")
    print(f"Pooled latencies -> rules: {lat_counts['rules']}, ml: {lat_counts['ml']}")

    print("\nIR-side split (by median fall X in radar.csv):")
    for z in ("ALL", "IR-visible", "IR-blind"):
        print(f"  {z}: {len(zone_sets[z])} session(s)")

    hdr = ("zone", "detector", "gt", "det", "recall", "FA", "FA/h", "lat_med_s")
    print("\n{:<11} {:<9} {:>4} {:>4} {:>7} {:>4} {:>7} {:>10}".format(*hdr))
    print("-" * 78)
    for z in ("ALL", "IR-visible", "IR-blind"):
        for d in DETECTORS:
            a = agg[(z, d)]
            recall = "n/a" if a["recall"] != a["recall"] else f"{a['recall']:.3f}"
            faph = "n/a" if a["fa_per_hour"] != a["fa_per_hour"] else f"{a['fa_per_hour']:.2f}"
            lat = "n/a" if a["latency_median_s"] is None else f"{a['latency_median_s']:.3f}"
            print("{:<11} {:<9} {:>4} {:>4} {:>7} {:>4} {:>7} {:>10}".format(
                z, d, a["gt_events"], a["detected"], recall,
                a["false_alarms"], faph, lat))
        print("-" * 78)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default="figures/replay_results.json")
    ap.add_argument("--sessions-root", default="sessions")
    ap.add_argument("--outdir", default="figures/eval")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    results = load_results(args.json)
    falls = fall_sessions(results)
    if not falls:
        print("No fall sessions (n_gt_fall_events>0) found — nothing to do.")
        return

    print(f"Loaded {len(results)} sessions, {len(falls)} with fall events.")

    lat_png, lat_counts = build_latency_histogram(falls, args.outdir)
    print(f"Wrote {lat_png}")

    csv_path, png_path, zone_sets, agg = build_ir_side_split(
        falls, args.sessions_root, args.outdir)
    print(f"Wrote {csv_path}")
    print(f"Wrote {png_path}")

    print_summary(falls, lat_counts, zone_sets, agg)


if __name__ == "__main__":
    main()
