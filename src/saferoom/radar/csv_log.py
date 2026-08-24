"""Per-track CSV logging for recorded sessions."""

import csv
import math
import time


class CsvLogger:
    def __init__(self, path: str):
        self._f = open(path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow([
            'timestamp', 'frameNum', 'presence',
            'tid', 'x', 'y', 'z', 'vx', 'vy', 'vz',
            'height_m', 'maxZ', 'minZ', 'maxZ_ref', 'peak_vz',
            'fall_detected', 'faint_detected'
        ])
        print(f'Logging to {path}')

    def log(self, frame: dict, fall_tids: set, faint_tids: set = None, fall_detector=None):
        ts = time.time()
        presence = frame['presence']
        fn = frame['frameNum']
        if not frame['tracks']:
            self._w.writerow([ts, fn, presence,
                              '', '', '', '', '', '', '',
                              '', '', '', '', ''])
        for t in frame['tracks']:
            tid = t['tid']
            h = frame['heights'].get(tid, {})
            max_z = h.get('maxZ', float('nan'))
            min_z = h.get('minZ', float('nan'))
            height_m = (max_z - min_z) if (not math.isnan(max_z) and not math.isnan(min_z)) else float('nan')
            ref      = round(fall_detector.ref_maxz(tid), 3)          if fall_detector else ''
            peak_vz  = round(fall_detector._last_peak_vz.get(tid, 0.0), 3) if fall_detector else ''
            self._w.writerow([
                ts, fn, presence,
                tid, round(t['x'], 3), round(t['y'], 3), round(t['z'], 3),
                round(t['vx'], 3), round(t['vy'], 3), round(t['vz'], 3),
                round(height_m, 3) if not math.isnan(height_m) else '',
                round(max_z, 3) if not math.isnan(max_z) else '',
                round(min_z, 3) if not math.isnan(min_z) else '',
                ref, peak_vz,
                1 if tid in fall_tids else 0,
                1 if (faint_tids and tid in faint_tids) else 0
            ])
        self._f.flush()

    def close(self):
        self._f.close()
