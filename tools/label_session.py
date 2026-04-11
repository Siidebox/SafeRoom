#!/usr/bin/env python3
"""
label_session.py — Post-hoc labeling tool for ML dataset annotation.

Loads a session CSV produced by MlCsvLogger (or any CSV with maxZ/vz columns)
and displays an interactive plot. The user drags to select time regions and
presses a key to assign a label to that region. The annotated CSV is saved
in-place (original is backed up as .bak).

Usage:
    python tools/label_session.py logs/ml_session_YYYYMMDD_HHMMSS.csv

Controls (in the plot window):
    Drag on either panel to select a time region.
    Then press one of:
        f — label selected region 'fall'
        n — label selected region 'normal'
        s — label selected region 'sitting'
        w — label selected region 'walking'
    Other keys:
        z — undo last labeling operation
        Enter / ctrl+s — save and exit
        Escape — exit without saving

Requirements:
    pip install pandas matplotlib
"""

import argparse
import shutil
import sys

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import SpanSelector


# ── Color map for labels ─────────────────────────────────────────────────────

LABEL_COLORS = {
    'fall':    '#e74c3c',   # red
    'normal':  '#2ecc71',   # green
    'sitting': '#f39c12',   # orange
    'walking': '#3498db',   # blue
    'unknown': '#cccccc',   # grey
}

KEY_TO_LABEL = {
    'f': 'fall',
    'n': 'normal',
    's': 'sitting',
    'w': 'walking',
}


# ── Main labeler class ────────────────────────────────────────────────────────

class SessionLabeler:
    def __init__(self, csv_path: str):
        self._path       = csv_path
        self._df         = pd.read_csv(csv_path)
        self._undo_stack = []   # list of (row_mask, old_labels)
        self._span_xmin  = None
        self._span_xmax  = None
        self._saved      = False

        # Ensure 'label' column exists
        if 'label' not in self._df.columns:
            self._df['label'] = 'unknown'

        # Build a time axis: use frameNum if available, else row index
        if 'frameNum' in self._df.columns:
            self._df['_t'] = self._df['frameNum']
            xlabel = 'Frame number'
        else:
            self._df['_t'] = range(len(self._df))
            xlabel = 'Row index'

        self._xlabel = xlabel
        self._build_plot()

    def _build_plot(self):
        fig, (ax1, ax2, ax3) = plt.subplots(
            3, 1, figsize=(14, 8), sharex=True,
            gridspec_kw={'height_ratios': [3, 2, 1]}
        )
        fig.suptitle(
            f'Label session: {self._path}\n'
            'Drag to select, then press: f=fall  n=normal  s=sitting  w=walking  |  '
            'z=undo  Enter=save  Esc=quit',
            fontsize=9
        )
        self._fig = fig
        self._ax1 = ax1
        self._ax2 = ax2
        self._ax3 = ax3

        df = self._df
        t  = df['_t'].values

        # ── Panel 1: maxZ timeseries ─────────────────────────────────────────
        if 'maxZ' in df.columns:
            ax1.plot(t, df['maxZ'].fillna(method='ffill'), color='#2c3e50', lw=1.0,
                     label='maxZ')
        if 'height_m' in df.columns:
            ax1.plot(t, df['height_m'].fillna(method='ffill'), color='#95a5a6',
                     lw=0.8, label='height_m', alpha=0.7)
        ax1.axhline(0.80, color='r', ls='--', lw=0.8, alpha=0.5, label='floor threshold')
        ax1.set_ylabel('Height (m)')
        ax1.legend(fontsize=7, loc='upper right')
        ax1.grid(True, alpha=0.3)

        # ── Panel 2: vz and vertical acceleration ────────────────────────────
        if 'vz' in df.columns:
            ax2.plot(t, df['vz'].fillna(0), color='#e67e22', lw=1.0, label='vz')
        if 'az' in df.columns:
            ax2.plot(t, df['az'].fillna(0), color='#9b59b6', lw=0.8,
                     label='az', alpha=0.7)
        ax2.axhline(-0.80, color='r', ls='--', lw=0.8, alpha=0.5,
                    label='vz threshold')
        ax2.axhline(0, color='k', lw=0.5, alpha=0.3)
        ax2.set_ylabel('Velocity / Accel (m/s)')
        ax2.legend(fontsize=7, loc='upper right')
        ax2.grid(True, alpha=0.3)

        # ── Panel 3: current labels (color bar) ─────────────────────────────
        ax3.set_ylabel('Label')
        ax3.set_xlabel(self._xlabel)
        ax3.set_yticks([])
        ax3.grid(False)

        self._redraw_labels()

        # ── SpanSelector (drag to select region) ────────────────────────────
        self._span = SpanSelector(
            ax1,
            onselect=self._on_select,
            direction='horizontal',
            useblit=True,
            props=dict(alpha=0.25, facecolor='steelblue'),
            interactive=True,
        )

        # ── Legend for labels ────────────────────────────────────────────────
        patches = [
            mpatches.Patch(color=c, label=l)
            for l, c in LABEL_COLORS.items() if l != 'unknown'
        ]
        fig.legend(handles=patches, loc='lower center', ncol=4, fontsize=8,
                   framealpha=0.8)

        fig.canvas.mpl_connect('key_press_event', self._on_key)
        plt.tight_layout(rect=[0, 0.06, 1, 1])

    def _redraw_labels(self):
        ax = self._ax3
        ax.cla()
        ax.set_ylabel('Label')
        ax.set_yticks([])

        df  = self._df
        t   = df['_t'].values
        lbl = df['label'].values

        if len(t) < 2:
            return

        # Draw colored spans for each contiguous label region
        i = 0
        while i < len(lbl):
            j = i + 1
            while j < len(lbl) and lbl[j] == lbl[i]:
                j += 1
            color = LABEL_COLORS.get(str(lbl[i]), '#cccccc')
            ax.axvspan(t[i], t[j - 1], color=color, alpha=0.8)
            i = j

        self._fig.canvas.draw_idle()

    def _on_select(self, xmin, xmax):
        self._span_xmin = xmin
        self._span_xmax = xmax

    def _on_key(self, event):
        key = event.key
        if key in KEY_TO_LABEL:
            self._apply_label(KEY_TO_LABEL[key])
        elif key == 'z':
            self._undo()
        elif key in ('enter', 'ctrl+s'):
            self._save()
            plt.close('all')
        elif key == 'escape':
            print('[LABEL] Exiting without saving.')
            plt.close('all')

    def _apply_label(self, label: str):
        xmin = self._span_xmin
        xmax = self._span_xmax
        if xmin is None or xmax is None:
            print('[LABEL] No region selected. Drag first, then press a key.')
            return

        df   = self._df
        mask = (df['_t'] >= xmin) & (df['_t'] <= xmax)
        n    = mask.sum()
        if n == 0:
            print('[LABEL] Selection contains no rows.')
            return

        # Save undo state
        self._undo_stack.append((mask.copy(), df.loc[mask, 'label'].copy()))

        df.loc[mask, 'label'] = label
        print(f'[LABEL] Labeled {n} rows as "{label}" (frames {xmin:.0f}–{xmax:.0f})')
        self._redraw_labels()

        # Reset selection
        self._span_xmin = None
        self._span_xmax = None

    def _undo(self):
        if not self._undo_stack:
            print('[LABEL] Nothing to undo.')
            return
        mask, old_labels = self._undo_stack.pop()
        self._df.loc[mask, 'label'] = old_labels
        print(f'[LABEL] Undo: restored {mask.sum()} rows.')
        self._redraw_labels()

    def _save(self):
        # Backup original
        shutil.copy2(self._path, self._path + '.bak')

        # Write annotated CSV (drop the temp column)
        out = self._df.drop(columns=['_t'])
        out.to_csv(self._path, index=False)
        self._saved = True
        print(f'[LABEL] Saved to {self._path}  (backup: {self._path}.bak)')

    def run(self):
        plt.show()
        if not self._saved:
            ans = input('Changes not saved. Save now? [y/N]: ').strip().lower()
            if ans == 'y':
                self._save()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Post-hoc labeling tool for ML session CSVs.'
    )
    parser.add_argument('csv', help='Path to ML session CSV file')
    args = parser.parse_args()

    labeler = SessionLabeler(args.csv)
    labeler.run()


if __name__ == '__main__':
    main()
