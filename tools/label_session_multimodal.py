#!/usr/bin/env python3
"""
label_session_multimodal.py — Post-hoc labeler for synchronized radar+IR sessions.

Input: a session directory containing radar.csv + thermal.npz + manifest.json
(produced by session_recorder.py or synth_session.py).

What you see:
    ax1  maxZ / height_m timeseries
    ax2  vz / az timeseries
    ax3  MLX90640 heatmap of the IR frame nearest to the cursor
    ax4  current label color bar

Interactions:
    Drag on ax1 or ax2 to select a time range.
    Then press one of:
        f  fall
        g  fall_lying (post-impact lying, until getting up)
        a  near_fall
        s  sit
        l  lie
        w  walk
        t  stand
        n  none
    Other keys:
        z       undo
        Enter   save (writes radar.csv labels in-place AND manifest.json
                LabelSpans)
        Esc     quit without saving

Move the mouse over the time-series plots to scrub the IR heatmap.

Usage:
    python tools/label_session_multimodal.py sessions/<session_id>/
"""

import argparse
import os
import shutil
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import SpanSelector

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from manifest_schema import Manifest, LabelSpan    # noqa: E402


LABEL_COLORS = {
    'fall':       '#e74c3c',
    'fall_lying': '#c0392b',
    'near_fall':  '#e67e22',
    'sit':       '#f39c12',
    'lie':       '#9b59b6',
    'walk':      '#3498db',
    'stand':     '#1abc9c',
    'none':      '#2ecc71',
    'unknown':   '#cccccc',
}

KEY_TO_LABEL = {
    'f': 'fall',
    'g': 'fall_lying',
    'a': 'near_fall',
    's': 'sit',
    'l': 'lie',
    'w': 'walk',
    't': 'stand',
    'n': 'none',
}


class MultimodalLabeler:
    def __init__(self, session_dir: str):
        self._dir = session_dir
        self._radar_path = os.path.join(session_dir, 'radar.csv')
        self._thermal_path = os.path.join(session_dir, 'thermal.npz')

        if not os.path.isfile(self._radar_path):
            raise FileNotFoundError(f'Missing radar.csv in {session_dir}')

        self._df = pd.read_csv(self._radar_path)
        if 'label' not in self._df.columns:
            self._df['label'] = 'unknown'

        # Time base: seconds since session start, derived from t_mono_ns when
        # present (synth/new sessions), otherwise frameNum/20.
        if 't_mono_ns' in self._df.columns:
            t_mono = self._df['t_mono_ns'].to_numpy(dtype=np.int64)
            self._t_mono_radar = t_mono
            self._t0_mono = int(t_mono.min())
            self._t = (t_mono - self._t0_mono) / 1e9
        else:
            print('[LABEL] WARNING: radar.csv has no t_mono_ns column; falling '
                  'back to frameNum/20 for the time axis.')
            self._t_mono_radar = None
            self._t0_mono = 0
            fn = self._df.get('frameNum', pd.Series(range(len(self._df))))
            self._t = fn.to_numpy(dtype=np.float64) / 20.0

        self._df['_t'] = self._t

        # ── Thermal ──
        self._has_ir = os.path.isfile(self._thermal_path)
        if self._has_ir:
            npz = np.load(self._thermal_path)
            self._ir_frames = npz['frames']        # (N, 24, 32)
            if 't_mono_ns' in npz.files:
                self._ir_t = (npz['t_mono_ns'].astype(np.int64)
                              - self._t0_mono) / 1e9
            else:
                # Legacy npz: distribute uniformly across the session duration
                n_ir = len(self._ir_frames)
                self._ir_t = np.linspace(0, max(self._t.max(), 1e-3), n_ir)
            print(f'[LABEL] Loaded thermal: {self._ir_frames.shape}  '
                  f'span={self._ir_t.min():.2f}–{self._ir_t.max():.2f}s')
        else:
            self._ir_frames = None
            self._ir_t = None
            print('[LABEL] No thermal.npz in session; running radar-only.')

        # ── Manifest ──
        try:
            self._manifest = Manifest.load(session_dir)
        except FileNotFoundError:
            print('[LABEL] No manifest.json; will create one on save.')
            self._manifest = Manifest(
                session_id=os.path.basename(os.path.normpath(session_dir)),
            )

        self._span_xmin = None
        self._span_xmax = None
        self._undo_stack = []
        self._saved = False

        self._build_plot()

    # ── plot ─────────────────────────────────────────────────────────────────

    def _build_plot(self):
        fig = plt.figure(figsize=(15, 9))
        gs = fig.add_gridspec(3, 3, height_ratios=[3, 2, 0.6],
                              width_ratios=[3, 0.05, 1.4])
        ax1 = fig.add_subplot(gs[0, 0])
        ax2 = fig.add_subplot(gs[1, 0], sharex=ax1)
        ax_lbl = fig.add_subplot(gs[2, 0], sharex=ax1)
        ax_ir = fig.add_subplot(gs[:, 2])

        fig.suptitle(
            f'{self._dir} — drag to select, then press: '
            'f=fall  g=fall_lying  a=near_fall  s=sit  l=lie  w=walk  t=stand  n=none  '
            '|  z=undo  Enter=save  Esc=quit',
            fontsize=9,
        )
        self._fig = fig
        self._ax1 = ax1
        self._ax2 = ax2
        self._ax_lbl = ax_lbl
        self._ax_ir = ax_ir

        df = self._df
        t  = self._t

        if 'maxZ' in df.columns:
            ax1.plot(t, df['maxZ'].ffill(), color='#2c3e50', lw=1.0, label='maxZ')
        if 'height_m' in df.columns:
            ax1.plot(t, df['height_m'].ffill(), color='#95a5a6', lw=0.8,
                     label='height_m', alpha=0.7)
        ax1.axhline(0.80, color='r', ls='--', lw=0.8, alpha=0.5,
                    label='floor threshold')
        ax1.set_ylabel('Height (m)')
        ax1.legend(fontsize=7, loc='upper right')
        ax1.grid(True, alpha=0.3)

        if 'vz' in df.columns:
            ax2.plot(t, df['vz'].fillna(0), color='#e67e22', lw=1.0, label='vz')
        if 'az' in df.columns:
            ax2.plot(t, df['az'].fillna(0), color='#9b59b6', lw=0.8,
                     label='az', alpha=0.7)
        ax2.axhline(-1.15, color='r', ls='--', lw=0.8, alpha=0.5,
                    label='vz threshold')
        ax2.axhline(0, color='k', lw=0.5, alpha=0.3)
        ax2.set_ylabel('Velocity / Accel (m/s, m/s²)')
        ax2.set_xlabel('Time (s)')
        ax2.legend(fontsize=7, loc='upper right')
        ax2.grid(True, alpha=0.3)

        ax_lbl.set_yticks([])
        ax_lbl.set_xlabel('Time (s)')

        if self._has_ir:
            vmin = float(np.percentile(self._ir_frames, 1))
            vmax = float(np.percentile(self._ir_frames, 99))
            self._ir_im = ax_ir.imshow(
                self._ir_frames[0], cmap='inferno', vmin=vmin, vmax=vmax,
                aspect='equal',
            )
            ax_ir.set_title('IR (cursor-sync)', fontsize=9)
            cbar = self._fig.colorbar(self._ir_im, ax=ax_ir, fraction=0.046)
            cbar.ax.tick_params(labelsize=7)
        else:
            ax_ir.text(0.5, 0.5, 'no thermal.npz', ha='center', va='center')
            ax_ir.axis('off')
            self._ir_im = None

        self._cursor_lines = [
            ax1.axvline(t[0], color='cyan', lw=1.0, alpha=0.6),
            ax2.axvline(t[0], color='cyan', lw=1.0, alpha=0.6),
        ]

        self._redraw_labels()

        self._span1 = SpanSelector(
            ax1, onselect=self._on_select, direction='horizontal',
            useblit=True, props=dict(alpha=0.25, facecolor='steelblue'),
            interactive=True,
        )
        self._span2 = SpanSelector(
            ax2, onselect=self._on_select, direction='horizontal',
            useblit=True, props=dict(alpha=0.25, facecolor='steelblue'),
            interactive=True,
        )

        patches = [mpatches.Patch(color=c, label=l)
                   for l, c in LABEL_COLORS.items() if l != 'unknown']
        fig.legend(handles=patches, loc='lower center', ncol=8, fontsize=8,
                   framealpha=0.8)

        fig.canvas.mpl_connect('key_press_event', self._on_key)
        fig.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)
        plt.tight_layout(rect=[0, 0.04, 1, 0.97])

    def _redraw_labels(self):
        ax = self._ax_lbl
        ax.cla()
        ax.set_yticks([])
        ax.set_xlabel('Time (s)')
        ax.set_xlim(self._ax1.get_xlim())

        t   = self._t
        lbl = self._df['label'].to_numpy()

        if len(t) < 2:
            return

        i = 0
        while i < len(lbl):
            j = i + 1
            while j < len(lbl) and lbl[j] == lbl[i]:
                j += 1
            color = LABEL_COLORS.get(str(lbl[i]), '#cccccc')
            ax.axvspan(t[i], t[j - 1], color=color, alpha=0.8)
            i = j

        self._fig.canvas.draw_idle()

    # ── interaction ──────────────────────────────────────────────────────────

    def _on_select(self, xmin, xmax):
        if xmax - xmin < 1e-6:
            return
        self._span_xmin = xmin
        self._span_xmax = xmax

    def _on_mouse_move(self, event):
        if event.inaxes not in (self._ax1, self._ax2):
            return
        x = event.xdata
        if x is None:
            return
        for ln in self._cursor_lines:
            ln.set_xdata([x, x])
        if self._has_ir and self._ir_im is not None:
            idx = int(np.argmin(np.abs(self._ir_t - x)))
            self._ir_im.set_data(self._ir_frames[idx])
            self._ax_ir.set_title(
                f'IR t={self._ir_t[idx]:.2f}s  (frame {idx})', fontsize=9,
            )
        self._fig.canvas.draw_idle()

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
        if self._span_xmin is None:
            print('[LABEL] No region selected. Drag first, then press a key.')
            return
        xmin, xmax = self._span_xmin, self._span_xmax
        mask = (self._t >= xmin) & (self._t <= xmax)
        n = int(mask.sum())
        if n == 0:
            print('[LABEL] Selection contains no rows.')
            return
        self._undo_stack.append((mask, self._df.loc[mask, 'label'].copy()))
        self._df.loc[mask, 'label'] = label
        print(f'[LABEL] Labeled {n} rows as "{label}"  '
              f'(t={xmin:.2f}–{xmax:.2f}s)')
        self._redraw_labels()
        self._span_xmin = None
        self._span_xmax = None

    def _undo(self):
        if not self._undo_stack:
            print('[LABEL] Nothing to undo.')
            return
        mask, old = self._undo_stack.pop()
        self._df.loc[mask, 'label'] = old
        print(f'[LABEL] Undo: restored {int(mask.sum())} rows.')
        self._redraw_labels()

    # ── save ─────────────────────────────────────────────────────────────────

    def _save(self):
        # Back up CSV once
        bak = self._radar_path + '.bak'
        if not os.path.isfile(bak):
            shutil.copy2(self._radar_path, bak)

        out = self._df.drop(columns=['_t'])
        out.to_csv(self._radar_path, index=False)
        print(f'[LABEL] radar.csv saved (backup: {bak})')

        # Manifest: rebuild labels as contiguous spans from the per-row column
        spans = []
        if self._t_mono_radar is not None:
            lbl = self._df['label'].to_numpy()
            tmn = self._t_mono_radar
            i = 0
            while i < len(lbl):
                j = i + 1
                while j < len(lbl) and lbl[j] == lbl[i]:
                    j += 1
                if str(lbl[i]) not in ('unknown', '', 'nan'):
                    spans.append(LabelSpan(
                        t_start_mono_ns=int(tmn[i]),
                        t_end_mono_ns=int(tmn[j - 1]),
                        label=str(lbl[i]),
                    ))
                i = j
            self._manifest.labels = spans
            self._manifest.write(self._dir)
            print(f'[LABEL] manifest.json updated with {len(spans)} label spans')
        else:
            print('[LABEL] No t_mono_ns in CSV — manifest spans not updated.')

        self._saved = True

    def run(self):
        plt.show()
        if not self._saved:
            ans = input('Changes not saved. Save now? [y/N]: ').strip().lower()
            if ans == 'y':
                self._save()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('session_dir', help='Path to session directory')
    args = ap.parse_args()
    MultimodalLabeler(args.session_dir).run()


if __name__ == '__main__':
    main()
