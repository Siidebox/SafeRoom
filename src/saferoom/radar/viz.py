"""Real-time PyQtGraph visualizer.

Renders the live point cloud, tracked targets and fall/faint counters in a 2D
window, optionally with an OpenGL 3D panel and an IR thermal panel. Qt and
OpenGL are imported lazily inside ``RadarWindow.__init__`` so that headless
runs (recording, replay, CI) never need them.
"""

import collections
import math
import time

import numpy as np


def _person_box_edges(cx: float, cy: float, minZ: float, maxZ: float,
                      hw: float = 0.2) -> np.ndarray:
    """
    Return (24, 3) array of line-segment endpoints forming a wireframe box
    around a person centred at (cx, cy) with half-width hw, from minZ to maxZ.
    Used for GLLinePlotItem with mode='lines' (each pair = one segment).
    """
    x0, x1 = cx - hw, cx + hw
    y0, y1 = cy - hw, cy + hw
    z0, z1 = minZ, maxZ
    return np.array([
        # bottom rectangle
        [x0, y0, z0], [x1, y0, z0],  [x1, y0, z0], [x1, y1, z0],
        [x1, y1, z0], [x0, y1, z0],  [x0, y1, z0], [x0, y0, z0],
        # top rectangle
        [x0, y0, z1], [x1, y0, z1],  [x1, y0, z1], [x1, y1, z1],
        [x1, y1, z1], [x0, y1, z1],  [x0, y1, z1], [x0, y0, z1],
        # vertical edges
        [x0, y0, z0], [x0, y0, z1],  [x1, y0, z0], [x1, y0, z1],
        [x1, y1, z0], [x1, y1, z1],  [x0, y1, z0], [x0, y1, z1],
    ], dtype=np.float32)


def _snr_to_rgba(snr_array: np.ndarray) -> np.ndarray:
    """Map SNR values (0–20) to RGBA: red=low, yellow=mid, green=high."""
    t = np.clip(snr_array / 20.0, 0.0, 1.0)
    r = np.where(t < 0.5, 255, (255 * (1.0 - t) * 2).clip(0, 255)).astype(np.uint8)
    g = np.where(t < 0.5, (255 * t * 2).clip(0, 255), 255).astype(np.uint8)
    b = np.zeros(len(t), dtype=np.uint8)
    a = np.full(len(t), 180, dtype=np.uint8)
    return np.column_stack([r, g, b, a])


class RadarWindow:
    """
    PyQtGraph-based real-time visualizer.

    Layout:
      [ Top view X-Y ] [ Side view X-Z ] [ Fall counter ]
      [ Status bar (spanning all columns) ]

    Driven by a QTimer that polls frame_queue every 30 ms (~33 fps redraw cap).
    The actual frame rate is limited by the radar (100 ms / 10 fps).
    """

    TRAIL_LEN = 50   # positions to keep per track

    def __init__(self, frame_queue, stop_event, boundary_box=None, plot3d=False,
                 ir_queue=None, ir_rotate: int = 0):
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets

        # PyQt5 had flat Qt.* enums (Qt.DashLine, Qt.AlignCenter); PyQt6
        # nests them under PenStyle / AlignmentFlag. Resolve both shapes.
        _QtEnums_pen   = getattr(QtCore.Qt, 'PenStyle',      QtCore.Qt)
        _QtEnums_align = getattr(QtCore.Qt, 'AlignmentFlag', QtCore.Qt)
        _Q_DASH         = _QtEnums_pen.DashLine
        _Q_ALIGN_CENTER = _QtEnums_align.AlignCenter

        self._queue    = frame_queue
        self._ir_queue = ir_queue
        # ir_rotate is the number of 90° CCW rotations applied to each IR frame
        # before display (0/1/2/3). Use to compensate physical camera tilt.
        self._ir_rotate = int(ir_rotate) % 4
        self._ir_img   = None
        self._ir_last_temp = None
        self._stop  = stop_event
        self._total_falls  = 0
        self._total_faints = 0
        # M1 — track trails and persistent scene items (reused each frame)
        self._trails: dict = {}        # tid -> deque of (x, y, z)
        self._trail_curves: dict = {}  # tid -> (PlotCurveItem_xy, PlotCurveItem_xz)
        self._track_labels: dict = {}  # tid -> (TextItem_xy, TextItem_xz)
        # 3D GL items (None if --plot3d not set)
        self._gl_view  = None
        self._gl_scat  = None
        self._gl_trails: dict = {}     # tid -> GLLinePlotItem (trail)
        self._gl_boxes:  dict = {}     # tid -> GLLinePlotItem (person wireframe)
        # M3 — flash alert
        self._flash_until = 0.0      # epoch time when flash expires
        self._flash_color = ''

        pg.setConfigOptions(antialias=True, background='w', foreground='k')

        self._app = pg.mkQApp('SafeRoom Radar')

        # ── Main window ──────────────────────────────────────────────────────
        self._win = QtWidgets.QWidget()
        self._win.setWindowTitle('SafeRoom Radar — IWR6843 3D People Tracking')
        self._win.resize(1600 if plot3d else 1200, 560)

        main_layout = QtWidgets.QHBoxLayout(self._win)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(6)

        # ── Plots (left side) ─────────────────────────────────────────────
        self._glw = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self._glw, stretch=7 if plot3d else 10)

        # Top view X-Y
        self._plot_xy = self._glw.addPlot(row=0, col=0, title='Top view (X-Y)')
        self._plot_xy.setLabel('bottom', 'X (m)')
        self._plot_xy.setLabel('left',   'Y (m)')
        self._plot_xy.setXRange(-2, 2, padding=0)
        self._plot_xy.setYRange(0, 5, padding=0)
        self._plot_xy.setAspectLocked(True)
        self._plot_xy.showGrid(x=True, y=True, alpha=0.3)
        self._scat_xy = pg.ScatterPlotItem(size=7, pen=None)
        self._plot_xy.addItem(self._scat_xy)

        # Side view X-Z
        self._plot_xz = self._glw.addPlot(row=0, col=1, title='Side view (X-Z)')
        self._plot_xz.setLabel('bottom', 'X (m)')
        self._plot_xz.setLabel('left',   'Z (m)')
        self._plot_xz.setXRange(-2, 2, padding=0)
        self._plot_xz.setYRange(0, 2.5, padding=0)
        self._plot_xz.setAspectLocked(True)
        self._plot_xz.showGrid(x=True, y=True, alpha=0.3)
        self._scat_xz = pg.ScatterPlotItem(size=7, pen=None)
        self._plot_xz.addItem(self._scat_xz)

        # M2 — Boundary box (static, drawn once)
        if boundary_box is not None:
            xmin, xmax, ymin, ymax, zmin, zmax = boundary_box
            box_pen = pg.mkPen(color=(80, 80, 200), width=1.5,
                               style=_Q_DASH)
            # Top view: X-Y rectangle
            bx_xy = np.array([xmin, xmax, xmax, xmin, xmin])
            by_xy = np.array([ymin, ymin, ymax, ymax, ymin])
            self._plot_xy.plot(bx_xy, by_xy, pen=box_pen)
            # Side view: X-Z rectangle
            bx_xz = np.array([xmin, xmax, xmax, xmin, xmin])
            bz_xz = np.array([zmin, zmin, zmax, zmax, zmin])
            self._plot_xz.plot(bx_xz, bz_xz, pen=box_pen)
            # Sensor marker (triangle ▲) in top view at origin
            self._plot_xy.plot([0], [0], pen=None,
                               symbol='t', symbolSize=12,
                               symbolBrush=(255, 100, 0), symbolPen=None)

        # Status bar below both plots
        self._status = self._glw.addLabel(
            '', row=1, col=0, colspan=2,
            color=(80, 80, 80), size='9pt'
        )

        # ── Fall counter panel (right side) ──────────────────────────────
        cnt_widget = QtWidgets.QWidget()
        cnt_widget.setFixedWidth(120)
        cnt_widget.setStyleSheet('background-color: #fff0f0; border-radius: 6px;')
        cnt_layout = QtWidgets.QVBoxLayout(cnt_widget)
        cnt_layout.setAlignment(_Q_ALIGN_CENTER)

        lbl_title = QtWidgets.QLabel('FALLS\ndetected')
        lbl_title.setAlignment(_Q_ALIGN_CENTER)
        lbl_title.setStyleSheet('color: darkred; font-weight: bold; font-size: 11px;')

        self._lbl_count = QtWidgets.QLabel('0')
        self._lbl_count.setAlignment(_Q_ALIGN_CENTER)
        self._lbl_count.setStyleSheet(
            'color: red; font-size: 52px; font-weight: bold;')

        self._lbl_last = QtWidgets.QLabel('')
        self._lbl_last.setAlignment(_Q_ALIGN_CENTER)
        self._lbl_last.setStyleSheet('color: gray; font-size: 9px;')

        lbl_faint_title = QtWidgets.QLabel('FAINTS\ndetected')
        lbl_faint_title.setAlignment(_Q_ALIGN_CENTER)
        lbl_faint_title.setStyleSheet(
            'color: darkorange; font-weight: bold; font-size: 11px; margin-top: 8px;')

        self._lbl_faint_count = QtWidgets.QLabel('0')
        self._lbl_faint_count.setAlignment(_Q_ALIGN_CENTER)
        self._lbl_faint_count.setStyleSheet(
            'color: orange; font-size: 36px; font-weight: bold;')

        cnt_layout.addStretch()
        cnt_layout.addWidget(lbl_title)
        cnt_layout.addWidget(self._lbl_count)
        cnt_layout.addWidget(self._lbl_last)
        cnt_layout.addWidget(lbl_faint_title)
        cnt_layout.addWidget(self._lbl_faint_count)
        cnt_layout.addStretch()

        main_layout.addWidget(cnt_widget, stretch=1)

        # ── Optional IR (MLX90640) heatmap panel ─────────────────────────
        if self._ir_queue is not None:
            ir_glw = pg.GraphicsLayoutWidget()
            ir_glw.setMinimumWidth(260)
            ir_plot = ir_glw.addPlot(row=0, col=0, title='IR (MLX90640)')
            ir_plot.setAspectLocked(True)
            ir_plot.hideAxis('bottom')
            ir_plot.hideAxis('left')
            ir_plot.invertY(True)   # match image-style top-down orientation

            # Initial frame: 24×32 zeros (or rotated shape)
            initial = np.zeros((24, 32), dtype=np.float32)
            if self._ir_rotate:
                initial = np.rot90(initial, k=self._ir_rotate)
            self._ir_img = pg.ImageItem(initial)

            # Inferno-like colormap (built-in lookup)
            try:
                cmap = pg.colormap.get('inferno', source='matplotlib')
            except Exception:
                cmap = pg.colormap.get('CET-L8')   # fallback bundled cmap
            self._ir_img.setLookupTable(cmap.getLookupTable())
            self._ir_img.setLevels((20.0, 38.0))   # auto-rescaled on first real frame
            ir_plot.addItem(self._ir_img)

            self._ir_label = ir_glw.addLabel(
                'IR  —  waiting…',
                row=1, col=0, color=(80, 80, 80), size='9pt',
            )
            main_layout.addWidget(ir_glw, stretch=3)

        # ── Optional 3D GL view ──────────────────────────────────────────
        if plot3d:
            try:
                import pyqtgraph.opengl as gl

                gl_widget = gl.GLViewWidget()
                gl_widget.setMinimumWidth(380)
                gl_widget.setCameraPosition(distance=6.0, elevation=25, azimuth=225)
                # Insert between the 2D plots and the fall counter
                main_layout.insertWidget(1, gl_widget, stretch=5)
                self._gl_view = gl_widget

                # Floor grid centred on the room
                grid = gl.GLGridItem()
                grid.setSize(x=3, y=5)
                grid.setSpacing(x=0.5, y=0.5)
                grid.translate(0, 2.0, 0)
                gl_widget.addItem(grid)

                # Boundary box wireframe
                if boundary_box is not None:
                    xmin, xmax, ymin, ymax, zmin, zmax = boundary_box
                    box = gl.GLBoxItem(color=(80, 80, 200, 60))
                    box.setSize(x=xmax - xmin, y=ymax - ymin, z=zmax - zmin)
                    box.translate(xmin, ymin, zmin)
                    gl_widget.addItem(box)

                # Sensor marker at origin
                sensor_dot = gl.GLScatterPlotItem(
                    pos=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
                    color=np.array([[1.0, 0.4, 0.0, 1.0]], dtype=np.float32),
                    size=14, pxMode=True,
                )
                gl_widget.addItem(sensor_dot)

                # Point cloud scatter (starts empty)
                self._gl_scat = gl.GLScatterPlotItem(
                    pos=np.zeros((1, 3), dtype=np.float32),
                    color=np.zeros((1, 4), dtype=np.float32),
                    size=5, pxMode=True,
                )
                gl_widget.addItem(self._gl_scat)

            except ImportError:
                print('[WARN] --plot3d requires PyOpenGL: pip install PyOpenGL')
                self._gl_view = None

        # ── Timer ────────────────────────────────────────────────────────
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(30)   # poll every 30 ms

    def _update(self):
        import datetime

        import pyqtgraph as pg

        if self._stop.is_set():
            self._timer.stop()
            self._app.quit()
            return

        # IR is independent from radar frame queue — pull latest if available.
        if self._ir_queue is not None and self._ir_img is not None:
            ir_latest = None
            try:
                while True:
                    ir_latest = self._ir_queue.get_nowait()
            except Exception:
                pass
            if ir_latest is not None:
                arr, t_mono = ir_latest
                if self._ir_rotate:
                    arr = np.rot90(arr, k=self._ir_rotate)
                # ImageItem expects (col, row) ordering — pass transposed view.
                self._ir_img.setImage(arr.T, autoLevels=False)
                # Adaptive levels: 1st / 99th percentile of this frame.
                lo = float(np.percentile(arr, 1))
                hi = float(np.percentile(arr, 99))
                if hi - lo < 1.0:
                    hi = lo + 1.0
                self._ir_img.setLevels((lo, hi))
                self._ir_last_temp = (lo, float(arr.max()), hi)
                self._ir_label.setText(
                    f'IR  min {arr.min():.1f}°C  max {arr.max():.1f}°C  '
                    f'(rot {self._ir_rotate*90}°)'
                )

        if self._queue.empty():
            return

        frame, fall_tids, faint_tids = self._queue.get_nowait()
        pts     = frame['points']
        tracks  = frame['tracks']
        heights = frame['heights']
        presence = frame['presence']

        # ── Point cloud ───────────────────────────────────────────────────
        if pts:
            xs   = np.array([p['x']   for p in pts], dtype=np.float32)
            ys   = np.array([p['y']   for p in pts], dtype=np.float32)
            zs   = np.array([p['z']   for p in pts], dtype=np.float32)
            snrs = np.array([p['snr'] for p in pts], dtype=np.float32)
            colors = _snr_to_rgba(snrs)
            # Pass Nx4 uint8 numpy array directly — avoids creating N QBrush objects
            self._scat_xy.setData(x=xs, y=ys, brush=colors)
            self._scat_xz.setData(x=xs, y=zs, brush=colors)
        else:
            self._scat_xy.setData(x=[], y=[])
            self._scat_xz.setData(x=[], y=[])

        # ── Update trail buffers + remove gone tracks ─────────────────────
        active_tids = {t['tid'] for t in tracks}
        for gone in list(self._trails.keys()):
            if gone not in active_tids:
                del self._trails[gone]
                if gone in self._trail_curves:
                    self._plot_xy.removeItem(self._trail_curves[gone][0])
                    self._plot_xz.removeItem(self._trail_curves[gone][1])
                    del self._trail_curves[gone]
                if gone in self._track_labels:
                    self._plot_xy.removeItem(self._track_labels[gone][0])
                    self._plot_xz.removeItem(self._track_labels[gone][1])
                    del self._track_labels[gone]

        # ── Draw tracks and trails (reuse existing scene items) ───────────
        for t in tracks:
            tid = t['tid']
            is_fall  = tid in fall_tids
            is_faint = tid in faint_tids
            color = (220, 30, 30) if is_fall else (220, 140, 0) if is_faint else (30, 90, 200)

            h = heights.get(tid, {})
            cluster_h = h.get('maxZ', 0.0) - h.get('minZ', 0.0) if h else float('nan')
            z_label = h.get('maxZ', t['z']) if h else t['z']

            # Append current position to trail
            if tid not in self._trails:
                self._trails[tid] = collections.deque(maxlen=self.TRAIL_LEN)
            self._trails[tid].append((t['x'], t['y'], z_label))

            # Trail: reuse PlotCurveItem, only addItem on first appearance
            trail = self._trails[tid]
            if len(trail) >= 2:
                txs = np.array([p[0] for p in trail], dtype=np.float32)
                tys = np.array([p[1] for p in trail], dtype=np.float32)
                tzs = np.array([p[2] for p in trail], dtype=np.float32)
                trail_pen = pg.mkPen(color=(*color, 80), width=1.5)
                if tid in self._trail_curves:
                    self._trail_curves[tid][0].setData(x=txs, y=tys)
                    self._trail_curves[tid][0].setPen(trail_pen)
                    self._trail_curves[tid][1].setData(x=txs, y=tzs)
                    self._trail_curves[tid][1].setPen(trail_pen)
                else:
                    ln_xy = pg.PlotCurveItem(x=txs, y=tys, pen=trail_pen)
                    ln_xz = pg.PlotCurveItem(x=txs, y=tzs, pen=trail_pen)
                    self._plot_xy.addItem(ln_xy)
                    self._plot_xz.addItem(ln_xz)
                    self._trail_curves[tid] = (ln_xy, ln_xz)

            label = f"T{tid}"
            if not math.isnan(cluster_h):
                label += f"\nh={cluster_h:.2f}m"
            if is_fall:
                label += "\n⚠ FALL"
            elif is_faint:
                label += "\n⚠ FAINT"

            # Labels: reuse TextItem, only addItem on first appearance
            # t['z'] from TLV 1010 is at ground level (tracker projection).
            # Place label at maxZ (top of cluster) so it sits on the person.
            if tid in self._track_labels:
                txt_xy, txt_xz = self._track_labels[tid]
                txt_xy.setText(label)
                txt_xy.setColor(color)
                txt_xy.setPos(t['x'], t['y'])
                txt_xz.setText(label)
                txt_xz.setColor(color)
                txt_xz.setPos(t['x'], z_label)
            else:
                txt_xy = pg.TextItem(label, color=color, anchor=(0.5, 1.0))
                txt_xy.setPos(t['x'], t['y'])
                self._plot_xy.addItem(txt_xy)
                txt_xz = pg.TextItem(label, color=color, anchor=(0.5, 1.0))
                txt_xz.setPos(t['x'], z_label)
                self._plot_xz.addItem(txt_xz)
                self._track_labels[tid] = (txt_xy, txt_xz)

        # ── 3D GL update (only when --plot3d) ────────────────────────────
        if self._gl_view is not None:
            import pyqtgraph.opengl as gl

            # Point cloud
            if pts:
                pos3d = np.column_stack([xs, ys, zs]).astype(np.float32)
                col3d = (colors.astype(np.float32) / 255.0)
                self._gl_scat.setData(pos=pos3d, color=col3d)
            else:
                self._gl_scat.setData(
                    pos=np.zeros((1, 3), dtype=np.float32),
                    color=np.zeros((1, 4), dtype=np.float32),
                )

            # Remove items for gone tracks
            for gone in [tid for tid in self._gl_trails if tid not in active_tids]:
                self._gl_view.removeItem(self._gl_trails.pop(gone))
            for gone in [tid for tid in self._gl_boxes if tid not in active_tids]:
                self._gl_view.removeItem(self._gl_boxes.pop(gone))

            for t in tracks:
                tid = t['tid']
                is_fall  = tid in fall_tids
                is_faint = tid in faint_tids
                r, g, b = (0.86, 0.12, 0.12) if is_fall else \
                          (0.86, 0.55, 0.0)  if is_faint else \
                          (0.12, 0.35, 0.78)

                # Trail
                trail = self._trails.get(tid)
                if trail and len(trail) >= 2:
                    pos3d = np.array([[p[0], p[1], p[2]] for p in trail],
                                     dtype=np.float32)
                    if tid in self._gl_trails:
                        self._gl_trails[tid].setData(pos=pos3d,
                                                     color=(r, g, b, 0.5))
                    else:
                        line = gl.GLLinePlotItem(pos=pos3d,
                                                 color=(r, g, b, 0.5),
                                                 width=2.0, antialias=True)
                        self._gl_view.addItem(line)
                        self._gl_trails[tid] = line

                # Person wireframe box (uses TLV 1012 height data)
                h = heights.get(tid)
                if h:
                    minZ = h.get('minZ', 0.0)
                    maxZ = h.get('maxZ', minZ + 0.3)
                    edges = _person_box_edges(t['x'], t['y'], minZ, maxZ)
                    if tid in self._gl_boxes:
                        self._gl_boxes[tid].setData(pos=edges,
                                                    color=(r, g, b, 0.9))
                    else:
                        box_line = gl.GLLinePlotItem(pos=edges,
                                                     color=(r, g, b, 0.9),
                                                     width=1.5,
                                                     antialias=True,
                                                     mode='lines')
                        self._gl_view.addItem(box_line)
                        self._gl_boxes[tid] = box_line

        # ── Fall / faint counters + M3 flash ─────────────────────────────
        if fall_tids:
            self._total_falls += len(fall_tids)
            self._lbl_count.setText(str(self._total_falls))
            now_str = datetime.datetime.now().strftime('%H:%M:%S')
            self._lbl_last.setText(f'last\n{now_str}')
            self._flash_until = time.time() + 2.0
            self._flash_color = '#ffcccc'
        if faint_tids:
            self._total_faints += len(faint_tids)
            self._lbl_faint_count.setText(str(self._total_faints))
            if not fall_tids:   # fall flash takes priority
                self._flash_until = time.time() + 2.0
                self._flash_color = '#ffe4b5'

        # Apply or clear flash background
        if time.time() < self._flash_until:
            self._win.setStyleSheet(f'background-color: {self._flash_color};')
        else:
            self._win.setStyleSheet('')

        # ── Status bar ────────────────────────────────────────────────────
        pres_str = '● PRESENT' if presence else '○ empty'
        if fall_tids:
            alert_str = '  ⚠ FALL DETECTED'
            alert_color = '#cc0000'
        elif faint_tids:
            alert_str = '  ⚠ FAINT DETECTED'
            alert_color = '#cc7700'
        else:
            alert_str = ''
            alert_color = '#505050'
        self._status.setText(
            f"Frame {frame['frameNum']}  |  {len(pts)} pts  |  "
            f"{len(tracks)} tracks  |  {pres_str}{alert_str}",
            color=alert_color,
            size='9pt',
        )

    def show(self):
        self._win.show()

    def exec(self):
        self._app.exec()

