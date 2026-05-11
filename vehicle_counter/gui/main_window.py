import cv2

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QToolBar,
    QWidget,
)

from vehicle_counter.utils import load_config

from .stats_panel import StatsPanel
from .video_widget import VideoWidget
from .worker import WorkerThread


class MainWindow(QMainWindow):

    def __init__(self, config_path: str = "config.yaml"):
        super().__init__()
        self.setWindowTitle("Vehicle Counter")
        self.setMinimumSize(1000, 600)

        self._config = load_config(config_path)
        self._video_path: str | None = None
        self._worker: WorkerThread | None = None
        self._line_start: tuple | None = None
        self._line_end: tuple | None = None

        self._build_ui()
        self._build_toolbar()
        self._set_status("Open a video file to begin.")

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(1)

        self._video = VideoWidget()
        self._video.line_drawn.connect(self._on_line_drawn)
        layout.addWidget(self._video, stretch=1)

        self._stats = StatsPanel()
        layout.addWidget(self._stats)

    def _build_toolbar(self):
        tb = QToolBar("Controls")
        tb.setMovable(False)
        tb.setStyleSheet("QToolBar { spacing: 6px; padding: 4px; }")
        self.addToolBar(tb)

        self._act_open = self._action(tb, "Open Video", "Ctrl+O", self._open_video)

        tb.addSeparator()

        self._act_start = self._action(tb, "▶  Start", "Ctrl+Return", self._start,
                                       enabled=False)
        self._act_stop = self._action(tb, "■  Stop", "Ctrl+.", self._stop,
                                      enabled=False)

        tb.addSeparator()

        self._act_clear = self._action(tb, "Clear Line", None, self._clear_line,
                                       enabled=False)

        tb.addSeparator()

        # Hint label
        self._hint = QLabel("  Drag on video to draw counting line")
        self._hint.setStyleSheet("color: #888888; font-size: 11px;")
        tb.addWidget(self._hint)

    @staticmethod
    def _action(toolbar: QToolBar, text: str, shortcut: str | None,
                slot, enabled: bool = True) -> QAction:
        act = QAction(text)
        if shortcut:
            act.setShortcut(QKeySequence(shortcut))
        act.triggered.connect(slot)
        act.setEnabled(enabled)
        toolbar.addAction(act)
        return act

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------

    def _open_video(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Video", "",
            "Video files (*.mp4 *.avi *.mov *.mkv *.webm *.m4v);;All files (*)",
        )
        if not path:
            return

        self._video_path = path
        self._clear_line()

        cap = cv2.VideoCapture(path)
        ret, frame = cap.read()
        cap.release()

        if not ret:
            QMessageBox.critical(self, "Error", f"Cannot read video:\n{path}")
            return

        self._video.set_frame(frame)
        self._act_clear.setEnabled(True)
        self._set_status(f"{path}  —  drag on the video to draw a counting line, then press Start.")

    def _on_line_drawn(self, start: tuple, end: tuple):
        self._line_start = start
        self._line_end = end
        self._act_start.setEnabled(True)
        self._hint.setText(f"  Line: {start} → {end}")
        self._set_status("Line set. Press ▶ Start (Ctrl+Return) to begin counting.")

    def _start(self):
        if not self._video_path:
            return
        if not self._line_start or not self._line_end:
            QMessageBox.warning(self, "No counting line",
                                "Draw a counting line on the video first.")
            return

        self._stats.reset()
        self._act_start.setEnabled(False)
        self._act_stop.setEnabled(True)
        self._act_open.setEnabled(False)
        self._set_status("Running…")

        self._worker = WorkerThread(
            self._video_path, self._line_start, self._line_end, self._config, parent=self
        )
        self._worker.frame_ready.connect(self._on_frame, Qt.ConnectionType.QueuedConnection)
        self._worker.count_updated.connect(self._stats.update_counts,
                                           Qt.ConnectionType.QueuedConnection)
        self._worker.finished.connect(self._on_finished, Qt.ConnectionType.QueuedConnection)
        self._worker.start()

    def _stop(self):
        if self._worker and self._worker.isRunning():
            self._worker.stop()

    def _clear_line(self):
        self._video.clear_line()
        self._line_start = None
        self._line_end = None
        self._act_start.setEnabled(False)
        self._hint.setText("  Drag on video to draw counting line")
        self._set_status("Line cleared. Draw a new counting line.")

    def _on_frame(self, frame):
        self._video.set_frame(frame)

    def _on_finished(self, counts: dict, total: int):
        self._act_start.setEnabled(True)
        self._act_stop.setEnabled(False)
        self._act_open.setEnabled(True)
        self._set_status(
            f"Done  —  Total: {total}   Down ↓: {counts['down']}   Up ↑: {counts['up']}"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _set_status(self, msg: str):
        self.statusBar().showMessage(msg)

    def closeEvent(self, event):
        self._stop()
        if self._worker:
            self._worker.wait(3000)
        event.accept()
