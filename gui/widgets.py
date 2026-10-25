"""Reusable view-only widgets. No control law or communication in this file."""
from __future__ import annotations
import numpy as np
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QPolygonF, QFont, QImage
from PyQt5.QtWidgets import (QWidget, QFrame, QLabel, QVBoxLayout, QHBoxLayout,
                             QDoubleSpinBox, QScrollArea, QTableWidget, QHeaderView,
                             QAbstractItemView)
from gui.style import AXIS_COLORS


def label(text: str, style: str = 'Muted', wrap: bool = True) -> QLabel:
    w = QLabel(text)
    w.setObjectName(style)
    w.setWordWrap(wrap)
    return w


class Card(QFrame):
    def __init__(self, title: str, subtitle: str = '') -> None:
        super().__init__()
        self.setObjectName('Card')
        self.box = QVBoxLayout(self)
        self.box.setContentsMargins(16, 14, 16, 14)
        self.box.setSpacing(10)
        self.box.addWidget(label(title, 'CardTitle'))
        if subtitle:
            self.box.addWidget(label(subtitle))


class StatTile(Card):
    def __init__(self, title: str, unit: str = '', color: str = '#e8f0fa') -> None:
        super().__init__(title)
        line = QHBoxLayout()
        self.value = label('--', 'StatValue', False)
        self.value.setStyleSheet(f'color: {color};')
        line.addWidget(self.value)
        line.addWidget(label(unit, 'Muted', False), alignment=Qt.AlignBottom)
        line.addStretch()
        self.box.addLayout(line)


def spin(low: float, high: float, value: float, decimals: int = 1,
         step: float = 1.) -> QDoubleSpinBox:
    w = QDoubleSpinBox()
    w.setRange(low, high)
    w.setDecimals(decimals)
    w.setSingleStep(step)
    w.setValue(value)
    w.setKeyboardTracking(False)
    w.setMinimumWidth(90)
    return w


def scroll_page(widget: QWidget) -> QScrollArea:
    area = QScrollArea()
    area.setWidgetResizable(True)
    area.setWidget(widget)
    area.setFrameShape(QFrame.NoFrame)
    return area


def make_table(headers: list[str], rows: int) -> QTableWidget:
    table = QTableWidget(rows, len(headers))
    table.setHorizontalHeaderLabels(headers)
    table.verticalHeader().setVisible(False)
    table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    table.setEditTriggers(QAbstractItemView.NoEditTriggers)
    table.setSelectionMode(QAbstractItemView.NoSelection)
    table.setAlternatingRowColors(True)
    table.setShowGrid(False)
    table.verticalHeader().setDefaultSectionSize(32)
    table.setMinimumHeight(38 + rows * 32)
    table.setMaximumHeight(45 + rows * 32)
    return table


class CubeView(QWidget):
    """Interactive projected 3-D cube driven only by telemetry.

    Dragging the rendered body does not directly edit the displayed attitude.
    Instead it emits a disturbance request to the simulator, so the same
    telemetry path shows the deviation and subsequent controller recovery.

    Mouse mapping:
      * drag left/right -> Yaw disturbance
      * drag up/down    -> Pitch disturbance
      * Shift + drag left/right -> Roll disturbance
    """
    disturbance_requested = pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(220)
        self.setMouseTracking(True)
        self.setCursor(Qt.OpenHandCursor)
        self.setToolTip('Simulation: drag the cube to disturb it. Shift + horizontal drag disturbs Roll.')
        self.attitude = np.zeros(3)
        self._drag_pos = None
        self._vertices = np.array([[-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
                                   [-1,-1,1], [1,-1,1], [1,1,1], [-1,1,1]], dtype=float)

    def set_attitude(self, attitude: np.ndarray) -> None:
        self.attitude = np.asarray(attitude, dtype=float).copy()
        self.update()

    @staticmethod
    def matrix(a: np.ndarray) -> np.ndarray:
        r, p, y = np.deg2rad(a)
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])

    @staticmethod
    def _view_matrix() -> np.ndarray:
        # Fixed isometric-like camera basis. Body orientation comes from telemetry.
        return np.array([[.707, -.707, 0.], [.408, .408, -.816], [.577, .577, .577]])

    def _project(self, xyz: np.ndarray, scale: float, center: np.ndarray) -> np.ndarray:
        # Mild perspective makes depth changes visually apparent while avoiding an
        # OpenGL dependency. z is shifted positive before perspective division.
        z = xyz[:, 2] + 6.0
        perspective = 5.2 / np.maximum(z, 1.0)
        return xyz[:, :2] * perspective[:, None] * scale + center

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._drag_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._drag_pos is None or not (event.buttons() & Qt.LeftButton):
            super().mouseMoveEvent(event)
            return
        delta = event.pos() - self._drag_pos
        self._drag_pos = event.pos()
        # About 0.12 deg/pixel gives a natural shove without huge jumps. Each
        # mouse event is clamped so fast drags remain bounded.
        dx = float(np.clip(delta.x() * 0.12, -5., 5.))
        dy = float(np.clip(delta.y() * 0.12, -5., 5.))
        if event.modifiers() & Qt.ShiftModifier:
            disturbance = np.array([dx, 0., 0.])
        else:
            disturbance = np.array([0., -dy, dx])
        if np.linalg.norm(disturbance) > 0.02:
            self.disturbance_requested.emit(disturbance.tolist())
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._drag_pos = None
            self.setCursor(Qt.OpenHandCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#030303'))
        w, h = self.width(), self.height()

        # Sparse technical floor grid; black background remains dominant.
        p.setPen(QPen(QColor('#171717'), 1))
        horizon = int(h * .66)
        for x in range(-w, w * 2, 34):
            p.drawLine(w // 2, horizon, x, h)
        for i in range(7):
            yy = horizon + int((h - horizon) * (i / 7) ** 1.55)
            p.drawLine(0, yy, w, yy)

        view = self._view_matrix()
        rot = self.matrix(self.attitude)
        transformed = (view @ rot @ self._vertices.T).T
        scale = min(w / 4.2, h / 3.5)
        center = np.array([w * .49, h * .48])
        points = self._project(transformed, scale, center)

        faces = [(0,1,2,3), (4,5,6,7), (0,1,5,4), (1,2,6,5), (2,3,7,6), (3,0,4,7)]
        colors = ['#0c0d0f', '#15171a', '#0a0b0d', '#1b1e22', '#111316', '#08090a']
        for idx in sorted(range(6), key=lambda i: transformed[list(faces[i]), 2].mean()):
            coords = points[list(faces[idx])]
            p.setBrush(QColor(colors[idx]))
            p.setPen(QPen(QColor('#a8b0b8'), 1.2))
            p.drawPolygon(QPolygonF([QPointF(float(x), float(y)) for x, y in coords]))

        # Body axes rotate with telemetry and therefore make 3-D motion obvious.
        origin3 = np.array([[0., 0., 0.]])
        origin2 = self._project(origin3, scale, center)[0]
        for i, color in enumerate(AXIS_COLORS):
            endpoint3 = np.array([(view @ rot @ (np.eye(3)[i] * 2.0))])
            endpoint = self._project(endpoint3, scale, center)[0]
            p.setPen(QPen(QColor(color), 2.2))
            p.drawLine(QPointF(*origin2), QPointF(*endpoint))
            p.drawText(QPointF(endpoint[0] + 5, endpoint[1] - 3), ('+X Roll', '+Y Pitch', '+Z Yaw')[i])

        p.setPen(QColor('#8f8f8f'))
        p.setFont(QFont('Segoe UI', 8))
        p.drawText(10, h - 22, 'TELEMETRY-DRIVEN 3D ATTITUDE')
        p.drawText(10, h - 8, 'SIM: drag = Pitch/Yaw disturbance · Shift+drag = Roll')
        p.end()


class CameraView(QWidget):
    target_clicked = pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(190)
        self.setCursor(Qt.CrossCursor)
        self.setToolTip('Simulation: click anywhere in the image to place the target point there.')
        self._image: QImage | None = None
        self.uv: tuple[float, float] | None = None
        self.detected = False

    def set_frame(self, frame: np.ndarray | None, uv: tuple[float, float] | None) -> None:
        if frame is None:
            self._image = None
        else:
            data = np.ascontiguousarray(frame, dtype=np.uint8)
            self._image = QImage(data.data, data.shape[1], data.shape[0], data.strides[0],
                                 QImage.Format_Grayscale8).copy()
        self.uv, self.detected = uv, uv is not None
        self.update()

    def _image_rect(self) -> tuple[QRectF | None, float]:
        if self._image is None:
            return None, 1.0
        iw, ih = self._image.width(), self._image.height()
        scale = min((self.width() - 16) / iw, (self.height() - 8) / ih)
        dw, dh = iw * scale, ih * scale
        left, top = (self.width() - dw) / 2, (self.height() - dh) / 2
        return QRectF(left, top, dw, dh), scale

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton and self._image is not None:
            rect, scale = self._image_rect()
            if rect is not None and rect.contains(QPointF(event.x(), event.y())):
                u = (event.x() - rect.left()) / scale
                v = (event.y() - rect.top()) / scale
                u = float(np.clip(u, 0, self._image.width() - 1))
                v = float(np.clip(v, 0, self._image.height() - 1))
                self.target_clicked.emit((u, v))
                event.accept()
                return
        super().mousePressEvent(event)

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#030303'))
        if self._image is None:
            p.setPen(QColor('#9a9a9a'))
            p.drawText(self.rect(), Qt.AlignCenter, 'NO CAMERA FRAME')
            p.end()
            return
        rect, scale = self._image_rect()
        assert rect is not None
        left, top, dw, dh = rect.left(), rect.top(), rect.width(), rect.height()
        p.drawImage(rect, self._image)
        cx, cy = left + (self._image.width() - 1) / 2 * scale, top + (self._image.height() - 1) / 2 * scale
        p.setPen(QPen(QColor('#686868'), 1, Qt.DashLine))
        p.drawLine(QPointF(cx, top + 7), QPointF(cx, top + dh - 7))
        p.drawLine(QPointF(left + 7, cy), QPointF(left + dw - 7, cy))
        if self.uv is not None:
            x, y = left + self.uv[0] * scale, top + self.uv[1] * scale
            p.setPen(QPen(QColor('#39e36d'), 1.5))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(x, y), 12, 12)
            p.drawLine(QPointF(x - 17, y), QPointF(x + 17, y))
            p.drawLine(QPointF(x, y - 17), QPointF(x, y + 17))
        p.setPen(QColor('#8a8a8a'))
        p.setFont(QFont('Segoe UI', 8))
        p.drawText(10, 17, 'SYNTHETIC CAMERA  /  320 x 240')
        p.drawText(10, 32, 'CLICK IMAGE TO PLACE TARGET')
        p.end()
