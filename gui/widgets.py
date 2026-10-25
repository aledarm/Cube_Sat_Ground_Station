"""Reusable view-only widgets. No control law or communication in this file."""
from __future__ import annotations
import numpy as np
from PyQt5.QtCore import Qt, QPointF, QRectF
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
    """A 3-D model projected with QPainter. No OpenGL/GPU dependency.

    This is an attitude drawing, not a physics engine. Rotation convention:
    Rz(yaw) Ry(pitch) Rx(roll). The simulation uses the same declared convention.
    """
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(190)
        self.attitude = np.zeros(3)
        self._vertices = np.array([[-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
                                   [-1,-1,1], [1,-1,1], [1,1,1], [-1,1,1]], dtype=float)

    def set_attitude(self, attitude: np.ndarray) -> None:
        self.attitude = np.asarray(attitude).copy()
        self.update()

    @staticmethod
    def matrix(a: np.ndarray) -> np.ndarray:
        r, p, y = np.deg2rad(a)
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#050505'))
        w, h = self.width(), self.height()
        p.setPen(QPen(QColor('#202020'), 1))
        for x in range(0, w, 26):
            p.drawLine(x, 0, x, h)
        for y in range(0, h, 26):
            p.drawLine(0, y, w, y)
        view = np.array([[.707, -.707, 0.], [.408, .408, -.816], [.577, .577, .577]])
        rot = self.matrix(self.attitude)
        transformed = (view @ rot @ self._vertices.T).T
        scale = min(w / 5.4, h / 4.4)
        center = np.array([w * .50, h * .52])
        points = transformed[:, :2] * scale + center
        faces = [(0,1,2,3), (4,5,6,7), (0,1,5,4), (1,2,6,5), (2,3,7,6), (3,0,4,7)]
        colors = ['#151a22', '#1c2530', '#111820', '#202833', '#171d25', '#0f151c']
        for idx in sorted(range(6), key=lambda i: transformed[list(faces[i]), 2].mean()):
            coords = points[list(faces[idx])]
            p.setBrush(QColor(colors[idx]))
            p.setPen(QPen(QColor('#9aa8b8'), 1.2))
            p.drawPolygon(QPolygonF([QPointF(float(x), float(y)) for x, y in coords]))
        for i, color in enumerate(AXIS_COLORS):
            endpoint = (view @ rot @ (np.eye(3)[i] * 1.9))[:2] * scale + center
            p.setPen(QPen(QColor(color), 2.2))
            p.drawLine(QPointF(*center), QPointF(*endpoint))
            p.drawText(QPointF(endpoint[0] + 5, endpoint[1] - 3), ('X / Roll', 'Y / Pitch', 'Z / Yaw')[i])
        p.setPen(QColor('#9a9a9a'))
        p.setFont(QFont('Segoe UI', 8))
        p.drawText(10, h - 9, 'ATTITUDE ONLY  /  schematically projected 3D')
        p.end()


class CameraView(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(190)
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

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#030303'))
        if self._image is None:
            p.setPen(QColor('#9a9a9a'))
            p.drawText(self.rect(), Qt.AlignCenter, 'NO CAMERA FRAME')
            p.end()
            return
        iw, ih = self._image.width(), self._image.height()
        scale = min((self.width()-16)/iw, (self.height()-8)/ih)
        dw, dh = iw*scale, ih*scale
        left, top = (self.width()-dw)/2, (self.height()-dh)/2
        dest = QRectF(left, top, dw, dh)
        p.drawImage(dest, self._image)
        cx, cy = left+(iw-1)/2*scale, top+(ih-1)/2*scale
        p.setPen(QPen(QColor('#686868'), 1, Qt.DashLine))
        p.drawLine(QPointF(cx, top+7), QPointF(cx, top+dh-7))
        p.drawLine(QPointF(left+7, cy), QPointF(left+dw-7, cy))
        if self.uv is not None:
            x, y = left+self.uv[0]*scale, top+self.uv[1]*scale
            p.setPen(QPen(QColor('#39e36d'), 1.5))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(x, y), 12, 12)
            p.drawLine(QPointF(x-17, y), QPointF(x+17, y))
            p.drawLine(QPointF(x, y-17), QPointF(x, y+17))
        p.setPen(QColor('#8a8a8a'))
        p.setFont(QFont('Segoe UI', 8))
        p.drawText(10, 17, 'SYNTHETIC CAMERA  /  320 x 240')
        p.end()
