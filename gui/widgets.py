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
    """Lightweight solid 3-D CubeSat model driven by telemetry.

    The renderer deliberately stays in QPainter so the Ground Station does not
    need OpenGL or an external 3-D asset. The result is a technical low-poly
    model: solid body panels, structural rails, reaction-wheel hubs and antennas.
    Dragging is a *physical simulator disturbance*, not a view-only rotation.
    """
    disturbance_requested = pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(300)
        self.setMouseTracking(True)
        self.setCursor(Qt.OpenHandCursor)
        self.setToolTip('Simulation: drag = Pitch/Yaw disturbance; Shift + drag = Roll.')
        self.attitude = np.zeros(3)
        self.controlled = False
        self.show_axes = True
        self.show_grid = True
        self.show_antennas = True
        self._drag_pos = None
        self._vertices = np.array([
            [-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
            [-1,-1,1],  [1,-1,1],  [1,1,1],  [-1,1,1]
        ], dtype=float)
        self._faces = [(0,1,2,3), (4,5,6,7), (0,1,5,4),
                       (1,2,6,5), (2,3,7,6), (3,0,4,7)]
        self._edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
                       (0,4),(1,5),(2,6),(3,7)]

    def set_attitude(self, attitude: np.ndarray) -> None:
        self.attitude = np.asarray(attitude, dtype=float).copy()
        self.update()

    def set_control_state(self, controlled: bool) -> None:
        self.controlled = bool(controlled)
        self.update()

    def set_show_axes(self, enabled: bool) -> None:
        self.show_axes = bool(enabled); self.update()

    def set_show_grid(self, enabled: bool) -> None:
        self.show_grid = bool(enabled); self.update()

    def set_show_antennas(self, enabled: bool) -> None:
        self.show_antennas = bool(enabled); self.update()

    @staticmethod
    def matrix(a: np.ndarray) -> np.ndarray:
        r, p, y = np.deg2rad(a)
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])

    @staticmethod
    def _view_matrix() -> np.ndarray:
        # Fixed isometric-like camera. Only spacecraft attitude changes.
        return np.array([[.707, -.707, 0.], [.408, .408, -.816], [.577, .577, .577]])

    def _project(self, xyz: np.ndarray, scale: float, center: np.ndarray) -> np.ndarray:
        z = xyz[:, 2] + 6.2
        perspective = 5.35 / np.maximum(z, 1.0)
        return xyz[:, :2] * perspective[:, None] * scale + center

    @staticmethod
    def _poly(points: np.ndarray) -> QPolygonF:
        return QPolygonF([QPointF(float(x), float(y)) for x, y in points])

    @staticmethod
    def _inset_quad(quad: np.ndarray, amount: float = .11) -> np.ndarray:
        c = quad.mean(axis=0)
        return c + (quad - c) * (1.0 - amount)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._drag_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept(); return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._drag_pos is None or not (event.buttons() & Qt.LeftButton):
            super().mouseMoveEvent(event); return
        delta = event.pos() - self._drag_pos
        self._drag_pos = event.pos()
        dx = float(np.clip(delta.x() * 0.12, -5., 5.))
        dy = float(np.clip(delta.y() * 0.12, -5., 5.))
        disturbance = np.array([dx, 0., 0.]) if event.modifiers() & Qt.ShiftModifier else np.array([0., -dy, dx])
        if np.linalg.norm(disturbance) > 0.02:
            self.disturbance_requested.emit(disturbance.tolist())
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._drag_pos = None
            self.setCursor(Qt.OpenHandCursor)
            event.accept(); return
        super().mouseReleaseEvent(event)

    def _draw_panel_grid(self, p: QPainter, quad: np.ndarray) -> None:
        # 4x4 solar/electronics panel pattern; cheap bilinear interpolation.
        a, b, c, d = quad
        p.setPen(QPen(QColor('#31435a'), 0.75))
        for t in (0.25, 0.5, 0.75):
            p.drawLine(QPointF(*(a*(1-t) + d*t)), QPointF(*(b*(1-t) + c*t)))
            p.drawLine(QPointF(*(a*(1-t) + b*t)), QPointF(*(d*(1-t) + c*t)))

    def _draw_background(self, p: QPainter, w: int, h: int) -> None:
        p.fillRect(self.rect(), QColor('#05080d'))
        if not self.show_grid:
            return
        p.setPen(QPen(QColor('#101923'), 1))
        horizon = int(h * .73)
        for x in range(-w, w * 2, 48):
            p.drawLine(w // 2, horizon, x, h)
        for i in range(7):
            yy = horizon + int((h - horizon) * (i / 7) ** 1.55)
            p.drawLine(0, yy, w, yy)
        # sparse technical stars/dots, deterministic and intentionally non-realistic
        p.setPen(QPen(QColor('#263544'), 1))
        for fx, fy in ((.08,.15),(.18,.27),(.31,.12),(.42,.22),(.61,.13),(.76,.25),(.88,.17),(.93,.37),(.13,.43)):
            p.drawPoint(int(w*fx), int(h*fy))

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        self._draw_background(p, w, h)

        view = self._view_matrix()
        rot = self.matrix(self.attitude)
        transformed = (view @ rot @ self._vertices.T).T
        scale = min(w / 4.25, h / 3.75)
        center = np.array([w * .48, h * .46])
        points = self._project(transformed, scale, center)

        # Dark blue/graphite body panels with a lighter structural chassis.
        face_colors = ['#0c1420', '#19263a', '#101a28', '#17263d', '#0d1724', '#142135']
        face_depth = [transformed[list(face), 2].mean() for face in self._faces]
        order = sorted(range(6), key=lambda i: face_depth[i])
        for idx in order:
            ids = list(self._faces[idx])
            quad = points[ids]
            p.setPen(QPen(QColor('#4b5868'), 1.1))
            p.setBrush(QBrush(QColor(face_colors[idx])))
            p.drawPolygon(self._poly(quad))

            inner = self._inset_quad(quad, .12)
            p.setPen(QPen(QColor('#28394f'), 1.0))
            p.setBrush(QBrush(QColor('#0b1627' if idx % 2 else '#0d1a2d')))
            p.drawPolygon(self._poly(inner))
            self._draw_panel_grid(p, inner)

        # Structural rails. Two strokes provide a simple metallic low-poly look.
        p.setPen(QPen(QColor('#414a55'), 8.0, Qt.SolidLine, Qt.RoundCap))
        for a, b in self._edges:
            p.drawLine(QPointF(*points[a]), QPointF(*points[b]))
        p.setPen(QPen(QColor('#aeb6bf'), 4.2, Qt.SolidLine, Qt.RoundCap))
        for a, b in self._edges:
            p.drawLine(QPointF(*points[a]), QPointF(*points[b]))

        # Corner blocks/fasteners.
        p.setPen(QPen(QColor('#20262d'), 1.0))
        p.setBrush(QColor('#747d87'))
        for x, y in points:
            p.drawRoundedRect(QRectF(float(x)-5.3, float(y)-5.3, 10.6, 10.6), 2, 2)
            p.setBrush(QColor('#20262d'))
            p.drawEllipse(QPointF(float(x), float(y)), 1.7, 1.7)
            p.setBrush(QColor('#747d87'))

        # Three reaction-wheel/motor hubs on the positive body faces.
        hubs_body = np.array([[1.03,0,0], [0,1.03,0], [0,0,1.03]], dtype=float)
        hubs_t = (view @ rot @ hubs_body.T).T
        hubs = self._project(hubs_t, scale, center)
        for i, (x, y) in enumerate(hubs):
            p.setPen(QPen(QColor('#11161c'), 5.0))
            p.setBrush(QColor('#303841'))
            p.drawEllipse(QPointF(float(x), float(y)), 10.0, 10.0)
            p.setPen(QPen(QColor(AXIS_COLORS[i]), 2.0))
            p.setBrush(QColor('#11161c'))
            p.drawEllipse(QPointF(float(x), float(y)), 5.0, 5.0)

        # Thin antennas keep the same schematic/engineering style as the mockup.
        if self.show_antennas:
            rods = [(6, np.array([1.65,1.65,1.65])), (5, np.array([1.65,-1.65,1.65])),
                    (2, np.array([1.65,1.65,-1.65])), (7, np.array([-1.65,1.65,1.65]))]
            p.setPen(QPen(QColor('#c59045'), 2.2, Qt.SolidLine, Qt.RoundCap))
            for vid, tip_body in rods:
                tip_t = np.array([(view @ rot @ tip_body)])
                tip = self._project(tip_t, scale, center)[0]
                p.drawLine(QPointF(*points[vid]), QPointF(*tip))

        # Body axes rotate with the actual telemetry attitude.
        if self.show_axes:
            origin2 = self._project(np.array([[0.,0.,0.]]), scale, center)[0]
            labels = ('+X', '+Y', '+Z')
            for i, color in enumerate(AXIS_COLORS):
                endpoint3 = np.array([(view @ rot @ (np.eye(3)[i] * 2.25))])
                endpoint = self._project(endpoint3, scale, center)[0]
                p.setPen(QPen(QColor(color), 3.0, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(QPointF(*origin2), QPointF(*endpoint))
                p.setBrush(QColor(color))
                vec = endpoint - origin2
                n = np.linalg.norm(vec) or 1.0
                u = vec / n; side = np.array([-u[1], u[0]])
                tri = np.array([endpoint, endpoint-u*12+side*5, endpoint-u*12-side*5])
                p.drawPolygon(self._poly(tri))
                p.setFont(QFont('Segoe UI', 9, QFont.DemiBold))
                p.drawText(QPointF(float(endpoint[0]+6), float(endpoint[1]-5)), labels[i])

        # Bottom status strip mirrors the simulator behaviour rather than decoration.
        p.setFont(QFont('Segoe UI', 8))
        state = 'ACTIVE STABILIZATION' if self.controlled else 'FREE DRIFT — REACTION WHEELS NOT CONTROLLING'
        p.setPen(QColor('#4ee58a' if self.controlled else '#ffd24a'))
        p.drawText(12, h - 12, state)
        p.setPen(QColor('#7d8792'))
        p.drawText(w - 250, h - 12, 'drag = physical disturbance')
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
