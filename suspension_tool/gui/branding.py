"""MICKSUS branding: name, splash screen, app icon, dark theme.

Everything is drawn programmatically (no bundled image assets) so the
splash/icon always match the version and survive packaging.
"""

from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import (QBrush, QColor, QFont, QIcon, QLinearGradient,
                           QPainter, QPalette, QPen, QPixmap,
                           QRadialGradient)

APP_NAME = "MICKSUS"
TAGLINE = "Baja Suspension Kinematics & Synthesis"

# Front-view line art of a double-wishbone corner (stylized, unitless
# coordinates in a 100x100 box; x = lateral, y = down-positive screen).
_CHASSIS = [(8.0, 30.0), (8.0, 74.0)]
_UCA = [(10.0, 38.0), (62.0, 34.0)]
_LCA = [(10.0, 68.0), (70.0, 72.0)]
_KINGPIN = [(62.0, 34.0), (70.0, 72.0)]
_SHOCK = [(14.0, 34.0), (52.0, 70.0)]
_TIRE_C = (79.0, 53.0)
_TIRE_R = 26.0


def _draw_linkage(p: QPainter, rect: QRectF, alpha: int = 255) -> None:
    """Draw the corner art scaled into `rect`."""
    sx = rect.width() / 100.0
    sy = rect.height() / 100.0

    def pt(xy):
        return QPointF(rect.left() + xy[0] * sx, rect.top() + xy[1] * sy)

    def line(seg, color, w):
        c = QColor(color)
        c.setAlpha(alpha)
        p.setPen(QPen(c, w, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(pt(seg[0]), pt(seg[1]))

    # tire first (behind the links)
    tire = QColor("#3c4148")
    tire.setAlpha(min(alpha, 210))
    p.setPen(QPen(tire, 3.2 * sx))
    p.setBrush(Qt.NoBrush)
    p.drawEllipse(pt(_TIRE_C), _TIRE_R * sx, _TIRE_R * sy)
    line(_CHASSIS, "#6a7280", 4.0 * sx)
    line(_UCA, "#4f8fd0", 3.2 * sx)
    line(_LCA, "#4f8fd0", 3.2 * sx)
    line(_KINGPIN, "#c8ccd4", 2.6 * sx)
    line(_SHOCK, "#d05050", 2.6 * sx)
    ball = QColor("#e8ecf2")
    ball.setAlpha(alpha)
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(ball))
    for xy in (_UCA[1], _LCA[1], _SHOCK[1], _UCA[0], _LCA[0]):
        p.drawEllipse(pt(xy), 2.2 * sx, 2.2 * sy)


def splash_pixmap(version: str) -> QPixmap:
    """The boot splash: dark gradient, big wordmark, corner line art."""
    w, h = 680, 420
    pm = QPixmap(w, h)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)

    g = QLinearGradient(0, 0, w, h)
    g.setColorAt(0.0, QColor("#14161c"))
    g.setColorAt(0.55, QColor("#1b1f29"))
    g.setColorAt(1.0, QColor("#232a38"))
    p.setBrush(QBrush(g))
    p.setPen(QPen(QColor("#3a4356"), 2))
    p.drawRoundedRect(QRectF(1, 1, w - 2, h - 2), 14, 14)

    # soft headlight glow behind the art
    glow = QRadialGradient(QPointF(w * 0.74, h * 0.55), w * 0.35)
    glow.setColorAt(0.0, QColor(80, 120, 190, 60))
    glow.setColorAt(1.0, QColor(0, 0, 0, 0))
    p.setBrush(QBrush(glow))
    p.setPen(Qt.NoPen)
    p.drawRect(0, 0, w, h)

    # faint linkage art on the right, clear of the wordmark/tagline
    _draw_linkage(p, QRectF(w * 0.52, h * 0.20, w * 0.46, h * 0.64),
                  alpha=170)

    title = QFont("Arial", 44, QFont.Black)
    title.setLetterSpacing(QFont.AbsoluteSpacing, 4.0)
    p.setFont(title)
    p.setPen(QColor("#f0f3f8"))
    p.drawText(QRectF(40, 96, w - 80, 80), Qt.AlignLeft, APP_NAME)
    p.setPen(QColor("#7fa8d8"))
    p.setFont(QFont("Arial", 13))
    p.drawText(QRectF(43, 176, w - 86, 30), Qt.AlignLeft, TAGLINE)
    p.setPen(QColor("#8a93a6"))
    p.setFont(QFont("Arial", 10))
    p.drawText(QRectF(43, h - 46, w - 86, 24), Qt.AlignLeft,
               f"v{version}  ·  seed → drag → checklist → optimize → "
               "export")
    p.end()
    return pm


def app_icon() -> QIcon:
    """Window/taskbar icon: the corner art on a dark rounded tile."""
    icon = QIcon()
    for size in (32, 64, 128, 256):
        pm = QPixmap(size, size)
        pm.fill(Qt.transparent)
        p = QPainter(pm)
        p.setRenderHint(QPainter.Antialiasing)
        g = QLinearGradient(0, 0, size, size)
        g.setColorAt(0.0, QColor("#1b1f29"))
        g.setColorAt(1.0, QColor("#2a3142"))
        p.setBrush(QBrush(g))
        p.setPen(QPen(QColor("#4f8fd0"), max(1.0, size / 40.0)))
        r = size * 0.04
        p.drawRoundedRect(QRectF(r, r, size - 2 * r, size - 2 * r),
                          size * 0.18, size * 0.18)
        _draw_linkage(p, QRectF(size * 0.10, size * 0.14, size * 0.80,
                                size * 0.72))
        p.end()
        icon.addPixmap(pm)
    return icon


def apply_dark_theme(app) -> None:
    """App-wide dark Fusion palette (the 3D viewport is already dark)."""
    app.setStyle("Fusion")
    pal = QPalette()
    bg = QColor("#232629")
    base = QColor("#1b1e21")
    text = QColor("#dde1e6")
    dim = QColor("#8a93a0")
    hi = QColor("#3d6fa8")
    pal.setColor(QPalette.Window, bg)
    pal.setColor(QPalette.WindowText, text)
    pal.setColor(QPalette.Base, base)
    pal.setColor(QPalette.AlternateBase, QColor("#24282c"))
    pal.setColor(QPalette.ToolTipBase, QColor("#2c313a"))
    pal.setColor(QPalette.ToolTipText, text)
    pal.setColor(QPalette.Text, text)
    pal.setColor(QPalette.PlaceholderText, dim)
    pal.setColor(QPalette.Button, QColor("#2b2f34"))
    pal.setColor(QPalette.ButtonText, text)
    pal.setColor(QPalette.BrightText, QColor("#ff6b6b"))
    pal.setColor(QPalette.Link, QColor("#6ba3e0"))
    pal.setColor(QPalette.Highlight, hi)
    pal.setColor(QPalette.HighlightedText, QColor("#ffffff"))
    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        pal.setColor(QPalette.Disabled, role, dim)
    app.setPalette(pal)
