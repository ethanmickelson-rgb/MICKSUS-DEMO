"""Design candidates — named whole-vehicle snapshots you can compare in
a table and hop between.

One candidate = front + rear hardpoints + vehicle parameters at the
moment you hit "Snapshot", plus the headline metrics (computed then,
frozen with it). Double-click the Name cell to rename. "Load" restores
the whole design (the current state is NOT auto-saved first — snapshot
before you hop if you care). Candidates live inside the .MICK file.
"""

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QGroupBox, QHBoxLayout, QPushButton,
                               QTableWidget, QTableWidgetItem, QVBoxLayout)

from ..units import Unit

# (metric key, column label, format kind): "len" converts mm->display,
# "ang" raw deg, "rate" deg/mm, "text" preformatted string.
COLUMNS = [
    ("rc_f", "RC F", "len"),
    ("rc_r", "RC R", "len"),
    ("camber_gain_f", "Cam gain F (deg/mm)", "rate"),
    ("bump_steer_f", "Bump steer F", "rate"),
    ("caster_f", "Caster F", "ang"),
    ("travel_f", "Travel F", "text"),
    ("travel_r", "Travel R", "text"),
    ("cv_worst", "CV worst", "ang"),
]


class CandidatesPanel(QGroupBox):
    changed = Signal()          # candidates edited/renamed/deleted
    snapshot_requested = Signal()
    load_requested = Signal(int)

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Design candidates", parent)
        self._unit = unit
        self._items: list[dict] = []
        self._refreshing = False
        lay = QVBoxLayout(self)
        self.table = QTableWidget(0, 1 + len(COLUMNS))
        self.table.setAlternatingRowColors(True)
        self.table.setHorizontalHeaderLabels(
            ["Name"] + [label for _, label, _ in COLUMNS])
        self.table.itemChanged.connect(self._on_item_changed)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        lay.addWidget(self.table)
        row = QHBoxLayout()
        snap = QPushButton("Snapshot current design")
        snap.clicked.connect(self.snapshot_requested)
        load = QPushButton("Load selected")
        load.clicked.connect(self._emit_load)
        delete = QPushButton("Delete selected")
        delete.clicked.connect(self._delete_selected)
        for b in (snap, load, delete):
            row.addWidget(b)
        lay.addLayout(row)

    # ------------------------------------------------------------------
    def set_units(self, unit: Unit) -> None:
        self._unit = unit
        self.refresh()

    def candidates(self) -> list:
        return self._items

    def load_list(self, items: list | None) -> None:
        self._items = list(items or [])
        self.refresh()

    def add_candidate(self, cand: dict) -> None:
        self._items.append(cand)
        self.refresh()
        self.changed.emit()

    # ------------------------------------------------------------------
    def refresh(self) -> None:
        self._refreshing = True
        u = self._unit
        self.table.setRowCount(len(self._items))
        for r, cand in enumerate(self._items):
            name_item = QTableWidgetItem(cand.get("name", f"candidate {r+1}"))
            self.table.setItem(r, 0, name_item)
            m = cand.get("metrics", {})
            for c, (key, _, kind) in enumerate(COLUMNS, start=1):
                v = m.get(key)
                if v is None:
                    text = "-"
                elif kind == "len":
                    text = f"{u.from_mm(v):+.{u.decimals}f} {u.label}"
                elif kind == "ang":
                    text = f"{v:+.2f} deg" if np.isfinite(v) else "-"
                elif kind == "rate":
                    text = f"{v:+.4f}" if np.isfinite(v) else "-"
                else:
                    text = str(v)
                item = QTableWidgetItem(text)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(r, c, item)
        self._refreshing = False

    def _on_item_changed(self, item) -> None:
        if self._refreshing or item.column() != 0:
            return
        r = item.row()
        if 0 <= r < len(self._items):
            self._items[r]["name"] = item.text()
            self.changed.emit()

    def _selected_row(self) -> int:
        rows = {i.row() for i in self.table.selectedIndexes()}
        return min(rows) if rows else -1

    def _emit_load(self) -> None:
        r = self._selected_row()
        if 0 <= r < len(self._items):
            self.load_requested.emit(r)

    def _delete_selected(self) -> None:
        r = self._selected_row()
        if 0 <= r < len(self._items):
            del self._items[r]
            self.refresh()
            self.changed.emit()
