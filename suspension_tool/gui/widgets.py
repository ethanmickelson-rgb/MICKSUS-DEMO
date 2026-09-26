"""Shared input widgets.

`CommitSpin` exists because a live-updating spin box is actively
dangerous on this tool. Every keystroke used to re-solve the linkage, so
clicking into a field showing `45`, forgetting to clear it, and typing
`120` walked the model through 45 → 451 → 4512 → 45123 and threw a link
off to infinity before the intended number was even finished. The
intermediate poses are meaningless and some of them do not solve.

Two changes fix it, and they are complementary:

  * KEYBOARD TRACKING OFF — `valueChanged` waits for Enter, Tab, focus
    loss, or the stepper. Nothing recomputes mid-word. Note that
    `value()` returns the last COMMITTED value while text is pending,
    which is what makes `commit_spins()` below necessary.
  * SELECT-ALL ON THE CLICK THAT FOCUSES — so typing REPLACES the old
    number instead of appending to it, which is the actual root cause.
    A second click inside an already-focused box behaves normally, so
    editing one digit still works.

Escape abandons a pending edit and puts the committed value back.

The seed and optimizer panels deliberately keep their apply-on-button
behaviour; this changes nothing there beyond the select-all, because
those panels never read a spin until their button is pressed.
"""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QAbstractSpinBox, QDoubleSpinBox, QSpinBox


class _CommitMixin:
    """Deferred-commit behaviour, shared by the float and int spins."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setKeyboardTracking(False)
        self._select_on_click = False

    # -- select-all on the click that brings focus in ------------------
    def focusInEvent(self, event):
        self._select_on_click = (
            event.reason() == Qt.FocusReason.MouseFocusReason)
        super().focusInEvent(event)

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        if self._select_on_click:
            self._select_on_click = False
            self.selectAll()

    # -- Escape abandons a pending edit --------------------------------
    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.revert()
            event.accept()
            return
        super().keyPressEvent(event)

    # -- public handles on Qt's protected text plumbing ----------------
    def commit(self) -> None:
        """Interpret whatever is typed right now, as Enter would.

        `interpretText` is protected in Qt, so it can only be reached
        from inside a subclass — hence this being a method rather than a
        free function.
        """
        self.interpretText()

    def revert(self) -> None:
        """Throw away a pending edit and redisplay the committed value."""
        edit = self.lineEdit()
        if edit is not None:
            edit.setText(f"{self.prefix()}"
                         f"{self.textFromValue(self.value())}"
                         f"{self.suffix()}")
        self.selectAll()


class CommitSpin(_CommitMixin, QDoubleSpinBox):
    """A QDoubleSpinBox that does not fire until you mean it."""


class CommitIntSpin(_CommitMixin, QSpinBox):
    """The integer twin, for counts and resolutions."""


def commit_spins(widget) -> None:
    """Force every pending spin-box edit under `widget` to commit.

    Focus loss normally does this when you click a button, but a button
    with a NoFocus policy never takes focus, and then a value that was
    typed but not confirmed would be read stale. Call this first in
    anything that reads spin values in response to a click — the seed
    generator and the optimizer both do.
    """
    for spin in widget.findChildren(QAbstractSpinBox):
        if isinstance(spin, _CommitMixin):
            spin.commit()
