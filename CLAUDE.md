# Project instructions for Claude

## Standing rule: ask before you start

Before beginning ANY substantive task in this repo, ask the clarifying
questions you believe are important to completing it well — design
intent, conventions, scope, trade-offs. Ethan explicitly prefers
answering a few questions up front over discovering baked-in
assumptions later. Use the question tool when available; otherwise ask
in plain text and wait. Skip only when the task is genuinely
unambiguous or Ethan says to proceed without questions.

## Domain knowledge

The upstream repo carries a `baja-suspension-design` skill holding the
team's vehicle data and the conventions distilled from building this
tool. It is NOT in this public demo branch, because it is the team's
data. The conventions it pins that you still need are in Quick facts
below, in `suspension_tool/equations.py` (every formula with its
Milliken / Gillespie citation), and in the module docstrings.

## Quick facts

- App: `python -m suspension_tool.gui` · tests: `python -m unittest`
  · GUI smoke: `QT_QPA_PLATFORM=offscreen python -m tests.smoke_gui`
- Internal math: +X fwd, +Y left, +Z up, mm, left corner. Display &
  export default to the CHASSIS convention (+Y fwd) — see
  `suspension_tool/axes.py`.
- Every release: bump `suspension_tool/__init__.py:__version__`, update
  the README status table and `gui/help_text.py`, run the full unit
  suite AND the GUI smoke test, extend both for new features.
- Physics changes need verification against hand calculations or the
  team's real data (see `tests/` for the established patterns).
- The offscreen sandbox GL intermittently aborts on Qt's first paint —
  known environment flake; deterministic failures are real bugs.
  `LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe` stabilizes it
  (verified 2026-07): prepend to any offscreen GUI run.
