"""Render the printable equation sheet from `suspension_tool.equations`.

The GUI's Help > Equations window renders the SAME data live, so the two
can never disagree — regenerate the PDF whenever equations.py changes:

    python tools/build_equation_sheet.py suspension_tool/docs/Equation_Sheet.pdf

Needs reportlab (build-time only; the app itself does not import it).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from reportlab.lib import colors                                   # noqa: E402
from reportlab.lib.pagesizes import LETTER                         # noqa: E402
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet  # noqa: E402
from reportlab.lib.units import inch                               # noqa: E402
from reportlab.platypus import (BaseDocTemplate, Frame, HRFlowable,  # noqa: E402
                                KeepTogether, PageTemplate, Paragraph,
                                Spacer, Table, TableStyle)

from suspension_tool import __version__                            # noqa: E402
from suspension_tool.equations import (SECTIONS, SYMBOLS,          # noqa: E402
                                       all_equations)

ACCENT = colors.HexColor("#1F3864")
HDRBG = colors.HexColor("#D9E2F3")
ALTBG = colors.HexColor("#F2F5FB")
GRID = colors.HexColor("#B4C6E7")
CODEBG = colors.HexColor("#F4F6FA")

ss = getSampleStyleSheet()
BODY = ParagraphStyle("b", parent=ss["Normal"], fontName="Helvetica",
                      fontSize=7.8, leading=10.2, spaceAfter=2)
TITLE = ParagraphStyle("t", parent=ss["Normal"], fontName="Helvetica-Bold",
                       fontSize=17, leading=20, textColor=ACCENT,
                       spaceAfter=2)
H1 = ParagraphStyle("h1", parent=ss["Normal"], fontName="Helvetica-Bold",
                    fontSize=10.5, leading=12.5, textColor=ACCENT,
                    spaceBefore=7, spaceAfter=3)
NAME = ParagraphStyle("n", parent=BODY, fontName="Helvetica-Bold",
                      fontSize=8.2, spaceBefore=3, spaceAfter=1)
EQ = ParagraphStyle("eq", parent=BODY, fontName="Courier", fontSize=7.6,
                    leading=9.8, leftIndent=8, backColor=CODEBG,
                    borderPadding=2, spaceAfter=1)
SMALL = ParagraphStyle("s", parent=BODY, fontSize=7.0, leading=8.8,
                       leftIndent=8, textColor=colors.HexColor("#333333"))
MOD = ParagraphStyle("m", parent=SMALL, fontName="Helvetica-Oblique",
                     textColor=colors.HexColor("#808080"))
IT = ParagraphStyle("i", parent=BODY, fontName="Helvetica-Oblique",
                    textColor=colors.HexColor("#404040"))


def esc(t):
    return (str(t).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def build(out_path):
    story = []
    story.append(Paragraph("Equation Reference", TITLE))
    story.append(Paragraph(
        f"<b>Every formula MICKSUS uses, for checking its numbers by "
        f"hand.</b> &nbsp; {len(all_equations())} equations · "
        f"v{__version__} · in-app copy at Help &gt; Equations", IT))
    story.append(HRFlowable(width="100%", thickness=1.1, color=ACCENT,
                            spaceBefore=4, spaceAfter=5))
    story.append(Paragraph(
        "<b>Units.</b> Geometry is millimetres in a right-handed frame — "
        "+X forward, +Y left, +Z up, origin at ground on the centreline, "
        "left corner modelled and mirrored. The dynamics layer is inches, "
        "pounds and seconds, so <b>mass = W / 386.4</b> (g in in/s²). Using "
        "32.2 is the classic factor-of-12 error.", BODY))

    # symbols
    story.append(Paragraph("Symbols", H1))
    data = [[Paragraph("<b>Symbol</b>", SMALL), Paragraph("<b>Meaning</b>",
                                                          SMALL),
             Paragraph("<b>Units</b>", SMALL)]]
    for sym, mean, unit in SYMBOLS:
        data.append([Paragraph(f"<b>{esc(sym)}</b>", SMALL),
                     Paragraph(esc(mean), SMALL),
                     Paragraph(esc(unit), SMALL)])
    t = Table(data, colWidths=[0.85 * inch, 4.4 * inch, 0.8 * inch],
              repeatRows=1)
    st = [("GRID", (0, 0), (-1, -1), 0.4, GRID),
          ("BACKGROUND", (0, 0), (-1, 0), HDRBG),
          ("VALIGN", (0, 0), (-1, -1), "TOP"),
          ("LEFTPADDING", (0, 0), (-1, -1), 4),
          ("RIGHTPADDING", (0, 0), (-1, -1), 4),
          ("TOPPADDING", (0, 0), (-1, -1), 2),
          ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]
    for i in range(2, len(data), 2):
        st.append(("BACKGROUND", (0, i), (-1, i), ALTBG))
    t.setStyle(TableStyle(st))
    story.append(t)

    for sec in SECTIONS:
        story.append(Paragraph(esc(sec.title), H1))
        if sec.blurb:
            story.append(Paragraph(esc(" ".join(sec.blurb.split())), IT))
        for eq in sec.equations:
            block = [Paragraph(esc(eq.name), NAME),
                     Paragraph(esc(eq.expr).replace("\n", "<br/>"), EQ)]
            if eq.where:
                block.append(Paragraph(f"<b>where</b> {esc(eq.where)}",
                                       SMALL))
            if eq.note:
                block.append(Paragraph(f"<i>{esc(eq.note)}</i>", SMALL))
            if eq.module:
                block.append(Paragraph(esc(eq.module), MOD))
            # keep each equation and its notes on one page
            story.append(KeepTogether(block))
        story.append(Spacer(1, 2))

    def footer(canvas, doc):
        canvas.saveState()
        canvas.setFont("Helvetica", 7)
        canvas.setFillColor(colors.HexColor("#707070"))
        canvas.drawString(0.7 * inch, 0.42 * inch,
                          "MICKSUS — Equation Reference")
        canvas.drawRightString(LETTER[0] - 0.7 * inch, 0.42 * inch,
                               "Page %d" % doc.page)
        canvas.restoreState()

    doc = BaseDocTemplate(out_path, pagesize=LETTER,
                          leftMargin=0.7 * inch, rightMargin=0.7 * inch,
                          topMargin=0.6 * inch, bottomMargin=0.6 * inch,
                          title="MICKSUS — Equation Reference",
                          author="MICKSUS")
    doc.addPageTemplates([PageTemplate(
        id="all",
        frames=[Frame(doc.leftMargin, doc.bottomMargin, doc.width, doc.height,
                      id="f")],
        onPage=footer)])
    doc.build(story)
    print(f"wrote {out_path} ({len(all_equations())} equations)")


if __name__ == "__main__":
    target = (sys.argv[1] if len(sys.argv) > 1
              else os.path.join(os.path.dirname(os.path.dirname(
                  os.path.abspath(__file__))), "suspension_tool", "docs",
                  "Equation_Sheet.pdf"))
    build(target)
