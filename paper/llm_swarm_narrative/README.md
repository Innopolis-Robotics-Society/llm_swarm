# Correctness Boundaries — the narrative version of the article

This is the article. It is the only manuscript in the repository; the earlier
two-column draft was removed. Editorial revision following review, 12 October
2026, using the official Springer Nature `sn-jnl` class, single column, numeric
citations. The submission venue is still open.

`main.tex` is the source, `main.pdf` the built version: 18 pages, 18 cited
sources (19 entries in `refs.bib`). No new experiments. The numbers of both
original experimental tables are preserved.

The central thesis and the title were changed, three RQs were added, along with
a connected review of mechanisms and tables for check coverage and provenance.
Carry success was made precise: a dwell at the pickup first, then at the
drop-off; physical transport of the object is not a measured condition. Failure
cases were moved to the centre of Results. Unknown parameters and accounting
problems are stated outright.

## Build

Everything the main build needs is in this directory. A normal TeX Live with
the dependencies of the official class is enough.

```bash
pdflatex -interaction=nonstopmode -halt-on-error main.tex
bibtex main
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
```

The figures are included as finished PDFs. To rebuild them:

```bash
python3 mkhero.py
python3 mkmap.py
pdflatex -interaction=nonstopmode -halt-on-error architecture.tex
```

The Python scripts use Matplotlib, NumPy, Pillow and PyMuPDF. They are not
needed to build the finished manuscript. The map geometry was recovered from
the embedded occupancy bitmap and the original task coordinates; the legend was
moved below the figure.

## Status

`REVIEW_RESPONSE.md` describes the changes made and the limitations that
remain. `AUTHOR_CHECKS.md` lists the specific facts that still have to be
confirmed against existing material. These are private working notes, not part
of the submitted article.

The title page keeps "Author details pending": the full author list,
affiliations and corresponding-author email were not provided. Funding and
competing-interest declarations were not invented. The simulation code and the
original experimental journals are not part of this package; they live in
`../results/`.

The stock class and `.bst` are unmodified. The official class manual is not
vendored here — it is at
https://www.springernature.com/gp/authors/campaigns/latex-author-support

## Layout pass, 13 October 2026

Heading and orphan-line breaking, figure and table placement, keeping the
conclusion on one page and preventing a single bibliography entry from being
split were all configured. The Springer class is unmodified. The text and the
content of every figure and table were checked against the previous version;
the changes are layout only. The PDF was checked page by page; the build has no
warnings, no undefined references and no overfull lines.

## Narrative edit, 16 October 2026

Related Work was rewritten as a connected discussion of the transition from
path planning to command interpretation, feedback, team coordination and
execution checks. Four internal subheadings and the redundant table listing
methods were removed. The introduction ties the research questions to the
stages a plan passes through; the discussion is written without three separate
RQ subheadings. The Nav2 reference was moved to the description of execution.

Abstract, methods, experimental design, results, limitations, conclusion,
numeric values, illustrations and `refs.bib` were not changed. Table and
reference numbering was updated at build time. Primary publications and author
pages were used; the organising reference for the argument is Gerkey and
Mataric, IJRR 2004, without borrowing its wording.
