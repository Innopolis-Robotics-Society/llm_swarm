# Correctness Boundaries of Typed Plans for Natural-Language Robot Fleet Control

The manuscript was submitted to *Autonomous Intelligent Systems* as a Short
Paper. The public preprint is [arXiv:2611.12654](https://arxiv.org/abs/2611.12654).

`main.tex` is the source and `main.pdf` is the 18-page manuscript. The
analysis uses the existing experiments documented in [`paper/results`](../results).
The recorded rosbags are in the public archive linked in the manuscript's
Availability of Data and Material section.

## Build

Everything the main build needs is in this directory. A normal TeX Live with
the dependencies of the official class is enough.

```bash
./build.sh
```

`build.sh` runs the four passes below and then refuses to leave a broken
`main.pdf` behind: it fails on TeX errors, on unresolved references or
citations, on an unreadable PDF, and on a literal `??` in the rendered text.

```bash
pdflatex -interaction=nonstopmode -halt-on-error main.tex
bibtex main
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
```

All four passes are required. A single `pdflatex` run leaves every `\ref` and
`\cite` as `??`, because the `.aux` is written at the end of a run and read at
the start of the next one. If you see `??` in the PDF, the build was cut short
or two builds ran at once over the same directory; rerun `./build.sh`.

The figures are included as finished PDFs. To rebuild them:

```bash
python3 mkhero.py
python3 mkmap.py
pdflatex -interaction=nonstopmode -halt-on-error -jobname=Fig2 architecture.tex
```

The Python scripts use Matplotlib, NumPy, Pillow and PyMuPDF. They are not
needed to build the finished manuscript. The map geometry was recovered from
the embedded occupancy bitmap and the original task coordinates; the legend was
moved below the figure.

To clean the manuscript build while keeping the figure PDFs, remove only the
files generated from `main.tex`:

```bash
rm -f main.aux main.bbl main.blg main.log main.out main.pdf
```

## Manuscript files

The article uses the official Springer Nature `sn-jnl` class and numerical
bibliography style. The `orcidlink` package draws the clickable ORCID badges.
The three article figures are `Fig1.pdf`, `Fig2.pdf`, and `Fig3.pdf`.

Statements and Declarations include the data and code availability statements,
competing interests, internal funding from Innopolis University, author
contributions, acknowledgements, ethics and consent statements, and a
generative AI disclosure.

## Fact check, 4 November 2026

Every quantitative claim was recomputed from `paper/results` rather than
re-read. E1 was rebuilt from `e1b_summary.json`, `e1ref_summary.json` and
`e2ref_summary.json`; E2 from `tasks.makespan_s` in the eighteen
`derived/metrics.json` files; the mechanism counters from
`analyze_sessions.py`. All table values reproduce exactly.

Six claims did not survive.

**The carry metric is not a dwell.** The manuscript said, in the abstract, in
Section 3.2, in Table 1, in the Discussion, in the Limitations and in the
Conclusion, that a carry task requires a *dwell* condition at the pickup and
then at the drop-off, and that the dwell threshold was undocumented. There is
no dwell anywhere in the implementation: `grep -ri dwell` over the source
returns nothing. `task_manager_node.py` sets `DONE` on the first 5 Hz poll
that finds the robot inside the radius, and `analyze_sessions.py` reads the
resulting status transition out of `/tasks/state`. The condition is a single
presence sample. The text now says so, and the "undocumented threshold"
limitation is replaced by the real one: a robot that crosses a zone without
stopping still completes the task.

**Auxiliary tool calls were not zero.** One B-M3 run (`20260818_113030`) made
three: `list_tasks` once, `verify_plan_execution_state` twice. Twelve of the
thirteen made none.

**The drop-off account is five runs, not six** (two pre-guard, three
post-guard).

**The prompts and responses are in the archive.** Two paragraphs claimed the
reporting package omitted them; `e1_plan_accuracy/raw/*.json` holds all 220
prompt/response pairs per model. What is missing is the serving stack.

**The version registry exists but is useless.** The text said no session-ID or
version mapping accompanied the manuscript. Every run has a `session.json`
with a `git_commit`, but all eighteen were recorded with `git_dirty: true`, so
the commit does not identify the code. That is the accurate limitation.

**"5 of 13 plans carry the defect"** was imprecise: in two of the five the
guard had already removed the extra carrier before the plan was stored, so the
count is of generated plans, not stored ones.

Two smaller wording fixes: the abstract now says *call-level* pass rates
(70.0--87.3 % is pass@1; pass@5 reaches 95.5 %), and "a separate observation
with the same command" now names it as a separate run with the same column
request.

### The five claims that looked unsupported

They were not wrong, they were untraceable, and the trace was in the decision
journal rather than in `metrics.json`:

| Claim | Source |
| --- | --- |
| "in model" 0 / 9.5 / 31.6 / 51.2 s | per-run max of `llm_seconds` in `llm_chat/decisions_*.jsonl`; means reproduce exactly |
| "supervision records one pass" | `supervision_steps` is 1 in all thirteen journals |
| "five groups of four with a home location each" | `amongus.world`: five colours, four robots each, 2x2 grid, centre = home from YAML |
| "moved a target by two metres" | no artifact; a development observation, and the paragraph already says no rates are given |
| "Gazebo Harmonic did not provide a usable real-time factor" | no artifact; already framed as an engineering observation, not a comparison |

The first three are now cited in the text: the E2 caption names `llm_seconds`
and the timing paragraph names `supervision_steps`. The last two stay as
labelled development observations.

## Results archive, 4 November 2026

The rosbag payloads are no longer "available from the corresponding author on
reasonable request". The whole `paper/results` directory is shipped as one
archive, and Data Availability names it, its size and its checksum:

```bash
cd paper && tar --zstd -cf ~/llm_swarm_results_2026-11-04.tar.zst results
sha256sum llm_swarm_results_2026-11-04.tar.zst | tee llm_swarm_results_2026-11-04.tar.zst.sha256
find results -type f -print0 | sort -z | xargs -0 sha256sum > llm_swarm_results_2026-11-04.manifest.sha256
```

| | |
| --- | --- |
| Archive | `llm_swarm_results_2026-11-04.tar.zst` |
| Uncompressed | 2.1 GB (1.97 GiB), 282 files, 382 entries |
| Compressed | 135 MB (128.3 MiB), ratio 15.7x |
| SHA-256 | `3866f1fce6254243986b0052e2fc0d6039eadc351e10c460f4ed0feec69f740d` |
| Manifest | 282 per-file checksums, sorted by path |

The 15.7x ratio is a property of the format, not of the data being small:
rosbag2 writes fixed-width CDR messages into 4 KB SQLite pages, so padding,
repeated frame names and near-identical consecutive `/tf` and `/odom` records
compress almost for free. The largest single bag, 195 MB, compresses to 12.7 MB
on its own with `zstd -3`.

The archive is **not** in the repository. At 135 MB it is above GitHub's 100 MB
hard per-file limit, and dropping `/tf` would still leave a 121 MB bag. The
root README and the manuscript both link to the public archive.

## Reference pass, 4 November 2026

`refs.bib` was normalised: the ten entries that were typed as `@misc` with a
`howpublished` field holding a journal or proceedings name are now proper
`@article` / `@inproceedings` records with volume, pages and DOI (CBS, PBS,
MAPF-LNS2, Nav2, SayCan, Code as Policies, Inner Monologue, RoCo, ROS 2, Stage).
`btcpp` remains `@misc` with a URL and a year. arXiv-only entries keep their
DOI and no longer repeat the abstract URL. The bibliography is unchanged in
content, only in record type and metadata.

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
