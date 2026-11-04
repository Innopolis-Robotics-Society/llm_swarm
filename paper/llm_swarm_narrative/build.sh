#!/usr/bin/env bash
# Build main.pdf and refuse to leave a half-built file behind.
#
# A single pdflatex pass leaves every \ref and \cite as "??" because the .aux
# is written at the end of the run and read at the beginning of the next one.
# The sequence below is the minimum that resolves everything: pdflatex (writes
# .aux) -> bibtex (writes .bbl) -> pdflatex (reads .bbl, writes labels) ->
# pdflatex (settles page numbers). Do not shorten it.
#
# Do not use `latexmk -C`: it deletes architecture.pdf, which is a source
# figure, not a build product.
set -euo pipefail
cd "$(dirname "$0")"

rm -f main.aux main.bbl main.blg main.out main.lot main.toc main.log

pdflatex -interaction=nonstopmode main.tex >/dev/null
bibtex   main                              >/dev/null
pdflatex -interaction=nonstopmode main.tex >/dev/null
pdflatex -interaction=nonstopmode main.tex >/tmp/main.last.log

fail=0
if grep -q '^!' /tmp/main.last.log; then
  echo "FAIL: TeX errors"; grep -n '^!' /tmp/main.last.log; fail=1
fi
if grep -q 'undefined' /tmp/main.last.log; then
  echo "FAIL: unresolved references or citations"
  grep -n 'undefined' /tmp/main.last.log; fail=1
fi
if grep -q 'Overfull' /tmp/main.last.log; then
  echo "WARN: overfull boxes"; grep -n 'Overfull' /tmp/main.last.log
fi
if ! pdfinfo main.pdf >/dev/null 2>&1; then
  echo "FAIL: main.pdf is not a readable PDF"; fail=1
fi
if pdftotext main.pdf - | grep -q '??'; then
  echo "FAIL: '??' in the rendered text"; fail=1
fi

[ "$fail" -eq 0 ] || exit 1
echo "OK: $(pdfinfo main.pdf | awk '/^Pages/{print $2}') pages, no unresolved references"
