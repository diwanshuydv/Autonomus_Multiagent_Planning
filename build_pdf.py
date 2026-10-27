"""Render REPORT.md to REPORT.pdf (Markdown -> HTML -> PDF via headless Chrome).

Usage:  python build_pdf.py [--chrome PATH]
"""
import argparse
import os
import shutil
import subprocess
import sys

import markdown

ROOT = os.path.dirname(os.path.abspath(__file__))
CHROME_CANDIDATES = [
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "/Applications/Chromium.app/Contents/MacOS/Chromium",
    "google-chrome", "chromium", "chromium-browser", "chrome",
]

CSS = """
@page { size: A4; margin: 16mm 14mm 16mm 14mm; }
body { font-family: -apple-system, "Helvetica Neue", Arial, sans-serif; font-size: 10pt;
       line-height: 1.45; color: #1a1a1a; max-width: 100%; }
h1 { font-size: 19pt; margin: 0 0 4pt; }
h2 { font-size: 14pt; margin: 18pt 0 6pt; border-bottom: 1px solid #ccc; padding-bottom: 2pt;
     break-after: avoid; }
h3 { font-size: 11.5pt; margin: 12pt 0 4pt; break-after: avoid; }
p, li { orphans: 3; widows: 3; }
table { border-collapse: collapse; margin: 6pt 0 10pt; font-size: 8.5pt; width: 100%;
        break-inside: avoid; }
th, td { border: 1px solid #c8c8c8; padding: 2.5pt 4pt; text-align: left; vertical-align: top; }
th { background: #f0f0ee; }
img { max-width: 100%; display: block; margin: 6pt auto; break-inside: avoid; }
code { font-family: Menlo, Consolas, monospace; font-size: 8.5pt; background: #f4f4f2;
       padding: 0 2pt; }
pre { background: #f4f4f2; padding: 6pt; font-size: 8pt; white-space: pre-wrap; break-inside: avoid; }
pre code { background: none; padding: 0; }
a { color: #1f5fae; text-decoration: none; }
.meta { color: #555; margin-bottom: 10pt; }
"""


def find_chrome(explicit):
    for c in ([explicit] if explicit else []) + CHROME_CANDIDATES:
        if os.path.isfile(c) or shutil.which(c):
            return c if os.path.isfile(c) else shutil.which(c)
    sys.exit("Chrome/Chromium not found; pass --chrome PATH")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chrome")
    ap.add_argument("--src", default="REPORT.md")
    ap.add_argument("--out", default="REPORT.pdf")
    args = ap.parse_args()
    with open(os.path.join(ROOT, args.src)) as fh:
        body = markdown.markdown(fh.read(), extensions=["tables", "fenced_code", "sane_lists"])
    html = (f"<!doctype html><html><head><meta charset='utf-8'><title>Report</title>"
            f"<style>{CSS}</style></head><body>{body}</body></html>")
    tmp = os.path.join(ROOT, "_report_build.html")
    with open(tmp, "w") as fh:
        fh.write(html)
    out = os.path.join(ROOT, args.out)
    if os.path.exists(out):
        os.remove(out)
    try:
        # Chrome sometimes exits non-zero during teardown after writing the PDF,
        # so success is judged by the output file, not the exit code
        proc = subprocess.run([find_chrome(args.chrome), "--headless", "--disable-gpu",
                               "--no-pdf-header-footer", f"--print-to-pdf={out}", "file://" + tmp],
                              capture_output=True, text=True, timeout=300)
    finally:
        os.remove(tmp)
    if not os.path.exists(out):
        sys.exit(f"PDF was not written:\n{proc.stderr[-2000:]}")
    print(f"wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
