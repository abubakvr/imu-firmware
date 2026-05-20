#!/usr/bin/env python3
"""Regenerate src/index_html.c from www/index.html."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
html_path = ROOT / "www" / "index.html"
out_path = ROOT / "src" / "index_html.c"

html = html_path.read_text(encoding="utf-8")
lines = ['#include "index_html.h"\n', '\n', 'const char index_html[] =\n']
for line in html.splitlines(keepends=True):
    if not line:
        lines.append('"\n"\n')
        continue
    escaped = line.replace("\\", "\\\\").replace('"', '\\"')
    lines.append(f'"{escaped.rstrip(chr(10)).rstrip(chr(13))}\\n"\n')
lines.append(";\n\n")
lines.append("const size_t index_html_len = sizeof(index_html) - 1;\n")
out_path.write_text("".join(lines), encoding="utf-8")
print(f"Wrote {out_path} ({len(html)} bytes from {html_path})")
