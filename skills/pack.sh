#!/usr/bin/env bash
# Repackage skills/multi-agent-uavs.skill from skills/multi-agent-uavs/.
#
# Run this after editing SKILL.md so the .skill ZIP bundle stays in sync
# with the source directory. The bundle is what Cowork's "Save skill"
# flow consumes, so a stale bundle means anyone installing the skill
# from this repo gets the old version. Claude Code reads the source
# directory directly (via the .claude/skills/ symlink) and doesn't need
# the bundle, but we keep them in lockstep so the repo never ships a
# half-fresh skill.

set -euo pipefail

# Always operate relative to the directory this script lives in,
# regardless of where it's invoked from.
cd "$(dirname "$0")"

OUT=multi-agent-uavs.skill
SRC=multi-agent-uavs

if [[ ! -d "$SRC" ]]; then
  echo "error: $SRC/ not found next to this script" >&2
  exit 1
fi
if ! command -v python3 >/dev/null; then
  echo "error: python3 required (using its stdlib zipfile module)" >&2
  exit 1
fi

python3 - <<PY
import os, zipfile
out = "$OUT"
src = "$SRC"
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zf:
    for root, _, files in os.walk(src):
        # Skip __pycache__ and the like if they ever appear
        if "__pycache__" in root:
            continue
        for f in sorted(files):
            full = os.path.join(root, f)
            # arcname preserves the "multi-agent-uavs/..." prefix inside
            # the ZIP, which is what Cowork expects.
            zf.write(full, arcname=full)
            print(f"added: {full}")
print(f"\nRepackaged {out} from {src}/")
PY
