#!/bin/bash
#
# rename_host.sh — sync the G1 host IP across docs and operator-facing
# command examples in this repo.
#
# Why this exists:
#   The G1 IP changes whenever the team moves to a new debugging site
#   (different lab, different network). The IP appears as a literal
#   string in ~22 places (README Quickstart, SSH commands, scp examples,
#   tools/launch.sh comments, side READMEs). We keep the literal
#   everywhere — not a ${G1_HOST} placeholder — so that operator
#   commands are copy-paste ready. This script rewrites all of them
#   in one shot when the IP changes.
#
# Source of truth:
#   configs/g1_host.txt holds the current G1 IP as a single plain line.
#   This file is read by THIS SCRIPT only. No runtime process reads it.
#   Cross-host Zenoh is still configured at run time via
#   ZENOH_CONFIG_OVERRIDE on the operator's shell (see README Quickstart).
#
# What gets touched:
#   Every git-tracked file containing the old IP, EXCEPT:
#     - docs/TEST_REPORTS/**           historical evidence of past runs
#     - README.md "> Status:" line     historical "verified on date X" claim
#     - configs/g1_host.txt            rewritten separately at the end
#
# Usage:
#   bash tools/rename_host.sh NEW_IP
#   bash tools/rename_host.sh --dry-run NEW_IP
#

set -uo pipefail

DRY_RUN=0
if [ "${1:-}" = "--dry-run" ]; then
    DRY_RUN=1
    shift
fi

NEW="${1:-}"
if [ -z "$NEW" ]; then
    echo "Usage: $0 [--dry-run] NEW_IP" >&2
    echo "  Reads the current G1 IP from configs/g1_host.txt and replaces" >&2
    echo "  it with NEW_IP across all operator-facing docs and scripts." >&2
    exit 1
fi

cd "$(git rev-parse --show-toplevel)"

HOST_FILE="configs/g1_host.txt"
if [ ! -f "$HOST_FILE" ]; then
    echo "ERROR: $HOST_FILE missing" >&2
    exit 1
fi

OLD="$(cat "$HOST_FILE")"
if [ -z "$OLD" ]; then
    echo "ERROR: $HOST_FILE is empty" >&2
    exit 1
fi

if [ "$OLD" = "$NEW" ]; then
    echo "Nothing to do: $HOST_FILE already says $NEW"
    exit 0
fi

# Sanity: OLD must appear somewhere in the tree besides the host file.
HITS=$(git grep -l "$OLD" 2>/dev/null | grep -v "^$HOST_FILE\$" || true)
if [ -z "$HITS" ]; then
    echo "ERROR: $HOST_FILE says $OLD but it appears nowhere else in the tree." >&2
    echo "  The source of truth is out of sync. Investigate before retrying." >&2
    exit 1
fi

TOUCHED=$(echo "$HITS" | grep -v '^docs/TEST_REPORTS/' || true)
HISTORICAL=$(echo "$HITS" | grep '^docs/TEST_REPORTS/' || true)

echo "Old IP (from $HOST_FILE): $OLD"
echo "New IP:                   $NEW"
echo ""
echo "Files that will change:"
echo "$TOUCHED" | sed 's/^/  /'
if [ -n "$HISTORICAL" ]; then
    echo ""
    echo "Excluded (kept as historical record):"
    echo "$HISTORICAL" | sed 's/^/  /'
fi
if grep -q "^> Status:.*$OLD" README.md 2>/dev/null; then
    echo ""
    echo "README.md '> Status:' line will be SKIPPED (historical claim)."
fi
echo ""

if [ "$DRY_RUN" = "1" ]; then
    echo "(dry run — no changes written)"
    exit 0
fi

# README.md: skip the historical "> Status:" claim line.
if echo "$TOUCHED" | grep -q '^README\.md$'; then
    sed -i "/^> Status:/!s|$OLD|$NEW|g" README.md
fi

# All other touched files.
echo "$TOUCHED" | grep -v '^README\.md$' | while read -r f; do
    [ -n "$f" ] && sed -i "s|$OLD|$NEW|g" "$f"
done

# Update the source of truth last.
echo "$NEW" > "$HOST_FILE"

echo "Done."
echo ""
echo "Review:   git diff"
echo "Commit:   git commit -am 'rename G1 host: $OLD -> $NEW'"
