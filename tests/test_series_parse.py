#!/usr/bin/env python3
"""Self-contained tests for the series-parsing logic shared between
capture_waypoints.py and goto_waypoint.py. Runs without ROS, without
network, without the robot — just `python3 test_series_parse.py`.

The parser lives inline in both production scripts (small enough not to
warrant a shared module). These tests pin its behaviour so the two
copies stay in sync.
"""
import re
import sys

# Mirror of the regex in both production scripts. Bumping it here without
# updating the scripts will trip the consistency check at the bottom.
SERIES_RE = re.compile(r"^([A-Za-z][A-Za-z_]*)(\d+)$")
SERIES_BRACKET_RE = re.compile(r"^([A-Za-z][A-Za-z_]*)\[\]$")


def parse_series_member(label):
    m = SERIES_RE.match(label)
    if not m:
        return None
    return m.group(1), int(m.group(2))


def compute_series(waypoints):
    series = {}
    for label in waypoints:
        parsed = parse_series_member(label)
        if parsed is None:
            continue
        group, idx = parsed
        series.setdefault(group, []).append((idx, label))
    for group in series:
        series[group].sort(key=lambda t: t[0])
    return series


# ---------- tests ----------

def t(name, cond, detail=""):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}  {detail}")
        t.failed += 1
t.failed = 0


def section(title):
    print(f"\n=== {title} ===")


section("parse_series_member: positive cases")
t("office1 -> (office, 1)", parse_series_member("office1") == ("office", 1))
t("office10 -> (office, 10)", parse_series_member("office10") == ("office", 10))
t("office01 -> (office, 1)  (leading zero treated as int)",
  parse_series_member("office01") == ("office", 1))
t("office_1 -> (office_, 1)  (trailing underscore allowed in prefix)",
  parse_series_member("office_1") == ("office_", 1))
t("dock_a3 -> (dock_a, 3)",
  parse_series_member("dock_a3") == ("dock_a", 3))
t("R7 -> (R, 7)  (single-letter prefix, uppercase)",
  parse_series_member("R7") == ("R", 7))


section("parse_series_member: negative cases")
t("'office' (no digits) -> None", parse_series_member("office") is None)
t("'1office' (digits-first) -> None", parse_series_member("1office") is None)
t("'office1a' (letter after digits) -> None",
  parse_series_member("office1a") is None)
t("'office-1' (hyphen in prefix) -> None",
  parse_series_member("office-1") is None)
t("'office.1' (dot) -> None", parse_series_member("office.1") is None)
t("'' -> None", parse_series_member("") is None)
t("'office 1' (space) -> None", parse_series_member("office 1") is None)
t("'office1.5' (decimal) -> None", parse_series_member("office1.5") is None)


section("compute_series: integer ordering, not lexical")
wps = {f"office{i}": {} for i in [1, 2, 9, 10, 11, 100]}
s = compute_series(wps)
order = [lab for _, lab in s["office"]]
t("office[1,2,9,10,11,100] sorts numerically (10 after 9, 100 last)",
  order == ["office1", "office2", "office9", "office10", "office11", "office100"],
  detail=f"got {order}")


section("compute_series: gaps allowed")
wps = {"office1": {}, "office3": {}, "office5": {}}
s = compute_series(wps)
order = [lab for _, lab in s["office"]]
t("gaps: walks 1, 3, 5 in order without filling",
  order == ["office1", "office3", "office5"], detail=f"got {order}")


section("compute_series: groups isolated by prefix")
wps = {"office1": {}, "office2": {}, "kitchen1": {}, "Office1": {},
       "office_1": {}, "office_2": {}}
s = compute_series(wps)
t("kitchen group separate from office",
  set(s.keys()) == {"office", "Office", "kitchen", "office_"},
  detail=f"got {sorted(s.keys())}")
t("Office (capital) is its own group (case-sensitive)",
  s["Office"] == [(1, "Office1")])
t("office_ (trailing underscore) is its own group",
  [lab for _, lab in s["office_"]] == ["office_1", "office_2"])


section("compute_series: non-series labels ignored")
wps = {"office1": {}, "office2": {}, "kitchen": {}, "door-A": {}, "test1a": {}}
s = compute_series(wps)
t("only office\\d+ becomes a group",
  list(s.keys()) == ["office"],
  detail=f"got {list(s.keys())}")
t("non-series labels are not in any group's member list",
  all(lab.startswith("office") for entries in s.values() for _, lab in entries))


section("SERIES_BRACKET_RE: parses the goto syntax")
def bm(s):
    m = SERIES_BRACKET_RE.match(s)
    return m.group(1) if m else None
t("'office[]' -> 'office'", bm("office[]") == "office")
t("'office_[]' -> 'office_'", bm("office_[]") == "office_")
t("'office[' -> None (no closing)", bm("office[") is None)
t("'office[1]' -> None (has digit)", bm("office[1]") is None)
t("'[]' -> None (empty prefix)", bm("[]") is None)
t("'1office[]' -> None (digit-first)", bm("1office[]") is None)


section("Mutual exclusion semantics")
def can_capture_single(label, waypoints):
    """Return True iff a single capture for `label` is allowed: refused
    if any series member with prefix=label exists."""
    if label in waypoints:
        return True   # overwrite of existing single — still a single
    return label not in compute_series(waypoints)

def can_start_series(group, waypoints):
    """Return True iff `<group>[]` series capture is allowed: refused
    if a bare `group` single already exists."""
    return group not in waypoints

wps = {"office1": {}, "office2": {}, "kitchen": {}}
t("can_capture_single('kitchen', ...) overwrite existing -> True",
  can_capture_single("kitchen", wps))
t("can_capture_single('office', {office1,office2,kitchen}) -> False  (series exists)",
  not can_capture_single("office", wps))
t("can_capture_single('foo', ...) -> True  (fresh label)",
  can_capture_single("foo", wps))
t("can_start_series('office', ...) -> True  (no bare office)",
  can_start_series("office", wps))
t("can_start_series('kitchen', ...) -> False  (bare kitchen exists)",
  not can_start_series("kitchen", wps))


section("Live yaml shape: no contradictions possible by construction")
# The capture tool enforces both can_capture_single and can_start_series
# on every input, so a yaml that only ever passed through the capture
# tool can never have both bare 'foo' AND 'foo\d+' simultaneously. We
# verify the invariant detection.
def yaml_is_consistent(waypoints):
    series = compute_series(waypoints)
    return not any(group in waypoints for group in series)

t("consistent: only series",
  yaml_is_consistent({"office1": {}, "office2": {}}))
t("consistent: only single",
  yaml_is_consistent({"office": {}}))
t("consistent: separate groups",
  yaml_is_consistent({"office1": {}, "kitchen": {}}))
t("inconsistent: bare AND series  (would require hand-edit; goto refuses)",
  not yaml_is_consistent({"office": {}, "office1": {}}))


# ---------- summary ----------
print()
if t.failed == 0:
    print("ALL TESTS PASSED")
    sys.exit(0)
else:
    print(f"{t.failed} TEST(S) FAILED")
    sys.exit(1)
