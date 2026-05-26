# Data

Per-environment runtime data (waypoints captured by the operator,
goto history, batch reports). Only this README is git-tracked; the
actual data files are environment-specific and excluded by
`.gitignore`.

## What lands here

| File | Produced by | Notes |
|---|---|---|
| `waypoints.yaml` | `tools/gotop/capture.sh` | label → pose dict; survives container stop/start because this dir is the bind-mounted host repo working tree |
| `goto_history.csv` | `tools/gotop/goto.sh` | append-only, each row is one segment with goal/reached/error |
| `batch_report.md` | `tools/gotop/batch.sh` | overwritten each run; per-segment + per-waypoint summary |

To inspect from the G1 host without docker exec:

```bash
ssh unitree@192.168.100.30
cd ~/g1_3d_nav_ros2_repo/data
ls -la
# you'll see whatever the wrappers have produced
```

## If you want to share a particular waypoints.yaml across machines

1. Copy or paste it into a file under `maps/` or push it to a
   separate branch — anything in `data/` itself is .gitignore'd
   precisely because waypoint sets are usually environment-specific
   and shouldn't pollute git history.
2. Or call the wrapper with an explicit alternate path:
   ```bash
   docker exec -it 3d_nav_ros2 \
       env WAYPOINTS_YAML=/g1_3d_nav_ros2/maps/lab_waypoints.yaml \
       /g1_3d_nav_ros2/tools/gotop/goto.sh
   ```
