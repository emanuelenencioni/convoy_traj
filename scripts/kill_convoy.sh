#!/bin/bash
# Force-kill all processes belonging to the convoy launch session.
# Use this only when Ctrl+C fails to stop the container.

for pattern in \
    "component_container_isolated" \
    "component_container_mt" \
    "ros2 launch convoy_traj"; do
    pids=$(pgrep -f "$pattern")
    if [ -n "$pids" ]; then
        echo "SIGKILL -> $pattern (PIDs: $pids)"
        kill -9 $pids
    fi
done

echo "Done."
