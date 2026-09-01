#!/usr/bin/env bash
# End-to-end integration test: MicroPython client <-> stock micro-ROS Agent <-> ROS 2.
#
# Asserts in both directions:
#   * what SnakeROS publishes is readable by `ros2 topic echo`
#   * what `ros2 topic pub` sends is received and decoded by SnakeROS
set -uo pipefail

AGENT_CONTAINER=${AGENT_CONTAINER:-snakeros_agent}
ROS_CONTAINER=${ROS_CONTAINER:-snakeros_ros}
MICROPYTHON=${MICROPYTHON:-micropython}
DURATION=${DURATION:-20}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

fail=0
say() { printf '\n=== %s ===\n' "$1"; }
ok()  { printf '  PASS  %s\n' "$1"; }
bad() { printf '  FAIL  %s\n' "$1"; fail=1; }

rosx() { docker exec "$ROS_CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash && $1"; }

say "preflight"
docker ps --format '{{.Names}}' | grep -q "^${AGENT_CONTAINER}$" || { echo "agent container not running"; exit 2; }
docker ps --format '{{.Names}}' | grep -q "^${ROS_CONTAINER}$"   || { echo "ros container not running"; exit 2; }
command -v "$MICROPYTHON" >/dev/null || { echo "micropython not found"; exit 2; }
ok "containers up, micropython present"

say "starting client"
"$MICROPYTHON" tests/integration_client.py 127.0.0.1 "$DURATION" > /tmp/snakeros_itest.log 2>&1 &
CLIENT_PID=$!
sleep 6

grep -q CONNECTED   /tmp/snakeros_itest.log && ok "session established" || bad "no session"
grep -q ENTITIES_OK /tmp/snakeros_itest.log && ok "entities created"    || bad "entity creation failed"

say "ROS 2 sees our topics"
TOPICS=$(rosx "timeout 10 ros2 topic list" 2>/dev/null)
echo "$TOPICS" | grep -q /snakeros_chatter && ok "/snakeros_chatter in graph" || bad "/snakeros_chatter missing"
echo "$TOPICS" | grep -q /snakeros_imu     && ok "/snakeros_imu in graph"     || bad "/snakeros_imu missing"

say "ROS 2 can decode what we publish"
ECHO_S=$(rosx "timeout 10 ros2 topic echo /snakeros_chatter std_msgs/msg/String --once" 2>/dev/null)
echo "$ECHO_S" | grep -q "snakeros integration" && ok "String decoded by ROS 2" || bad "String not decoded: $ECHO_S"

ECHO_I=$(rosx "timeout 12 ros2 topic echo /snakeros_imu sensor_msgs/msg/Imu --once" 2>/dev/null)
echo "$ECHO_I" | grep -q "imu_link"  && ok "Imu frame_id decoded"      || bad "Imu frame_id wrong"
echo "$ECHO_I" | grep -q "9.81"      && ok "Imu float64 decoded"       || bad "Imu float64 wrong"
echo "$ECHO_I" | grep -q "0.5"       && ok "Imu covariance decoded"    || bad "Imu covariance wrong"

say "we can decode what ROS 2 publishes"
rosx "timeout 8 ros2 topic pub -r 10 /snakeros_cmd geometry_msgs/msg/Twist '{linear: {x: 0.42}, angular: {z: -1.75}}'" >/dev/null 2>&1 &
wait $CLIENT_PID 2>/dev/null

RECEIVED=$(grep '^RECEIVED' /tmp/snakeros_itest.log | awk '{print $2}')
ERRORS=$(grep '^DECODE_ERRORS' /tmp/snakeros_itest.log | awk '{print $2}')
SAMPLE=$(grep '^SAMPLE' /tmp/snakeros_itest.log | awk '{print $2, $3}')

[ "${RECEIVED:-0}" -gt 5 ]  && ok "received $RECEIVED Twist messages"  || bad "received only ${RECEIVED:-0} Twist messages"
[ "${ERRORS:-1}" -eq 0 ]    && ok "no decode errors"                   || bad "$ERRORS decode errors"
[ "$SAMPLE" = "0.420 -1.750" ] && ok "Twist values correct ($SAMPLE)"  || bad "Twist values wrong: '$SAMPLE'"
grep -q DONE /tmp/snakeros_itest.log && ok "clean shutdown" || bad "client did not shut down cleanly"

say "result"
if [ "$fail" -eq 0 ]; then echo "integration: ALL PASSED"; else echo "integration: FAILURES"; sed -n '1,40p' /tmp/snakeros_itest.log; fi
exit $fail
