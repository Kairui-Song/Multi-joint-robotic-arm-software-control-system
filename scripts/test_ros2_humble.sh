#!/usr/bin/env bash
# Run in Ubuntu 22.04 with ROS Humble dependencies installed.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo "This validation targets Ubuntu 22.04 / ROS Humble; found $PRETTY_NAME" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS Humble is not installed.' >&2; exit 2
fi
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
# Build on the Linux filesystem; source remains the shared Windows project.
validation_dir="${ARM_VALIDATION_DIR:-$HOME/arm_humble_validation}"
mkdir -p "$validation_dir"
run_dir="$(mktemp -d "$validation_dir/run-XXXXXXXX")"
exec > >(tee "$run_dir/validation.log") 2>&1
trap 'status=$?; echo "Exit status: $status; results: $run_dir"' EXIT
printf 'Source: %s\nResults: %s\n' "$project_dir" "$run_dir"
cd "$validation_dir"
colcon --log-base "$run_dir/colcon-log" build \
  --base-paths "$project_dir/参考文件/ros2_ws/src" --build-base "$validation_dir/build" \
  --install-base "$validation_dir/install" --executor sequential \
  --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DARM_WITH_ZCAN=OFF -DARM_WITH_LOONG=OFF -DARM_WITH_IGH=OFF -DARM_STANDALONE=OFF -DBUILD_TESTING=ON
source "$validation_dir/install/setup.bash"
colcon --log-base "$run_dir/test-log" test --base-paths "$project_dir/参考文件/ros2_ws/src" \
  --build-base "$validation_dir/build" --install-base "$validation_dir/install" \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base "$validation_dir/build" --verbose
for backend in sim can_mock rm_mock mit_mock; do
  echo "Running full ROS integration: $backend"
  python3 "$project_dir/参考文件/ros2_ws/src/arm_control/test/integration_sim.py" --backend "$backend"
done
echo 'PASS: ROS Humble build, unit/lifecycle tests, and all three simulated backends.'
