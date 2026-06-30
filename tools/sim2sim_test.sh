#!/bin/bash
# ============================================================
# sim2sim_test.sh — Navigation Sim2Sim 验证 (navigation 子模块内版)
#
# 适用于导航团队独立开发时使用。假设 motion_control 已构建。
#
# 用法:
#   # 在 F1 集成仓库中 (navigation/ 是 submodule):
#   cd navigation
#   tools/sim2sim_test.sh
#
#   # 在独立 clone 的 Humanoid_navigation 仓库中:
#   tools/sim2sim_test.sh --build-dir /path/to/F1/build --model-path /path/to/xyber_x1_nav.xml
#
#   # 可选参数:
#   tools/sim2sim_test.sh --no-build-nav          # 跳过 navigation 构建
#   tools/sim2sim_test.sh --scenarios A,C,E       # 只跑指定场景
#   tools/sim2sim_test.sh --single 5.0 0.0        # 单点目标
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NAV_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"    # navigation 仓库根目录

# --- 路径自动检测 ---
# 情况 1: 在 F1 集成仓库中 (../motion_control 和 ../build 存在)
INTEGRATION_DIR="$(cd "$NAV_ROOT/.." && pwd)"
if [ -f "${INTEGRATION_DIR}/build/aimrt_main" ]; then
    BUILD_DIR="${INTEGRATION_DIR}/build"
    DEFAULT_MODEL="${BUILD_DIR}/cfg/sim_module/model/mjcf/xyber_x1_nav.xml"
    ROOT_DIR="$INTEGRATION_DIR"
    INTEGRATION_MODE=true
else
    BUILD_DIR="${BUILD_DIR:-}"
    DEFAULT_MODEL="${MODEL_PATH:-}"
    ROOT_DIR="$NAV_ROOT"
    INTEGRATION_MODE=false
fi

MODEL_PATH="${DEFAULT_MODEL}"
DO_BUILD_NAV=true
SCENARIOS=""
SINGLE_GOAL=""
REPORT_DIR="${NAV_ROOT}/reports"
READINESS_TIMEOUT=120
SESSION_NAME="f1_sim_nav"

# --- 颜色 ---
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

# --- 参数解析 ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --build-dir)    BUILD_DIR="$2"; shift 2 ;;
        --model-path)   MODEL_PATH="$2"; shift 2 ;;
        --no-build-nav) DO_BUILD_NAV=false; shift ;;
        --scenarios)    SCENARIOS="$2"; shift 2 ;;
        --single)       SINGLE_GOAL="$2 $3"; shift 3 ;;
        --report-dir)   REPORT_DIR="$2"; shift 2 ;;
        --help|-h)      head -18 "$0" | tail -16; exit 0 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

mkdir -p "$REPORT_DIR"

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════╗"
echo "║   Nav Sim2Sim Validation (navigation tools)  ║"
echo "╚══════════════════════════════════════════════╝"
echo -e "${NC}"

# ============================================================
# Phase 1: Pre-check
# ============================================================
echo -e "${BOLD}[Phase 1] Pre-check${NC}"

# Check motion_control build
AIMRT_MAIN="${BUILD_DIR}/aimrt_main"
if [ ! -f "$AIMRT_MAIN" ]; then
    echo -e "${RED}[ERROR] motion_control not built: ${AIMRT_MAIN}${NC}"
    if [ "$INTEGRATION_MODE" = true ]; then
        echo -e "  Run: cd ${ROOT_DIR} && scripts/build.sh"
    else
        echo -e "  Specify: --build-dir /path/to/F1/build"
    fi
    exit 1
fi
echo -e "${GREEN}  ✓ motion_control: ${AIMRT_MAIN}${NC}"

# Check scene model
if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${RED}[ERROR] Scene model missing: ${MODEL_PATH}${NC}"
    echo -e "  Specify: --model-path /path/to/xyber_x1_nav.xml"
    exit 1
fi
echo -e "${GREEN}  ✓ Scene model: ${MODEL_PATH}${NC}"

# Build navigation
if [ "$DO_BUILD_NAV" = true ]; then
    echo -e "${YELLOW}  → Building navigation (colcon)...${NC}"
    if [ "$INTEGRATION_MODE" = true ]; then
        cd "$ROOT_DIR" && bash scripts/build_nav.sh
    else
        cd "$NAV_ROOT"
        # Standalone colcon build
        source "${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}" 2>/dev/null || true
        colcon build --symlink-install \
            --packages-select fast_lio humanoid_sim open3d_loc \
            livox_laser_simulation_ros2 livox_ros_driver2
    fi
else
    echo -e "${GREEN}  ✓ navigation build skipped${NC}"
fi

[ -f "${NAV_ROOT}/install/setup.bash" ] || { echo -e "${RED}[ERROR] navigation install/ missing${NC}"; exit 1; }

# ============================================================
# Phase 2: Launch sim stack
# ============================================================
echo -e "\n${BOLD}[Phase 2] Launch Simulation Stack${NC}"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo -e "${YELLOW}  → Killing existing session${NC}"
    tmux kill-session -t "${SESSION_NAME}"; sleep 2
fi

if [ "$INTEGRATION_MODE" = true ]; then
    echo -e "${YELLOW}  → Starting via integration run_sim_nav.sh...${NC}"
    cd "$ROOT_DIR" && bash scripts/run_sim_nav.sh &
else
    # Standalone mode: launch components individually via tmux
    echo -e "${YELLOW}  → Starting standalone sim stack...${NC}"
    # (In standalone mode, assume motion_control sim + lidar bridge are running externally)
    echo -e "${YELLOW}  ⚠ Standalone mode: ensure aimrt_main + lidar_bridge are running${NC}"
    # Launch nav2 stack
    source "${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}" 2>/dev/null || true
    source "${NAV_ROOT}/install/setup.bash"
    tmux new-session -d -s "${SESSION_NAME}" -n "nav2"
    tmux send-keys -t "${SESSION_NAME}:0" \
        "source ${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}; source ${NAV_ROOT}/install/setup.bash; ros2 launch humanoid_sim navigation.launch.py" Enter
fi
LAUNCH_PID=$!

# ============================================================
# Phase 3: Wait for readiness
# ============================================================
echo -e "\n${BOLD}[Phase 3] Wait for Readiness${NC}"

source "${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}" 2>/dev/null || true
[ -f "${NAV_ROOT}/install/setup.bash" ] && source "${NAV_ROOT}/install/setup.bash"

READY=false
for i in $(seq 1 $READINESS_TIMEOUT); do
    if ros2 topic list 2>/dev/null | grep -q "/mujoco/ground_truth" && \
       ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
        READY=true
        echo -e "${GREEN}  ✓ Ready (${i}s)${NC}"
        break
    fi
    [ $((i % 10)) -eq 0 ] && echo -e "${YELLOW}  ... waiting (${i}/${READINESS_TIMEOUT}s)${NC}"
    sleep 1
done

[ "$READY" = true ] || { echo -e "${RED}[ERROR] Not ready after ${READINESS_TIMEOUT}s${NC}"; exit 1; }

echo -e "${YELLOW}  → Settling 5s...${NC}"
sleep 5

# ============================================================
# Phase 4: Run test scenarios
# ============================================================
echo -e "\n${BOLD}[Phase 4] Run Test Scenarios${NC}"

if [ -n "$SINGLE_GOAL" ]; then
    python3 "${SCRIPT_DIR}/nav_test_runner.py" --goal-x $(echo $SINGLE_GOAL | cut -d' ' -f1) \
        --goal-y $(echo $SINGLE_GOAL | cut -d' ' -f2) --report-dir "$REPORT_DIR"
elif [ -n "$SCENARIOS" ]; then
    for s in $(echo "$SCENARIOS" | tr ',' ' '); do
        echo -e "\n${CYAN}  >>> Scenario: ${s}${NC}"
        python3 "${SCRIPT_DIR}/nav_test_runner.py" --scenario "$s" --report-dir "$REPORT_DIR"
    done
else
    python3 "${SCRIPT_DIR}/nav_test_runner.py" --batch --report-dir "$REPORT_DIR"
fi

# ============================================================
# Phase 5: Teardown & Summary
# ============================================================
echo -e "\n${BOLD}[Phase 5] Teardown${NC}"
if [ "$INTEGRATION_MODE" = true ]; then
    tmux kill-session -t "${SESSION_NAME}" 2>/dev/null && echo -e "${GREEN}  ✓ Session killed${NC}"
fi
kill $LAUNCH_PID 2>/dev/null || true

echo -e "\n${GREEN}✓ Reports: ${REPORT_DIR}${NC}"
echo -e "${GREEN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║          Sim2Sim Test Complete              ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════╝${NC}"
