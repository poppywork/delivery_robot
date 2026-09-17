#!/usr/bin/env bash
# 安装 move_grasp_demo 所需的第三方依赖（系统 python3.10，与 ROS 2 Humble 同一解释器）。
#
#   ./install_move_grasp_demo_deps.sh            # 装到当前用户 ~/.local（推荐，免 sudo）
#   ./install_move_grasp_demo_deps.sh --system   # 装到系统 /usr/local（需要 sudo，多用户共享）
#
# 为什么是这些包（实测得出的最小集，非猜测）：
#   open3d      delivery_robot 的点云 / 刚体变换
#   pyqtgraph   delivery_robot 显示窗口（capGrasp → GUI → displayDataWindow）
#   urx+math3d  robotic_arm_dal 无条件导入 UR 驱动，少一个就 import 失败
#   plotly/dash `import open3d` 会连带加载 open3d.visualization.draw_plotly
#   sklearn     `import open3d` → open3d.ml.datasets → semantickitti
#   addict      open3d._ml3d.utils.config
#   pyquaternion/pillow   open3d 声明依赖
#   pymodbus+pyudev       DH 夹爪的 Modbus 串口通信
#
# 两个必须注意的点：
#   1) numpy 必须停在 1.x —— open3d 的依赖会把 numpy 拉到 2.x（实测 1.24.4 → 2.2.6），
#      而 ROS 2 与 graspgen 宿主节点都按 numpy 1.24 验证，所以命令里始终带 "numpy<2"。
#   2) open3d/pyqtgraph/urx 用 --no-deps 安装：open3d 声明里拖着 matplotlib/pandas/
#      ipywidgets/jupyter 等一系列导入期并不需要的包（约 50MB+），显式列最小集更省流量。
set -euo pipefail

PY=/usr/bin/python3
BIG=(open3d==0.18.0 pyqtgraph==0.13.7 urx==0.11.0)
SMALL=("numpy<2" plotly dash scikit-learn addict pyquaternion "pillow>=9.3"
       math3d==3.4.1 pymodbus==3.7.4 pyudev)

avail_gb=$(df -Pk "$HOME" | awk 'NR==2 {print int($4/1024/1024)}')
if (( avail_gb < 3 )); then
    echo "[deps] 磁盘可用仅 ${avail_gb}G，open3d 约需 1.5G，请先清理后重试" >&2
    exit 1
fi

if [[ "${1:-}" == "--system" ]]; then
    echo "[deps] 系统级安装到 /usr/local（sudo；复用本机 pip 缓存避免重复下载）"
    sudo "$PY" -m pip install --cache-dir "$HOME/.cache/pip" --no-deps "${BIG[@]}"
    sudo "$PY" -m pip install --cache-dir "$HOME/.cache/pip" "${SMALL[@]}"
else
    echo "[deps] 用户级安装到 ~/.local（免 sudo；~/.local 优先级高于 /usr/local）"
    "$PY" -m pip install --user --no-deps "${BIG[@]}"
    "$PY" -m pip install --user "${SMALL[@]}"
fi

echo
echo "[deps] 自检整条导入链 ……"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$(dirname "$(readlink -f "$0")")/../delivery_robot"
"$PY" - <<'PYEOF'
import rclpy, numpy, open3d, pyqtgraph, urx
print("  rclpy/open3d/pyqtgraph/urx  OK   numpy =", numpy.__version__)
from robotic_arm.robotic_arm_dal import robotic_arm_register  # noqa: F401
print("  机械臂驱动链                OK")
import capGrasp, armCom  # noqa: F401
print("  夹爪/触觉线程链 + armCom    OK")
assert numpy.__version__.startswith("1."), "numpy 必须是 1.x，当前 " + numpy.__version__
print("\n[deps] 依赖就绪")
PYEOF