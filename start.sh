#!/usr/bin/env bash
# ambition_radar 一键启动脚本
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

find_project_root() {
    if [[ -n "${AMBITION_RADAR_ROOT:-}" ]]; then
        echo "$AMBITION_RADAR_ROOT"
        return 0
    fi

    local dir="$SCRIPT_DIR"
    while [[ "$dir" != "/" ]]; do
        if [[ -f "$dir/CMakeLists.txt" && -f "$dir/config/config.yaml" ]]; then
            echo "$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
    done

    local fallback="$HOME/ws/ambition_radar"
    if [[ -f "$fallback/CMakeLists.txt" && -f "$fallback/config/config.yaml" ]]; then
        echo "$fallback"
        return 0
    fi

    echo "$SCRIPT_DIR"
}

ROOT="$(find_project_root)"
BUILD_DIR="$ROOT/build"
APP="$BUILD_DIR/app"
CONFIG="$ROOT/config/config.yaml"

DO_BUILD=0
if [[ "${1:-}" == "--build" ]]; then
    DO_BUILD=1
    shift
elif [[ "${1:-}" == "--no-build" ]]; then
    shift
fi

prepend_path() {
    local dir="$1"
    [[ -d "$dir" ]] || return 0
    if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
        LD_LIBRARY_PATH="$dir:$LD_LIBRARY_PATH"
    else
        LD_LIBRARY_PATH="$dir"
    fi
    export LD_LIBRARY_PATH
}

setup_runtime_env() {
    prepend_path "/opt/MVS/lib/64"
    prepend_path "/opt/intel/openvino_2024.6/runtime/lib/intel64"
    prepend_path "/opt/intel/openvino_2024.6/runtime/3rdparty/tbb/lib"
    prepend_path "/usr/local/lib"
    if [[ -d "/usr/local/cuda/lib64" ]]; then
        prepend_path "/usr/local/cuda/lib64"
    fi
    for cuda_dir in /usr/local/cuda-*/lib64; do
        [[ -d "$cuda_dir" ]] && prepend_path "$cuda_dir"
    done

    if [[ -f "/opt/intel/openvino_2024.6/setupvars.sh" ]]; then
        # shellcheck disable=SC1091
        source "/opt/intel/openvino_2024.6/setupvars.sh" >/dev/null 2>&1 || true
    fi
}

need_build() {
    [[ ! -x "$APP" ]] && return 0
    [[ "$DO_BUILD" -eq 1 ]] && return 0
    return 1
}

build_app() {
    echo "[start] 编译项目..."
    mkdir -p "$BUILD_DIR"
    cmake -S "$ROOT" -B "$BUILD_DIR"
    cmake --build "$BUILD_DIR" -j"$(nproc)"
    echo "[start] 编译完成: $APP"
}

check_prerequisites() {
    if [[ ! -f "$CONFIG" ]]; then
        echo "[start] 错误: 找不到项目目录（缺少 config/config.yaml）" >&2
        echo "[start] 脚本位置: $SCRIPT_DIR" >&2
        echo "[start] 尝试过的项目根: $ROOT" >&2
        echo "[start] 可设置环境变量: export AMBITION_RADAR_ROOT=/path/to/ambition_radar" >&2
        exit 1
    fi

    if [[ -z "${DISPLAY:-}" ]]; then
        echo "[start] 警告: 未检测到 DISPLAY，OpenCV 预览窗口可能无法显示" >&2
    fi
}

main() {
    check_prerequisites

    if need_build; then
        build_app
    elif [[ ! -x "$APP" ]]; then
        echo "[start] 错误: 可执行文件不存在，请运行: $0 --build" >&2
        exit 1
    fi

    setup_runtime_env

    echo "[start] 项目目录: $ROOT"
    echo "[start] 配置文件: $CONFIG"
    echo "[start] 启动 ambition_radar..."
    cd "$ROOT"
    exec "$APP" "$@"
}

main "$@"
