#!/bin/bash
set -e

# ============================================================
# cuarm_rt_control build script
# Safe against interrupted builds — auto-cleans stale .o files
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
BUILD_TYPE="Release"    # Default: Release
JOBS=$(nproc)           # Default: all cores
DO_CLEAN=0              # Whether to do full clean rebuild

usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -d, --debug        Build with Debug mode (default: Release)"
    echo "  -c, --clean        Full clean rebuild (delete all build artifacts)"
    echo "  -j <N>             Number of parallel jobs (default: $(nproc))"
    echo "  -h, --help         Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                    # Release build"
    echo "  $0 -d                 # Debug build"
    echo "  $0 -c                 # Full clean rebuild"
    echo "  $0 -d -c -j 2         # Debug + clean + 2 parallel jobs"
    exit 0
}

# ---------- Parse arguments ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--debug) BUILD_TYPE="Debug"; shift ;;
        -c|--clean) DO_CLEAN=1; shift ;;
        -j) JOBS="$2"; shift 2 ;;
        -h|--help) usage ;;
        *) echo "Unknown option: $1"; usage ;;
    esac
done

echo "============================================"
echo " cuarm_rt_control Build"
echo " Build type : ${BUILD_TYPE}"
echo " Build dir  : ${BUILD_DIR}"
echo " Parallel   : ${JOBS} jobs"
echo "============================================"

# ---------- Full clean if requested ----------
if [ "$DO_CLEAN" -eq 1 ]; then
    echo "[CLEAN] Removing entire build directory..."
    rm -rf "$BUILD_DIR"
fi

# ---------- Create build dir ----------
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# ---------- CMake configure (only if needed) ----------
if [ ! -f "CMakeCache.txt" ]; then
    echo "[CMAKE] Configuring..."
    cmake "$SCRIPT_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
else
    current_type=$(grep "CMAKE_BUILD_TYPE:" CMakeCache.txt | cut -d= -f2)
    if [ "$current_type" != "$BUILD_TYPE" ]; then
        echo "[CMAKE] Build type changed (${current_type} -> ${BUILD_TYPE}), reconfiguring..."
        cmake "$SCRIPT_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    else
        echo "[CMAKE] Already configured, skipping."
    fi
fi

# ---------- Clean 0-byte stale .o files ----------
echo "[CHECK] Scanning for stale (0-byte) object files..."
stale_count=$(find . -name "*.o" -size 0 2>/dev/null | wc -l)
if [ "$stale_count" -gt 0 ]; then
    echo "[CLEAN] Found ${stale_count} stale .o file(s), removing..."
    find . -name "*.o" -size 0 -delete 2>/dev/null || true
    echo "[CLEAN] Done."
else
    echo "[CHECK] No stale object files found."
fi

# ---------- Build ----------
echo "[BUILD] Compiling with ${JOBS} parallel job(s)..."
make -j"$JOBS"

echo ""
echo "============================================"
echo " Build succeeded!"
echo " Executable: ${BUILD_DIR}/spark2_v2_rt_control"
echo "============================================"
