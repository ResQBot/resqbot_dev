#!/usr/bin/env bash
# =============================================================================
# Res.Q Bots — Ubuntu 24.04 / ROS 2 Jazzy setup script
#
# Usage:
#   chmod +x setup_jazzy.sh
#   ./setup_jazzy.sh
#
# What this script does:
#   1. Configure locale
#   2. Install ROS 2 Jazzy (desktop)
#   3. Install Gazebo Harmonic + ROS-Gz bridge packages
#   4. Install all robot-specific ROS packages (MoveIt2, ros2_control, etc.)
#   5. Install system-level deps (libserial, OpenCV, ...)
#   6. Clone the resqbot_dev repo and build the workspace
#   7. Set up .bashrc (source ROS, set DOMAIN_ID)
#   [Optional] Livox SDK2 + livox_ros_driver2 + Fast_LIO (LiDAR stack)
#   [Optional] audio_common (robot audio)
# =============================================================================

set -euo pipefail
IFS=$'\n\t'

# ── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

info()    { echo -e "${BLUE}[INFO]${NC}  $*"; }
ok()      { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
header()  { echo -e "\n${BOLD}${BLUE}══ $* ══${NC}"; }
die()     { echo -e "${RED}[FAIL]${NC}  $*" >&2; exit 1; }

ask() {
    # Returns 0 (yes) or 1 (no)
    local prompt="$1"
    local answer
    read -rp "$(echo -e "${YELLOW}[?]${NC} ${prompt} [y/N] ")" answer
    [[ "${answer,,}" == "y" || "${answer,,}" == "yes" ]]
}

check_ubuntu_version() {
    header "Checking OS"
    local ver
    ver=$(lsb_release -rs 2>/dev/null || echo "unknown")
    if [[ "$ver" != "24.04" ]]; then
        warn "This script targets Ubuntu 24.04 (detected: $ver)."
        if ! ask "Continue anyway?"; then
            die "Aborted."
        fi
    else
        ok "Ubuntu 24.04 detected."
    fi
}

# ── Step 1: Locale ────────────────────────────────────────────────────────────
setup_locale() {
    header "Step 1 — Locale"
    sudo apt update -q
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ok "Locale configured."
}

# ── Step 2: ROS 2 Jazzy ───────────────────────────────────────────────────────
install_ros2_jazzy() {
    header "Step 2 — ROS 2 Jazzy"

    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        ok "ROS 2 Jazzy already installed — skipping."
        return 0
    fi

    info "Adding ROS 2 apt repository..."
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository -y universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    info "Installing ROS 2 Jazzy desktop + dev tools..."
    sudo apt update -q
    sudo apt install -y ros-dev-tools
    sudo apt upgrade -y
    sudo apt install -y ros-jazzy-desktop

    ok "ROS 2 Jazzy installed."
}

# ── Step 3: Gazebo Harmonic + bridges ─────────────────────────────────────────
install_gazebo() {
    header "Step 3 — Gazebo Harmonic + ROS-Gz packages"

    info "Installing Gazebo Harmonic and ROS bridges..."
    sudo apt install -y \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-interfaces

    ok "Gazebo Harmonic packages installed."
}

# ── Step 4: Robot-specific ROS packages ───────────────────────────────────────
install_ros_packages() {
    header "Step 4 — Robot ROS packages"

    info "Installing ros2_control, MoveIt2, controllers, and tools..."
    sudo apt install -y \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-moveit \
        ros-jazzy-moveit-servo \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-rviz2 \
        ros-jazzy-joy \
        ros-jazzy-cv-bridge \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-tools \
        ros-jazzy-rqt-image-view

    info "Installing system libraries (serial, OpenCV, numpy, etc.)..."
    sudo apt install -y \
        libserial-dev \
        python3-opencv \
        python3-numpy \
        python3-yaml

    info "Initialising rosdep..."
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init
    else
        ok "rosdep already initialised."
    fi
    rosdep update

    ok "ROS packages and system libs installed."
}

# ── Step 5: Clone repo + build workspace ──────────────────────────────────────
setup_workspace() {
    header "Step 5 — resqbot_dev workspace"

    local default_dir="$HOME/resqbot_dev"
    local repo_dir

    read -rp "$(echo -e "${YELLOW}[?]${NC} Where to clone resqbot_dev? [${default_dir}] ")" repo_dir
    repo_dir="${repo_dir:-$default_dir}"

    if [[ -d "$repo_dir/.git" ]]; then
        ok "Repo already cloned at $repo_dir — pulling latest..."
        git -C "$repo_dir" pull --rebase
    else
        info "Cloning resqbot_dev (branch: lotti3)..."
        git clone --branch lotti3 \
            https://github.com/ResQBot/resqbot_dev.git \
            "$repo_dir"
    fi

    local ws_dir="$repo_dir/control_ws"
    info "Running rosdep in $ws_dir ..."
    # Source ROS so rosdep can find Jazzy packages
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    rosdep install --from-paths "$ws_dir/src" --ignore-src -r -y

    info "Building workspace with colcon..."
    cd "$ws_dir"
    colcon build --symlink-install

    ok "Workspace built: $ws_dir"
    echo ""
    warn "NOTE: If the build fails because the Unitree SDK is missing, that is expected."
    warn "      Drive hardware needs to be built separately with:"
    warn "      colcon build --symlink-install --cmake-args -DLOTTI_ENABLE_VENDOR_UNITREE_SDK=ON -DUNITREE_SDK_DIR=/path/to/sdk"
    echo ""

    # Store for bashrc step
    export RESQBOT_WS="$ws_dir"
}

# ── Step 6: .bashrc ───────────────────────────────────────────────────────────
setup_bashrc() {
    header "Step 6 — .bashrc"

    local bashrc="$HOME/.bashrc"
    local ws="${RESQBOT_WS:-$HOME/resqbot_dev/control_ws}"

    append_if_missing() {
        local line="$1"
        if ! grep -qF "$line" "$bashrc"; then
            echo "$line" >> "$bashrc"
            info "Added: $line"
        else
            ok "Already present: $line"
        fi
    }

    append_if_missing "source /opt/ros/jazzy/setup.bash"
    append_if_missing "source ${ws}/install/setup.bash"
    append_if_missing "export ROS_DOMAIN_ID=17"

    ok ".bashrc updated."
    info "Run 'source ~/.bashrc' or open a new terminal to activate."
}

# ── Optional: Livox LiDAR stack ───────────────────────────────────────────────
install_livox_stack() {
    header "Optional — Livox SDK2 + livox_ros_driver2 + Fast_LIO"

    local livox_ws="$HOME/livox_ws"
    mkdir -p "$livox_ws/src"

    # ── Livox SDK2 ──
    info "Cloning Livox-SDK2..."
    if [[ ! -d "$livox_ws/src/Livox-SDK2" ]]; then
        git clone https://github.com/Livox-SDK/Livox-SDK2.git \
            "$livox_ws/src/Livox-SDK2"
    fi

    info "Patching Livox-SDK2 for C++ std compliance..."
    local files=(
        "sdk_core/comm/define.h"
        "sdk_core/logger_handler/file_manager.h"
    )
    for f in "${files[@]}"; do
        local fp="$livox_ws/src/Livox-SDK2/$f"
        if ! grep -q "#include <cstdint>" "$fp"; then
            sed -i '1s/^/#include <cstdint>\n/' "$fp"
            info "Patched $f"
        fi
    done

    info "Building Livox-SDK2 (this takes a while)..."
    mkdir -p "$livox_ws/src/Livox-SDK2/build"
    cmake -S "$livox_ws/src/Livox-SDK2" -B "$livox_ws/src/Livox-SDK2/build"
    make -j"$(nproc)" -C "$livox_ws/src/Livox-SDK2/build"
    sudo make -C "$livox_ws/src/Livox-SDK2/build" install
    ok "Livox-SDK2 installed."

    # ── livox_ros_driver2 ──
    info "Cloning livox_ros_driver2..."
    if [[ ! -d "$livox_ws/src/livox_ros_driver2" ]]; then
        git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
            "$livox_ws/src/livox_ros_driver2"
    fi

    info "Building livox_ros_driver2 for Jazzy..."
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    cd "$livox_ws/src/livox_ros_driver2"
    # The build script supports 'jazzy' directly
    ./build.sh jazzy || {
        warn "build.sh jazzy failed — trying with humble tag (common fallback)..."
        ./build.sh humble
    }

    append_if_missing_bashrc "source ${livox_ws}/install/setup.bash"
    ok "livox_ros_driver2 built."

    # ── Fast_LIO ──
    info "Installing Fast_LIO dependencies (PCL, Eigen3)..."
    sudo apt install -y \
        pcl-tools \
        libpcl-dev \
        ros-jazzy-pcl-ros \
        libeigen3-dev

    local fast_lio_ws="$HOME/fast_lio_ws"
    mkdir -p "$fast_lio_ws/src"

    info "Cloning Fast_LIO_ROS2..."
    if [[ ! -d "$fast_lio_ws/src/FAST_LIO_ROS2" ]]; then
        git clone --recursive \
            https://github.com/Ericsii/FAST_LIO_ROS2.git \
            "$fast_lio_ws/src/FAST_LIO_ROS2"
    fi

    info "Patching Fast_LIO CMakeLists for C++20..."
    sed -i 's/c++14/c++20/g; s/c++17/c++20/g' \
        "$fast_lio_ws/src/FAST_LIO_ROS2/CMakeLists.txt"

    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    source "$livox_ws/install/setup.bash"
    rosdep install --from-paths "$fast_lio_ws/src" --ignore-src -y

    cd "$fast_lio_ws"
    colcon build --symlink-install

    append_if_missing_bashrc "source ${fast_lio_ws}/install/setup.bash"
    ok "Fast_LIO built."
    warn "Edit ${fast_lio_ws}/src/FAST_LIO_ROS2/config/mid360.yaml to set your map_file_path."
    warn "Edit the Livox MID360_config.json to match your host IP (see setup guide)."
}

append_if_missing_bashrc() {
    local line="$1"
    if ! grep -qF "$line" "$HOME/.bashrc"; then
        echo "$line" >> "$HOME/.bashrc"
    fi
}

# ── Optional: audio_common ────────────────────────────────────────────────────
install_audio() {
    header "Optional — audio_common"

    sudo apt install -y \
        libasound2-dev \
        gstreamer1.0-plugins-base \
        libgstreamer1.0-dev \
        liborc-0.4-dev \
        espeak \
        alsa-utils \
        pavucontrol

    local audio_ws="$HOME/ros2_audio_ws"
    mkdir -p "$audio_ws/src"

    if [[ ! -d "$audio_ws/src/audio_common" ]]; then
        git clone -b ros2 \
            https://github.com/ros-drivers/audio_common.git \
            "$audio_ws/src/audio_common"
    fi

    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    rosdep install --from-paths "$audio_ws/src" --ignore-src -r -y
    cd "$audio_ws"
    colcon build --symlink-install

    append_if_missing_bashrc "source ${audio_ws}/install/setup.bash"
    ok "audio_common built."
}

# ── Summary ───────────────────────────────────────────────────────────────────
print_summary() {
    header "Setup complete"
    echo ""
    echo -e "${BOLD}What was installed:${NC}"
    echo "  ✓ ROS 2 Jazzy (desktop)"
    echo "  ✓ Gazebo Harmonic + ros_gz_sim + ros_gz_bridge + gz_ros2_control"
    echo "  ✓ MoveIt2 + moveit_servo"
    echo "  ✓ ros2_control + controllers"
    echo "  ✓ libserial-dev, OpenCV, NumPy"
    echo "  ✓ resqbot_dev workspace cloned + built"
    echo "  ✓ .bashrc configured (ROS source + DOMAIN_ID=17)"
    echo ""
    echo -e "${BOLD}Next steps:${NC}"
    echo "  1. source ~/.bashrc   (or open a new terminal)"
    echo "  2. ros2 launch lotti_control3 gazebo.launch.py"
    echo "  3. ros2 control list_controllers"
    echo ""
    echo -e "${BOLD}For real hardware (robot only):${NC}"
    echo "  - Drive: rebuild with -DLOTTI_ENABLE_VENDOR_UNITREE_SDK=ON"
    echo "  - Flipper: write_only mode until Arduino firmware sends feedback"
    echo "  - Both controllers are disabled by default in robot.launch.py:"
    echo "    ros2 launch lotti_control3 robot.launch.py enable_drive_controller:=true enable_flipper_controller:=true"
    echo ""
    warn "Simulation steps in CLAUDE.md are documentation-level until verified on this machine."
}

# ── Main ──────────────────────────────────────────────────────────────────────
main() {
    echo -e "${BOLD}"
    echo "╔══════════════════════════════════════════════════╗"
    echo "║   Res.Q Bots — Ubuntu 24.04 / ROS 2 Jazzy       ║"
    echo "║   Lotti3 full setup script                       ║"
    echo "╚══════════════════════════════════════════════════╝"
    echo -e "${NC}"

    check_ubuntu_version
    setup_locale
    install_ros2_jazzy
    install_gazebo
    install_ros_packages
    setup_workspace
    setup_bashrc

    echo ""
    if ask "Install Livox LiDAR stack (SDK2 + livox_ros_driver2 + Fast_LIO)?"; then
        install_livox_stack
    else
        info "Skipping Livox LiDAR stack."
    fi

    if ask "Install audio_common (robot microphone / speaker)?"; then
        install_audio
    else
        info "Skipping audio_common."
    fi

    print_summary
}

main "$@"
