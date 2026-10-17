#!/usr/bin/env bash
#
# Install the Micro-XRCE-DDS-Agent and run it as a service against the FC's TELEM1.
#
# Needed on any board WITHOUT ARK OS (e.g. the Orin NX on stock JetPack). ARK OS
# shipped this as `dds-agent.service`; a stock image has nothing, so PX4's
# uxrce_dds_client sits at "Running, disconnected" forever and no /fmu/* topics appear.
#
# Not in apt for Humble -- must be built from source. Build runs as you; only the
# install/service steps use sudo.
#
# Usage:  bash scripts/px4/xrce_agent_setup.sh
# Env:    AGENT_DIR, XRCE_DEV, XRCE_BAUD, XRCE_TAG
#
set -eo pipefail

AGENT_DIR="${AGENT_DIR:-$HOME/Micro-XRCE-DDS-Agent}"
XRCE_DEV="${XRCE_DEV:-/dev/ttyTHS1}"      # Jetson side of the FC's TELEM1
XRCE_BAUD="${XRCE_BAUD:-3000000}"         # must equal PX4's SER_TEL1_BAUD
XRCE_TAG="${XRCE_TAG:-v2.4.3}"            # 2.4.x builds against Fast DDS 2.x, as Humble uses

echo "==> [1/5] sanity checks"
[ -e "$XRCE_DEV" ] || { echo "ERROR: $XRCE_DEV does not exist."; exit 1; }
if systemctl list-units --all 2>/dev/null | grep -qiE "getty.*$(basename "$XRCE_DEV")"; then
    echo "ERROR: a getty owns $XRCE_DEV. Disable it first (sudo systemctl disable --now nvgetty)."
    exit 1
fi
# A second reader corrupts the stream -- see docs/uart_bringup_log.md.
if sudo lsof "$XRCE_DEV" >/dev/null 2>&1; then
    echo "ERROR: $XRCE_DEV is already open by:"; sudo lsof "$XRCE_DEV"; exit 1
fi

echo "==> [2/5] clone + build agent $XRCE_TAG (slow: it superbuilds Fast DDS)"
[ -d "$AGENT_DIR" ] || git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR"
git -C "$AGENT_DIR" fetch -q --tags
git -C "$AGENT_DIR" checkout -q "$XRCE_TAG"
mkdir -p "$AGENT_DIR/build"
( cd "$AGENT_DIR/build" && cmake -DCMAKE_BUILD_TYPE=Release .. >/dev/null && make -j"$(nproc)" )

echo "==> [3/5] install"
( cd "$AGENT_DIR/build" && sudo make install )
sudo ldconfig /usr/local/lib/
command -v MicroXRCEAgent

echo "==> [4/5] dialout membership (for running the agent by hand)"
id -nG | tr ' ' '\n' | grep -qx dialout || {
    sudo usermod -aG dialout "$USER"
    echo "    added $USER to dialout -- LOG OUT AND BACK IN for it to apply to your shell."
    echo "    (the systemd service runs as root, so it works regardless)"
}

echo "==> [5/5] systemd service"
sudo tee /etc/systemd/system/xrce-agent.service > /dev/null <<EOF
[Unit]
Description=Micro XRCE-DDS Agent (PX4 uXRCE-DDS <-> ROS 2 bridge)
After=network.target

[Service]
Type=simple
# Domain 0 and the default RMW, matching the rest of the stack (scripts/laptop_viewer.sh).
Environment=ROS_DOMAIN_ID=0
ExecStart=/usr/local/bin/MicroXRCEAgent serial --dev ${XRCE_DEV} -b ${XRCE_BAUD}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now xrce-agent.service
sleep 3
systemctl is-active xrce-agent.service

cat <<'EOF'

Done. Verify -- in this order:

  systemctl status xrce-agent --no-pager
  python3 scripts/px4/px4_shell.py "uxrce_dds_client status"   # want: Running, CONNECTED
  ros2 topic list | grep fmu                                   # want: /fmu/in/*, /fmu/out/*

If it stays "Running, disconnected", the agent is fine but the link is not:
  - Is TELEM1 physically wired to this Jetson's UART? UXRCE_DDS_CFG=101 only proves the
    FC is *configured* for TELEM1, not that the harness connects those pins.
  - Does the baud match? PX4's SER_TEL1_BAUD must equal XRCE_BAUD (3000000 here).
  - Do NOT open the device to "test" it while the agent runs -- that corrupts the stream.
EOF
