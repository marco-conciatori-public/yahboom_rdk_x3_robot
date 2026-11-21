#! /bin/bash

###############################################################################
# Install the robot's systemd services (sunriseRobot/systemd/*.service) and
# disable the old desktop autostart entries they replace.
#
# Usage (on the robot):
#   sudo bash install_services.sh
#
# Idempotent: re-run it after changing a .service file in the repo, the units
# are copied, not linked.
###############################################################################

set -e

if [ "$EUID" -ne 0 ]; then
  echo "This script must be run as root (use sudo)."
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DIR="$SCRIPT_DIR/../systemd"
SERVICES="robot_control.service robot_fan.service"

# 1) disable the desktop autostart entries that launch the main app or the fan,
#    otherwise a desktop login would start a second instance next to the service.
#    Moved (not deleted) to a backup folder, found by content so the file names
#    on the robot do not matter.
BACKUP_DIR=/root/.config/autostart_disabled
for AUTOSTART_DIR in /root/.config/autostart /etc/xdg/autostart; do
  [ -d "$AUTOSTART_DIR" ] || continue
  for ENTRY in "$AUTOSTART_DIR"/*.desktop; do
    [ -f "$ENTRY" ] || continue
    if grep -q -e 'main_thread.py' -e 'start_robot_control' -e 'physical_accessories/fan.py' "$ENTRY"; then
      mkdir -p "$BACKUP_DIR"
      mv "$ENTRY" "$BACKUP_DIR/"
      echo "Disabled autostart entry: $ENTRY (moved to $BACKUP_DIR)"
    fi
  done
done

# 2) install and enable the services
for SERVICE in $SERVICES; do
  cp "$UNIT_DIR/$SERVICE" /etc/systemd/system/
  echo "Installed /etc/systemd/system/$SERVICE"
done
systemctl daemon-reload
systemctl enable $SERVICES

echo ""
echo "============================================================"
echo "Services installed and enabled, they start at the next boot."
echo "To start them now instead (first close any main_thread.py"
echo "already running, e.g. its gnome-terminal window):"
echo "  systemctl start robot_fan robot_control"
echo ""
echo "  status:       systemctl status robot_control"
echo "  live output:  journalctl -u robot_control -f"
echo "  restart:      systemctl restart robot_control"
echo ""
echo "If the robot still starts a second copy at boot, look for other"
echo "launchers (crontab -l, /etc/rc.local, the original Yahboom app)."
echo "============================================================"
