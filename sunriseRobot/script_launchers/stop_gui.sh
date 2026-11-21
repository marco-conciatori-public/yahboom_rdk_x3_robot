# This command stops the graphical user interface and switches to a multi-user target.
# The main app runs as robot_control.service (part of multi-user.target), so it keeps running: it must not be
# started again here, or two instances would fight over the serial port and GPIO.
systemctl isolate multi-user.target
exit 0
