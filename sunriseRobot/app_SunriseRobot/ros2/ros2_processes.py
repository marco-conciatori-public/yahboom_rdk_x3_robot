"""
Background ROS2 launch processes (ROS-TCP-Endpoint, lidar publisher) that the main app starts and stops.

They used to be opened in separate gnome-terminal windows. That needs a desktop session, so it fails silently when
the main app runs as a systemd service, and it could not be stopped reliably: gnome-terminal hands the command to
its own server process, so the app had no handle on it and could only kill processes by name, leaving pieces (and
the console) behind. Here each one is a child process in its own process group, so stopping it signals the launch
process and every node it spawned, together.

Their output goes to the main app's output (the journal, under systemd), prefixed by ros2 launch with the node name.
"""

import os
import atexit
import signal
import threading
import subprocess

ROS2_SETUP = 'source /opt/ros/foxy/setup.bash; source /root/marco_ros2_ws/install/local_setup.bash'


class ManagedProcess:
    """
    A shell command run as a child process in its own process group. start() is idempotent (a second call while
    it runs does nothing), stop() terminates the whole group. Thread-safe.
    """
    # how long to wait after each signal (SIGINT, then SIGTERM) before escalating
    STOP_STEP_TIMEOUT = 5.0

    def __init__(self, name: str, command: str):
        self.name = name
        self.command = command
        self._process = None
        self._lock = threading.Lock()

    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def start(self, verbose: int = 0) -> None:
        with self._lock:
            if self.is_running():
                if verbose >= 2:
                    print(f'{self.name} is already running')
                return
            # start_new_session: own process group (pgid == pid), so stop() can signal the launch process and all
            # the nodes it spawns, without touching the main app
            self._process = subprocess.Popen(['bash', '-c', self.command], start_new_session=True)
            if verbose >= 1:
                print(f'{self.name} started (pid {self._process.pid})')

    def stop(self, verbose: int = 0) -> None:
        with self._lock:
            if self._process is None:
                return
            pgid = self._process.pid
            # SIGINT first: ros2 launch treats it like Ctrl+C and shuts its nodes down cleanly
            for sig in (signal.SIGINT, signal.SIGTERM):
                if self._process.poll() is not None:
                    break
                self._signal_group(pgid, sig)
                try:
                    self._process.wait(timeout=ManagedProcess.STOP_STEP_TIMEOUT)
                except subprocess.TimeoutExpired:
                    if verbose >= 1:
                        print(f'{self.name} did not stop after {signal.Signals(sig).name}')
            # whatever is still alive in the group (including nodes outliving the launch process) is killed
            self._signal_group(pgid, signal.SIGKILL)
            if self._process.poll() is None:
                self._process.wait()
            self._process = None
            if verbose >= 1:
                print(f'{self.name} stopped')

    @staticmethod
    def _signal_group(pgid: int, sig: int) -> None:
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass  # the group is already empty


# `exec` so the launch process replaces bash and receives the signals directly
ROS_TCP_ENDPOINT = ManagedProcess(
    name='ROS-TCP-Endpoint',
    command=f'{ROS2_SETUP}; cd /root/marco_ros2_ws/; exec ros2 launch ros_tcp_endpoint endpoint_launch.py',
)
# shared by everything that needs lidar scans (VR connection, lidar listeners): only one instance can own the lidar
LIDAR_PUBLISHER = ManagedProcess(
    name='Lidar publisher',
    command=f'{ROS2_SETUP}; exec ros2 launch lidar_pub ms200_scan.launch.py',
)


def stop_all(verbose: int = 0) -> None:
    for managed_process in (ROS_TCP_ENDPOINT, LIDAR_PUBLISHER):
        managed_process.stop(verbose=verbose)


# children in their own process group do not get the terminal's Ctrl+C, so on a normal interpreter exit they are
# stopped here (under systemd, stopping the service already kills the whole control group)
atexit.register(stop_all)
