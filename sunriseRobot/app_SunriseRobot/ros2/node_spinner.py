import threading

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor


class NodeSpinner:
    """
    Spins one ROS2 node in a background thread, with its own executor, and can stop it again.

    Replaces threading.Thread(target=rclpy.spin, args=(node,)). In ROS2 Foxy rclpy.spin() loops until the whole
    rclpy context shuts down: destroying the node does not end it, so joining that thread blocked forever. A
    listener stopped from the controller loop (lidar toggle) froze the loop, and with it the wheels.
    Shutting down a private executor ends its spin() right away, without touching other nodes.
    """
    STOP_TIMEOUT = 2.0  # seconds

    def __init__(self, node: Node, thread_name: str):
        self.node = node
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, name=thread_name, daemon=True)
        self._thread.start()

    def stop(self) -> bool:
        """
        Stop spinning, then destroy the node. Returns False if the spin thread did not end within the timeout
        (it is a daemon thread, so it cannot keep the process alive). Never blocks for long.
        """
        self._executor.shutdown(timeout_sec=NodeSpinner.STOP_TIMEOUT)
        self._thread.join(timeout=NodeSpinner.STOP_TIMEOUT)
        stopped = not self._thread.is_alive()
        self.node.destroy_node()
        return stopped
