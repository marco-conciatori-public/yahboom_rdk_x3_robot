import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import utils
from ros2 import ros2_processes
from ros2.node_spinner import NodeSpinner


class LidarListener(Node):
    def __init__(self,
                 topic_name: str,
                 queue_size: int,
                 response_dist: float = 12.0,
                 min_response_dist: float = 0.0,
                 front_arc: list = None,
                 rear_arc: list = None,
                 rear_min_response_dist: float = None,
                 sector_angle: float = None,
                 search_only_arc: list = None,
                 scan_expiration_time: float = 0.5,
                 ):
        super().__init__('lidar_listener')
        assert sector_angle or search_only_arc is not None, \
            'Either "sector_angle" or "search_only_arc" must be provided'
        assert sector_angle or search_only_arc is None, \
            'Only one of "sector_angle" or "search_only_arc" can be provided'
        self.subscription = self.create_subscription(
            LaserScan,
            topic_name,
            self.lidar_scan_callback,
            queue_size
        )
        self.lidar_data = None
        self.scan_timestamp = 0
        self.response_dist = response_dist
        # Any return closer than this is treated as a self-hit from the robot frame (or an invalid reading)
        # and ignored. NaN/inf also fail this comparison, so they get filtered out as a side effect.
        # When front_arc is set, this threshold is only applied to angles OUTSIDE the front arc — front
        # readings pass through with no minimum, since the lidar's front is unobstructed.
        self.min_response_dist = min_response_dist
        # [low_deg, high_deg] (in the same convention as scan_data.angle_min). Wrap-around supported when
        # low > high (e.g., [330, 30] covers the 60° arc that crosses 0°). None disables the feature.
        self.front_arc = front_arc
        # Arc behind the lidar where the robot's own parts sit farther than min_response_dist: there, returns
        # closer than rear_min_response_dist are ignored instead. Same angle convention as front_arc. None
        # disables it (rear_min_response_dist is then unused).
        self.rear_arc = rear_arc
        self.rear_min_response_dist = rear_min_response_dist
        self.scan_expiration_time = scan_expiration_time

        # sector_angle attributes
        self.sector_angle = sector_angle
        self.number_of_sectors = int(360 / sector_angle) if sector_angle else 0
        self.obstacles_by_sector = np.zeros(shape=self.number_of_sectors, dtype=bool)
        self.hit_counter_by_sector = np.zeros(shape=self.number_of_sectors, dtype=int)
        self.average_distance_by_sector = np.zeros(shape=self.number_of_sectors, dtype=float)

        # search_only_arc attributes
        self.search_only_arc = search_only_arc
        self.obstacle_in_arc = False
        self.average_distance_in_arc = 0

        # narrower sectors (smaller angles) means less hits are required to detect an obstacle
        # self.obstacle_found_threshold = 5
        if sector_angle is not None:
            self.obstacle_found_threshold = int(sector_angle / 4)
        elif search_only_arc is not None:
            # self.obstacle_found_threshold = int(search_only_arc[1] - search_only_arc[0] / 4)
            self.obstacle_found_threshold = int((search_only_arc[1] - search_only_arc[0]) / 4)

    def lidar_scan_callback(self, msg: LaserScan) -> None:
        # self.get_logger().info('Published processed lidar data')
        self.lidar_data = msg
        self.scan_timestamp = time.time()
        if self.sector_angle is not None:
            self.search_obstacles(msg)
        elif self.search_only_arc:
            self.scan_arc(msg)

    def _is_in_front_arc(self, angle_deg: float) -> bool:
        return self._is_in_arc(angle_deg=angle_deg, arc=self.front_arc)

    @staticmethod
    def _is_in_arc(angle_deg: float, arc: list) -> bool:
        if arc is None:
            return False
        low, high = arc
        if high - low >= 360:
            return True
        a = angle_deg % 360
        low_n = low % 360
        high_n = high % 360
        if low_n <= high_n:
            return low_n <= a <= high_n
        return a >= low_n or a <= high_n

    def _effective_min_dist(self, angle_deg: float) -> float:
        # Front of the lidar is unobstructed: accept any positive return.
        # Rear arc: the robot's own parts reach farther there, so a larger minimum.
        # Sides: enforce min_response_dist to drop the robot's own frame.
        if self._is_in_front_arc(angle_deg):
            return 0.0
        if self.rear_arc is not None and self._is_in_arc(angle_deg=angle_deg, arc=self.rear_arc):
            return self.rear_min_response_dist
        return self.min_response_dist

    def search_obstacles(self, scan_data: LaserScan) -> None:
        if not isinstance(scan_data, LaserScan):
            return
        self.hit_counter_by_sector[:] = 0
        temp_average_distance_by_sector = np.zeros(shape=self.number_of_sectors, dtype=float)
        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            angle = np.rad2deg(scan_data.angle_min + scan_data.angle_increment * i)
            # Reject returns from the robot's own frame (too close, sides/back only) and
            # returns beyond our area of interest (too far). NaN/inf fail both.
            if self._effective_min_dist(angle) < ranges[i] < self.response_dist:
                # assert 0 <= angle <= 360, f'Angle {angle} is out of range [0, 360]'
                sector_num = int(angle / self.sector_angle)
                self.hit_counter_by_sector[sector_num] += 1
                temp_average_distance_by_sector[sector_num] += ranges[i]

        for sector_num in range(len(self.obstacles_by_sector)):
            if self.hit_counter_by_sector[sector_num] > self.obstacle_found_threshold:
                self.obstacles_by_sector[sector_num] = True
                self.average_distance_by_sector[sector_num] = (temp_average_distance_by_sector[sector_num] /
                                                               self.hit_counter_by_sector[sector_num])
            else:
                self.obstacles_by_sector[sector_num] = False
                self.average_distance_by_sector[sector_num] = -1

    def scan_arc(self, scan_data: LaserScan) -> None:
        if not isinstance(scan_data, LaserScan):
            return
        hit_counter = 0
        temp_average_distance = 0
        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            angle = np.rad2deg(scan_data.angle_min + scan_data.angle_increment * i)
            # Reject returns from the robot's own frame (too close, sides/back only) and
            # returns beyond our area of interest (too far). NaN/inf fail both.
            if self._effective_min_dist(angle) < ranges[i] < self.response_dist:
                if self.search_only_arc[0] <= angle <= self.search_only_arc[1]:
                    hit_counter += 1
                    temp_average_distance += ranges[i]

        if hit_counter > self.obstacle_found_threshold:
            self.obstacle_in_arc = True
            self.average_distance_in_arc = temp_average_distance / hit_counter
        else:
            self.obstacle_in_arc = False
            self.average_distance_in_arc = -1

    def get_raw_scan(self) -> LaserScan:
        if (time.time() - self.scan_timestamp) < self.scan_expiration_time:
            return self.lidar_data
        return None

    def get_obstacles_by_sector(self) -> tuple:
        if (time.time() - self.scan_timestamp) < self.scan_expiration_time:
            return self.obstacles_by_sector, self.average_distance_by_sector
        return None, None

    def get_obstacle_in_arc(self) -> tuple:
        if (time.time() - self.scan_timestamp) < self.scan_expiration_time:
            return self.obstacle_in_arc, self.average_distance_in_arc
        return None, None


class ThreadedLidarListener:
    def __init__(self,
                 topic_name: str = '/scan',
                 queue_size: int = 5,
                 response_dist: float = 12.0,
                 min_response_dist: float = 0.0,
                 front_arc: list = None,
                 rear_arc: list = None,
                 rear_min_response_dist: float = None,
                 sector_angle: float = 20,
                 search_only_arc: list = None,
                 scan_expiration_time: float = 0.5,
                 verbose: int = 0,
                 ):
        self.topic_name = topic_name
        self.queue_size = queue_size
        self.sector_angle = sector_angle
        self.response_dist = response_dist
        self.min_response_dist = min_response_dist
        self.front_arc = front_arc
        self.lidar_listener_node = None
        self.spinner = None
        self.verbose = verbose
        try:
            if not rclpy.ok():
                rclpy.init()
            self.lidar_listener_node = LidarListener(
                topic_name=topic_name,
                queue_size=queue_size,
                response_dist=response_dist,
                min_response_dist=min_response_dist,
                front_arc=front_arc,
                rear_arc=rear_arc,
                rear_min_response_dist=rear_min_response_dist,
                sector_angle=sector_angle,
                search_only_arc=search_only_arc,
                scan_expiration_time=scan_expiration_time,
            )
            # Spin the node in a separate thread (stoppable, see NodeSpinner)
            self.spinner = NodeSpinner(node=self.lidar_listener_node, thread_name='lidar_listener')
            if self.verbose >= 1:
                print('Lidar listener created and listening')

            # shared publisher: does nothing if the VR connection or another listener already started it
            ros2_processes.LIDAR_PUBLISHER.start(verbose=self.verbose)

        except Exception as e:
            utils.print_exception(exception=e, message='Lidar listener creation error')
            try:
                if self.lidar_listener_node:
                    self.lidar_listener_node.destroy_node()
            except:
                pass

    def get_raw_scan(self) -> LaserScan:
        if self.lidar_listener_node is not None:
            return self.lidar_listener_node.get_raw_scan()
        if self.verbose >= 2:
            print('Lidar listener node is None')
        return None

    def get_obstacles_by_sector(self) -> tuple:
        if self.lidar_listener_node is not None:
            return self.lidar_listener_node.get_obstacles_by_sector()
        if self.verbose >= 2:
            print('Lidar listener node is None')
        return None, None

    def get_obstacle_in_arc(self) -> tuple:
        if self.lidar_listener_node is not None:
            return self.lidar_listener_node.get_obstacle_in_arc()
        if self.verbose >= 2:
            print('Lidar listener node is None')
        return None, None

    def delete_listener(self):
        # idempotent: also called by __del__ after an explicit call
        if self.spinner is not None:
            stopped = self.spinner.stop()
            self.spinner = None
            self.lidar_listener_node = None
            if not stopped:
                print('Lidar listener spin thread did not stop in time')
            elif self.verbose >= 2:
                print('Lidar listener stopped')
        else:
            if self.verbose >= 2:
                print('Lidar listener not stopped, it is already stopped')

    def __del__(self):
        self.delete_listener()
