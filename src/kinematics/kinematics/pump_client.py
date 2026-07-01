import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from .Inverse_kinematics import inverse
import math
import csv
from enum import Enum

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String


# ── State Machine ─────────────────────────────────────────
class State(Enum):
    WAITING       = 0
    PICK_APPROACH = 1
    PICK_LIFT     = 2
    DROP_APPROACH = 3
    DROP_LIFT     = 4
    HOMING        = 5


# ── Coordinates ───────────────────────────────────────────
PICK_X, PICK_Y = -0.25,  0.0
DROP_X, DROP_Y = 0.25,  0.0
Z_HOME         = -0.48
Z_APPROACH     = -0.5
Z_GRAB         = -0.7


class deltanode(Node):

    def __init__(self):
        super().__init__('delta_client')

        # ── Action client ─────────────────────────────────
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # ── Subscribe to pick trigger from hardware interface ──
        self.pick_sub = self.create_subscription(
            String,
            '/pick_trigger',
            self.on_pick_trigger,
            10
        )

        # ── Publish pump commands to hardware interface ───
        self.pump_pub = self.create_publisher(String, '/pump_command', 10)

        # ── State ─────────────────────────────────────────
        self.state = State.WAITING

        # ── Logging ───────────────────────────────────────
        self.logged_positions = []
        self.logged_time      = []
        self.start_time       = self.get_clock().now()

        self.get_logger().info('Delta client ready. Waiting for pick trigger...')

    # ─────────────────────────────────────────────────────
    def on_pick_trigger(self, msg):
        if self.state != State.WAITING:
            self.get_logger().warn('Pick trigger ignored — robot is busy.')
            return

        self.get_logger().info('Pick trigger received! Starting pick cycle.')
        self.state = State.PICK_APPROACH
        self.send_trajectory([
            (PICK_X, PICK_Y, Z_APPROACH),  # point 1 — above product
            (PICK_X, PICK_Y, Z_GRAB),       # point 2 — down to product
        ], durations=[1, 2])

    # ─────────────────────────────────────────────────────
    def send_trajectory(self, xyz_list, durations):
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'link_0_JOINT_1',
            'link_0_JOINT_2',
            'link_0_JOINT_3'
        ]

        for xyz, sec in zip(xyz_list, durations):
            x, y, z = xyz
            thetas = inverse(x, y, z)

            if thetas is None:
                self.get_logger().error(f'IK failed for position {xyz}')
                self.state = State.WAITING
                return

            thetas = [-math.radians(t) for t in thetas]

            point = JointTrajectoryPoint()
            point.positions     = thetas
            point.velocities    = [0.0] * 3
            point.accelerations = [0.0] * 3
            point.time_from_start = Duration(sec=sec)
            trajectory.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending trajectory — state: {self.state.name}')

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # ─────────────────────────────────────────────────────
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.state = State.WAITING
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_trajectory_done)

    # ─────────────────────────────────────────────────────
    def on_trajectory_done(self, future):
        self.get_logger().info(f'Trajectory done — state: {self.state.name}')

        if self.state == State.PICK_APPROACH:
            # arrived at product → pump ON → lift up
            self.publish_pump('ON')
            
            self.state = State.PICK_LIFT
            self._pump_timer = self.create_timer(1.5, self._on_pump_ready)
            # self.send_trajectory([
            #     (PICK_X, PICK_Y, Z_APPROACH),  # lift back up
            # ], durations=[1])

        elif self.state == State.PICK_LIFT:
            # lifted with product → move to drop zone
            self.state = State.DROP_APPROACH
            self.send_trajectory([
                (DROP_X, DROP_Y, Z_APPROACH),  # above drop zone
                (DROP_X, DROP_Y, Z_GRAB),       # down to drop zone
            ], durations=[1, 2])

        elif self.state == State.DROP_APPROACH:
            # arrived at drop zone → pump OFF → lift up
            self.publish_pump('OFF')
            self.state = State.DROP_LIFT
            self._release_timer = self.create_timer(1.5, self._on_release_ready)
            # self.send_trajectory([
            #     (DROP_X, DROP_Y, Z_APPROACH),  # lift back up
            # ], durations=[2])

        elif self.state == State.DROP_LIFT:
            # lifted → return home
            self.state = State.HOMING
            self.send_trajectory([
                (0.0, 0.0, Z_HOME),            # home position
            ], durations=[1])

        elif self.state == State.HOMING:
            # cycle complete
            self.state = State.WAITING
            self.get_logger().info('Cycle complete. Waiting for next pick trigger.')
            self.save_to_csv()

    def _on_pump_ready(self):
        self.destroy_timer(self._pump_timer)  # one shot
        self.send_trajectory([
            (PICK_X, PICK_Y, Z_APPROACH),
        ], durations=[1])

    def _on_release_ready(self):
        self.destroy_timer(self._release_timer)  # one shot
        self.send_trajectory([
            (DROP_X, DROP_Y, Z_APPROACH),  # lift back up
        ], durations=[1])    

    # ─────────────────────────────────────────────────────
    def publish_pump(self, state):
        msg = String()
        msg.data = f'PUMP_{state}'
        self.pump_pub.publish(msg)
        self.get_logger().info(f'Pump command: PUMP_{state}')

    # ─────────────────────────────────────────────────────
    def feedback_callback(self, feedback_msg):
        feedback   = feedback_msg.feedback
        positions  = list(feedback.actual.positions)
        velocities = list(feedback.actual.velocities)
        t          = (self.get_clock().now() - self.start_time).nanoseconds

        self.logged_positions.append(positions)
        self.logged_time.append(t)

        self.get_logger().info(
            f"t_ns={t}  q={positions}  v={velocities}"
        )

    # ─────────────────────────────────────────────────────
    def save_to_csv(self):
        filename = 'delta_joint_log.csv'

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time_nanoseconds', 'joint_1', 'joint_2', 'joint_3'])
            for t, q in zip(self.logged_time, self.logged_positions):
                writer.writerow([t, q[0], q[1], q[2]])

        self.get_logger().info(f'Saved trajectory log to {filename}')


# ─────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    delta = deltanode()
    rclpy.spin(delta)
    rclpy.shutdown()


if __name__ == '__main__':
    main()