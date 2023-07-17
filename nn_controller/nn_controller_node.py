#!/usr/bin/env python3
import numpy as np
from stable_baselines3 import PPO
import rclpy
from rclpy.node import Node
from rclpy.qos import * #QoSProfile
from sensor_msgs.msg import LaserScan
from autoware_auto_control_msgs.msg import AckermannControlCommand

class NNControllerNode(Node):
    def __init__(self):
        super().__init__('nn_controller')

        # Load model and extract policy
        self.model_path = 'src/nn_controller/models/best_model.zip'
        model = PPO.load(self.model_path)
        self.policy = model.policy
        del model

        self.lidar_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic='/sensing/lidar/scan',
            callback=self.control_callback,
            qos_profile=QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )
        self.publisher = self.create_publisher(
            AckermannControlCommand,
            '/control/command/control_cmd',
            qos_profile=QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )


    def control_callback(self, msg: LaserScan):
        self.get_logger().info("Received laser scan")
        observations = np.array(msg.ranges)
        observations = observations / msg.range_max # Normalize
        # Filter infs and -infs
        mask = np.isin(observations, np.inf)
        observations[mask] = msg.range_max
        mask = np.isin(observations, [-np.inf, np.nan])
        observations[mask] = 0

        # Reverse order
        observations = observations[::-1]

        # Predict
        action = self.policy.predict(observations, deterministic=True)

        console_log = "Actions taken: {}, {}".format(action[0][0], action[0][1])
        self.get_logger().info(console_log)

        # Act
        control_msg = AckermannControlCommand()
        control_msg.longitudinal.acceleration = float(action[0][0])
        control_msg.lateral.steering_tire_angle = float(action[0][1])

        self.publisher.publish(control_msg)


def main(args=None):
    print("Starting nn_controller_node...")
    rclpy.init(args=args)
    lidar_controller_node = NNControllerNode()
    rclpy.spin(lidar_controller_node)
    lidar_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

