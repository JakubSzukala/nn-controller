import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from stable_baselines3 import PPO
from simple_pid import PID
# from std_msgs.msg import Float32 #test
from sensor_msgs.msg import PointCloud2 #test
from autoware_control_msgs.msg import Control

class NN_ControllerNode(Node):
    def __init__(self):
        super().__init__('lidar_controller_node')
        self.subscribeLidar = self.create_subscription(
            LaserScan,
            '/sensing/lidar/pointcloud',
            self.nn_control_callback,
            10
        )
        self.prev_lidar_scans = []
        self.publisher = self.create_publisher(Control, '/control/command/control_cmd', 10)
        self.model = PPO.load('path_to_your_model')
        #self.pidSte = PID(Kp=1.0, Ki=0.1, Kd=0.05)
        #self.pidVel = PID(Kp=1.0, Ki=0.1, Kd=0.05)  # wartości do ustawienia


    def nn_control_callback(self, msg):
        self.prev_lidar_scans.append(msg.fields)
        if len(self.prev_lidar_scans)>5:
            self.prev_lidar_scans.pop(0)

        input_data = self.prev_lidar_scans[-1] # jaki format punktów przyjmuje nn?
        output = self.model.predict(input_data)


        control_msg = Control()
        control_msg.Lateral.steering_tire_angle = output[0] #???
        control_msg.Longitudinal.velocity = output[1] #???

        # param1 = output[0]
        # param2 = output[1]
        # control_signal = self.pidSte(param1)
        # control_signal = self.pidVel(param2)

        self.publisher.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_controller_node = NN_ControllerNode()
    rclpy.spin(lidar_controller_node)
    lidar_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

