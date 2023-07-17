# NN controller
ROS2 node that performs inference with trained model to control a toy car in a [simulation](https://github.com/amadeuszsz/AWSIM/tree/v1.1.0_f1tenth). It's intended use is with a model trained in [this](https://github.com/JakubSzukala/self-driving-car-simulator) environment.

#### Description
The node expects model to be trained and exported with Stable Baselines 3. Model is fed distance measurements subscribed from the car's lidar sensor topic and performs inference returning wheels' turn angle and acceleration that then is published to /control/command/control_cmd topic.

#### Usage
To run the node:
```bash
# Install stable baselines 3 dependency, preferably version 1.8.0
$ pip3 install stable-baselines3==1.8.0

# Launch all the infrastructure
$ ros2 launch f1tenth_launch f1tenth.launch.py

# In separate terminal run a test environment
$ ./F1Tenth_v0.5.x86_64

# Then run the nn controller node to perform inference
$ ros2 run nn_controller nn_controller_node
```
