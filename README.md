To run the node:
```bash
$ ros2 launch f1tenth_launch f1tenth.launch.py

# In separate terminal run a test environment
$ ./F1Tenth_v0.5.x86_64

# Then run the nn controller node to perform inference
$ ros2 run nn_controller nn_controller_node
```
