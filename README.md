# Minimal ros2 node in r2r

This package shows how to use r2r to create a minimal ros node.

``` sh
git clone [this repo]
colcon build
install/setup.sh
ros2 run r2r_minimal_node r2r_minimal_node --ros-args -p param_key:=[hej,hopp] -p key2:=5.5 -r __ns:=/demo -r __node:=my_node
```
