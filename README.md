# Minimal ros2 node in r2r

This package shows how to use r2r to create a minimal ros node.

``` sh
mkdir src
cd src
git clone [this repo]
cd ..
colcon build
install/setup.sh
ros2 run r2r_minimal_node r2r_minimal_node --ros-args -p param1:=[str1,str2] -p param2:=5.5 param3=true -r __ns:=/demo -r __node:=my_node
```

The integration with colcon is just a cmake hack that calls cargo, see CMakeLists.txt.

cargo clean can be invoked by passing `-DCARGO_CLEAN=ON` to cmake, eg.
```
colcon build --cmake-args -DCARGO_CLEAN=ON
```
