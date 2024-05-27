# Minimal ros2 node using r2r

This package shows how to use <https://github.com/sequenceplanner/r2r> to create a minimal ros node. Some custom messages are defined in the ros package `r2r_minimal_node_msgs`, while `r2r_minimal_node` implements a node with a simple service that uses the custom messages.

## Using colcon to build

Install the Rust plugins for the ROS buildtools.

```sh
cargo install --debug cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```

```sh
mkdir src
cd src
git clone [this repo]
cd ..
colcon build --clean-build --cargo-args --release
. install/setup.sh
ros2 run r2r_minimal_node r2r_minimal_node --ros-args -p param1:="a string" -p param2:=5.5 -p param3:=true -r __ns:=/demo -r __node:=my_node
[ctrl-z]
bg
ros2 service call /hello_world r2r_minimal_node_msgs/srv/HelloWorld '{ hello: "Hello" }'
```

## Building using only cargo

Another option is to build the message package first, then sourcing the resulting workspace. When `r2r_minimal_node_msgs` is sourced, the r2r build script will automatically pick up the custom messages (it defaults to building everything).

To avoid building everything, it is possible to declare only the messages needed using the environment variable `IDL_PACKAGE_FILTER`. Setting this can be done in `.cargo/config.toml` for convenience, e.g. <https://github.com/m-dahl/r2r_minimal_node/blob/master/r2r_minimal_node/.cargo/config.toml>

```sh
mkdir src
cd src
git clone [this repo]
cd ..
colcon build --packages-select r2r_minimal_node_msgs # note that we only build the messages here
. install/setup.sh
cd src/r2r_minimal_node/r2r_minimal_node
cargo run -- --ros-args  -p param1:="a string" -p param2:=5.5 -p param3:=true
[ctrl-z]
bg
ros2 service call /hello_world r2r_minimal_node_msgs/srv/HelloWorld '{ hello: "Hello" }'
```
