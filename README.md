# Minimal ros2 node using r2r

This package shows how to use <https://github.com/sequenceplanner/r2r> to create a minimal ros node. Some custom messages are defined in the ros package `r2r_minimal_node_msgs`, while `r2r_minimal_node` implements a node with a simple service that uses the custom messages.

## Using colcon to build

``` sh
mkdir src
cd src
git clone [this repo]
cd ..
colcon build
. install/setup.sh
ros2 run r2r_minimal_node r2r_minimal_node --ros-args -p param1:="a string" -p param2:=5.5 -p param3:=true -r __ns:=/demo -r __node:=my_node
[ctrl-z]
bg
ros2 service call /hello_world r2r_minimal_node_msgs/srv/HelloWorld '{ hello: "Hello" }'
```

The integration with colcon is just a cmake hack that calls cargo, see CMakeLists.txt. Note the `r2r_cargo` function call. **Note that an empty `dummy.c`** is required in the crate root. To keep builds performed by colcon separate, `r2r_cargo` **expects a custom profile called `colcon` to exist**. See <https://github.com/m-dahl/r2r_minimal_node/blob/master/r2r_minimal_node/Cargo.toml> for an example.

cargo clean can be invoked by passing `-DCARGO_CLEAN=ON` to cmake, eg.
```
colcon build --cmake-args -DCARGO_CLEAN=ON
```

### Note on cmake and Ubuntu 20.04
The `r2r_cargo` function requires cmake version 3.21 or newer, but
ubuntu 20.04 ships with cmake 3.16.3. You can update to a newer cmake
version by adding the kitware apt repo: <https://apt.kitware.com>.

## Building using the Rust plugins for the ROS buildtools.
It is also possible to build using the Rust plugins for the ROS buildtools.

To install the Rust plugins for the ROS buildtools:
```sh
cargo install --debug cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```

`r2r_minimal_node_colcon_plugin` is configured to be built using the rust plugins. To only build one version of the minimal node package, this directory has a `COLCON_IGNORE` file in it. Move it to `r2r_minimal_node` to skip that package instead.

```sh
mkdir src
cd src
git clone [this repo]
mv r2r_minimal_node_colcon_plugin/COLCON_IGNORE r2r_minimal_node/
cd ..
colcon build --clean-build --cargo-args --release
```

## Building using only cargo

Another option is to build the message package first, then sourcing the resulting workspace. When `r2r_minimal_node_msgs` is sourced, the r2r build script will automatically pick up the custom messages (it defaults to building everything).

To avoid building everything, it is possible to declare only the messages needed using the environment variable `IDL_PACKAGE_FILTER`. Setting this can be done in `.cargo/config.toml` for convenience, e.g. <https://github.com/m-dahl/r2r_minimal_node/blob/master/r2r_minimal_node/.cargo/config.toml>

``` sh
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
