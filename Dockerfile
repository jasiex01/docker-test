FROM ros:jazzy-ros-base-noble
# Make sure bash catches errors (no need to chain commands with &&, use ; instead)
SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]

# 1.) Bring system up to the latest ROS Jazzy configuration
RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get -y install ros-jazzy-ros-base

# 5.) Install ROS1
RUN apt install curl
RUN curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros1.list
RUN echo "# deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main-dbg" | sudo tee -a /etc/apt/sources.list.d/ros1.list
RUN apt-get -y install software-properties-common
RUN apt-get -y update
RUN apt-get -y upgrade
RUN echo "yaml https://ros.packages.techfak.net/ros-one.yaml ubuntu" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-one.list
RUN rosdep update
RUN apt-get -y install ros-one-desktop

# 6.) Compile custom ROS1 and ROS2 messages
RUN git clone https://github.com/LeoRover/leo_common;                 \
    git clone https://github.com/LeoRover/leo_common-ros2;            \
    # Compile ROS1 msgs:                                              \
    mkdir -p /ros1_ws/src;                                            \
    cp -r /leo_common/leo_msgs /ros1_ws/src;                          \
    cd /ros1_ws;                                                      \
    source /opt/ros/one/setup.bash;                                   \
    catkin_make_isolated --install;                                   \
    # Compile ROS2 msgs:                                              \
    unset ROS_DISTRO;                                                 \
    mkdir -p /ros2_ws/src;                                            \
    cp -r /leo_common-ros2/leo_msgs /ros2_ws/src;                     \
    source /opt/ros/jazzy/setup.bash;                                 \
    cd /ros2_ws;                                                      \
    colcon build --packages-select leo_msgs; 

# 7.) Compile ros1_bridge
# g++-11 and needed
RUN apt -y install g++-11 gcc-11
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 11
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 11

RUN                                                                                     \
     # Get the Bridge code                                                              \
     mkdir -p /ros-jazzy-ros1-bridge/src;                                               \
     cd /ros-jazzy-ros1-bridge/src;                                                     \
     git clone -b action_bridge_humble https://github.com/smith-doug/ros1_bridge.git;   \
     cd ros1_bridge/;                                                                   \
                                                                                        \
     # Apply the ROS1 and ROS2 underlays                                                \
     source /opt/ros/one/setup.bash;                                                    \
     source /opt/ros/jazzy/setup.bash;                                                  \
     source /ros1_ws/install_isolated/setup.bash;                                       \
     source /ros2_ws/install/local_setup.bash;                                          \
                                                                                        \
     # Finally, build the Bridge                                                        \
     MEMG=$(printf "%.0f" $(free -g | awk '/^Mem:/{print $2}'));                        \
     NPROC=$(nproc);  MIN=$((MEMG<NPROC ? MEMG : NPROC));                               \
     cd /ros-jazzy-ros1-bridge/;                                                        \
     echo "Please wait...  running $MIN concurrent jobs to build ros1_bridge";          \
     time ROS_DISTRO=humble MAKEFLAGS="-j $MIN" colcon build                            \
       --event-handlers console_direct+                                                 \
       --cmake-args -DCMAKE_BUILD_TYPE=Release

# 8.) Clean up
RUN apt-get -y clean all; apt-get -y update

# 9.) Pack all ROS1 dependent libraries
RUN unset ROS_DISTRO;                                                           \
    ROS1_LIBS="libactionlib.so";                                                \
    ROS1_LIBS="$ROS1_LIBS libroscpp.so";                                        \
    ROS1_LIBS="$ROS1_LIBS librosconsole.so";                                    \
    ROS1_LIBS="$ROS1_LIBS libroscpp_serialization.so";                          \
    ROS1_LIBS="$ROS1_LIBS librostime.so";                                       \
    ROS1_LIBS="$ROS1_LIBS libxmlrpcpp.so";                                      \
    ROS1_LIBS="$ROS1_LIBS libcpp_common.so";                                    \
    ROS1_LIBS="$ROS1_LIBS librosconsole_log4cxx.so";                            \
    ROS1_LIBS="$ROS1_LIBS librosconsole_backend_interface.so";                  \
    ROS1_LIBS="$ROS1_LIBS liblog4cxx.so.15";                                    \
    ROS1_LIBS="$ROS1_LIBS libaprutil-1.so.0";                                   \
    ROS1_LIBS="$ROS1_LIBS libapr-1.so.0";                                       \
    cd /ros-jazzy-ros1-bridge/install/ros1_bridge/lib;                          \
    source /opt/ros/one/setup.bash;                                             \
    for soFile in $ROS1_LIBS; do                                                \
      soFilePath=$(ldd libros1_bridge.so | grep $soFile | awk '{print $3;}');   \
      cp -L $soFilePath ./;                                                     \
    done

# 10.) Spit out ros1_bridge tarball by default when no command is given
RUN tar czf /ros-jazzy-ros1-bridge.tgz \
     --exclude '*/build/*' --exclude '*/src/*' /ros-jazzy-ros1-bridge 
ENTRYPOINT []
CMD cat /ros-jazzy-ros1-bridge.tgz; sync
