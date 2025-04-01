FROM ros:jazzy-ros-base-noble
# The above base image is multi-platform (works on ARM64 and AMD64):
# Docker will automatically select the correct platform variant based on the host's architecture.

#
# How to build this docker image:
#  docker build . -t ros-jazzy-ros1-bridge-builder
#
# How to build ros-jazzy-ros1-bridge:
#  # 0.) From a Ubuntu 24.04 (Noble) ROS 2 Jazzy system, create a "ros-jazzy-ros1-bridge/" ROS2 package:
#    docker run --rm ros-jazzy-ros1-bridge-builder | tar xvzf -
#
# How to use the ros-jazzy-ros1-bridge:
#  # 1.) First start a ROS1 Noetic docker and bring up a GUI terminal, something like:
#    rocker --x11 --user --privileged \
#         --volume /dev/shm /dev/shm --network=host -- ros:noetic-ros-base-focal \
#         'bash -c "sudo apt update; sudo apt install -y ros-noetic-rospy-tutorials tilix; tilix"'
#
#  # 2.) Then, start "roscore" inside the ROS1 container:
#    source /opt/ros/noetic/setup.bash
#    roscore
#
#  # 3.) Now, from the Ubuntu 24.04 (Noble) ROS2 Desktop Jazzy system, start the ros1 bridge node:
#    apt-get -y install ros-jazzy-desktop
#    source /opt/ros/jazzy/setup.bash
#    source ros-jazzy-ros1-bridge/install/local_setup.bash
#    ros2 run ros1_bridge dynamic_bridge
#
#  # 4.) Back to the ROS1 Noetic container, run in another terminal tab:
#    source /opt/ros/noetic/setup.bash
#    rosrun rospy_tutorials talker
#
#  # 5.) Finally, from the Ubuntu 24.04 (Noble) ROS2 Jazzy system:
#    source /opt/ros/jazzy/setup.bash
#    ros2 run demo_nodes_cpp listener
#

# Make sure bash catches errors (no need to chain commands with &&, use ; instead)
SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]


###########################
# 1.) Bring system up to the latest ROS desktop configuration
###########################
RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get -y install ros-jazzy-ros-base


###########################
# 5.) Install ROS1 Noetic desktop
# (Currently, ppa contains AMD64 builds only)
###########################
RUN apt install curl
RUN curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros1.list
RUN echo "# deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main-dbg" | sudo tee -a /etc/apt/sources.list.d/ros1.list
RUN apt-get -y install software-properties-common
#RUN add-apt-repository ppa:ros-for-jammy/noble
RUN apt-get -y update
RUN apt-get -y upgrade
#RUN apt-get -y install ros-noetic-desktop
RUN echo "yaml https://ros.packages.techfak.net/ros-one.yaml ubuntu" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-one.list
RUN rosdep update
RUN apt-get -y install ros-one-desktop

# fix ARM64 pkgconfig path issue -- Fix provided by ambrosekwok 
#RUN if [[ $(uname -m) = "arm64" || $(uname -m) = "aarch64" ]]; then                     \
#      cp /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/;   \
#    fi

RUN git clone https://github.com/LeoRover/leo_common;                 \
    git clone https://github.com/LeoRover/leo_common-ros2;             \
    # Compile ROS1:                                                   \
    cd /leo_common/leo_msgs; \
    unset ROS_DISTRO;   \
    source /opt/ros/one/setup.bash; \                                             
    time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release;        \
    # Compile ROS2:                                                   \
    unset ROS_DISTRO;   \
    cd /leo_common-ros2/leo_msgs;                                     \
    source /opt/ros/jazzy/setup.bash;                                 \
    time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release;        

###########################
# 7.) Compile ros1_bridge
###########################

# g++-11 and needed
RUN apt -y install g++-11 gcc-11
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 11
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 11

RUN                                                                                     \
     #-------------------------------------                                             \
     # Get the Bridge code                                                              \
     #-------------------------------------                                             \
     mkdir -p /ros-jazzy-ros1-bridge/src;                                               \
     cd /ros-jazzy-ros1-bridge/src;                                                     \
     git clone -b action_bridge_humble https://github.com/smith-doug/ros1_bridge.git;   \
     cd ros1_bridge/;                                                                   \
                                                                                        \
     #-------------------------------------                                             \
     # Apply the ROS1 and ROS2 underlays                                                \
     #-------------------------------------                                             \
     source /opt/ros/one/setup.bash;                                                    \
     source /opt/ros/jazzy/setup.bash;                                                  \
     source /leo_common/leo_msgs/install/setup.bash; \
     source /leo_common-ros2/leo_msgs/install/setup.bash; \
                                                                                        \
     #-------------------------------------                                             \
     # Finally, build the Bridge                                                        \
     #-------------------------------------                                             \
     MEMG=$(printf "%.0f" $(free -g | awk '/^Mem:/{print $2}'));                        \
     NPROC=$(nproc);  MIN=$((MEMG<NPROC ? MEMG : NPROC));                               \
     cd /ros-jazzy-ros1-bridge/;                                                        \
     echo "Please wait...  running $MIN concurrent jobs to build ros1_bridge";          \
     time ROS_DISTRO=humble MAKEFLAGS="-j $MIN" colcon build                            \
       --event-handlers console_direct+                                                 \
       --cmake-args -DCMAKE_BUILD_TYPE=Release

###########################
# 8.) Clean up
###########################
RUN apt-get -y clean all; apt-get -y update

###########################
# 9.) Pack all ROS1 dependent libraries
###########################
# fix ARM64 pkgconfig path issue -- Fix provided by ambrosekwok 
RUN if [[ $(uname -m) = "arm64" || $(uname -m) = "aarch64" ]]; then                    \
      cp /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/;  \
    fi


RUN ROS1_LIBS="libactionlib.so";                                                \
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

###########################
# 10.) Spit out ros1_bridge tarball by default when no command is given
###########################
RUN tar czf /ros-jazzy-ros1-bridge.tgz \
     --exclude '*/build/*' --exclude '*/src/*' /ros-jazzy-ros1-bridge 
ENTRYPOINT []
CMD cat /ros-jazzy-ros1-bridge.tgz; sync
