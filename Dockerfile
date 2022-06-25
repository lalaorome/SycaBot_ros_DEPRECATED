FROM dustynv/jetbot_ros:foxy-r32.5.0

SHELL ["/bin/bash", "-c"] 
ENV SHELL /bin/bash

# install Fast-RTPS dependencies
RUN sudo apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev
# install Cyclone DDS dependencies
RUN sudo apt install --no-install-recommends -y \
    libcunit1-dev

# Create an overlay
RUN source ${ROS_ROOT}/install/setup.bash \
    && mkdir -p /syca_ws/src \
    && cd /syca_ws

COPY src/jetbot /syca_ws/src/jetbot
COPY src/interfaces /syca_ws/src/interfaces

RUN cd /syca_ws &&\
    source ${ROS_ROOT}/install/setup.bash && \
    colcon build
COPY entrypoint.sh /

RUN cat /entrypoint.sh
RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/install\/setup.bash"/g' \
    /entrypoint.sh && \
    cat /entrypoint.sh
RUN chmod -x /entrypoint.sh
WORKDIR /syca_ws

ENTRYPOINT ["/usr/bin/env"]
CMD ["bash", "/entrypoint.sh"]
