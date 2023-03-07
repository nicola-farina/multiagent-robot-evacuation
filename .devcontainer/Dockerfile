FROM pla10/ros2_humble:amd64

# Install some packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install nano git \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists

# Clone repository
RUN cd /root/ros2_ws \
    && rm -r src \
    && git clone https://github.com/pla10/Shelfino_ROS2.git . \
    && git checkout devel

# Add startup to bashrc
RUN echo "source /root/startup.sh" > /root/.bashrc

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "bash" ]
