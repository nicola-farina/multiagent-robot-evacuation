FROM pla10/ros2_humble:amd64

# Install some packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install nano git \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists

# Add a non-root user and setup a startup file in bashrc
RUN useradd -ms /bin/bash user && chmod -R a+rwx /root
COPY --chown=user startup.sh /home/user
USER user
RUN echo "source ${HOME}/startup.sh" >> ${HOME}/.bashrc

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "bash" ]
