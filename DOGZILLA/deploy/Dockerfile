ARG YAHBOOM_BASE_IMAGE=yahboomtechnology/ros-humble:3.8
FROM ${YAHBOOM_BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

ARG ROS_APT_SOURCE_URL="http://packages.ros.org/ros2/ubuntu/pool/main/r/ros-apt-source/ros2-apt-source_1.2.0~jammy_all.deb"
ARG ROS_APT_SOURCE_SHA256="767884cf4ed03116b9d64438930a832ed854147ae435279a7924dfdf60f94433"

LABEL org.opencontainers.image.title="DOGZILLA S2 Pi-only mapping"
LABEL org.opencontainers.image.description="Reproducible ROS 2 Humble Cartographer runtime for Yahboom DOGZILLA S2"
LABEL org.opencontainers.image.source="https://github.com/kingnust/doggyzilla"

# Bootstrap the official ROS apt-source package. Yahboom's base image contains
# an expired legacy key, so the source package is downloaded by a pinned URL
# and verified before apt is used.
RUN set -eux; \
    curl -fsSL "${ROS_APT_SOURCE_URL}" -o /tmp/ros2-apt-source.deb; \
    echo "${ROS_APT_SOURCE_SHA256}  /tmp/ros2-apt-source.deb" | sha256sum -c -; \
    rm -f /etc/apt/sources.list.d/ros2.list; \
    dpkg -i /tmp/ros2-apt-source.deb; \
    rm -f /tmp/ros2-apt-source.deb /etc/apt/sources.list.d/ros2.list.disabled

COPY deploy/ros-packages.lock /tmp/ros-packages.lock

RUN set -eux; \
    apt-get update; \
    mapfile -t ros_packages < /tmp/ros-packages.lock; \
    apt-get install -y --no-install-recommends "${ros_packages[@]}"; \
    rm -rf /var/lib/apt/lists/* /tmp/ros-packages.lock

COPY deploy/patches/oradar-lidar-stop.patch /tmp/oradar-lidar-stop.patch

RUN set -eux; \
    cd /root/yahboomcar_ws/src/oradar_lidar; \
    patch --batch --forward --strip=1 < /tmp/oradar-lidar-stop.patch; \
    rm -f /tmp/oradar-lidar-stop.patch

COPY ros2/dogzilla_slam /root/yahboomcar_ws/src/dogzilla_slam

RUN set -ex; \
    source /opt/ros/humble/setup.bash; \
    cd /root/yahboomcar_ws; \
    colcon build \
        --packages-select oradar_lidar dogzilla_slam \
        --event-handlers console_direct+; \
    rm -rf \
        /root/yahboomcar_ws/build/oradar_lidar \
        /root/yahboomcar_ws/build/dogzilla_slam \
        /root/yahboomcar_ws/log

COPY deploy/ros-entrypoint.sh /usr/local/bin/dogzilla-ros-entrypoint
RUN chmod 0755 /usr/local/bin/dogzilla-ros-entrypoint

ENV ROS_DOMAIN_ID=12
ENV PYTHONUNBUFFERED=1
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1

WORKDIR /root/yahboomcar_ws
ENTRYPOINT ["/usr/local/bin/dogzilla-ros-entrypoint"]
CMD ["bash"]
