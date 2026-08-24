# syntax=docker/dockerfile:1.7
FROM ubuntu:22.04

ARG ROS_DISTRO=humble
ARG ROS_APT_SOURCE_VERSION=1.2.0
ARG GITHUB_MIRROR_PREFIX=""

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    RUNS_IN_DOCKER=true \
    PROJECT_ROOT=/opt/project19 \
    XDG_CACHE_HOME=/opt/project19/.cache \
    PIP_CACHE_DIR=/opt/project19/.cache/pip \
    COLCON_HOME=/opt/project19/.cache/colcon \
    TMPDIR=/opt/project19/.cache/tmp \
    ROS_LOG_DIR=/opt/project19/log/ros \
    PYTHONUSERBASE=/opt/project19/.local/python \
    MICRO_XRCE_DDS_AGENT_PREFIX=/opt/project19/.local/micro-xrce-dds-agent

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN mkdir -p "${TMPDIR}"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      git \
      locales \
      software-properties-common \
    && locale-gen en_US.UTF-8 \
    && add-apt-repository -y universe \
    && rm -rf /var/lib/apt/lists/*

RUN ros_apt_source_url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb" \
    && if [[ -n "${GITHUB_MIRROR_PREFIX}" ]]; then \
      ros_apt_source_url="${GITHUB_MIRROR_PREFIX}${ros_apt_source_url}"; \
    fi \
    && curl -fsSL --retry 5 --retry-all-errors --connect-timeout 15 \
      "${ros_apt_source_url}" \
      -o /tmp/ros2-apt-source.deb \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb \
    && curl -fsSL --retry 5 --retry-all-errors --connect-timeout 15 \
      https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      cmake \
      ffmpeg \
      gosu \
      ninja-build \
      openbox \
      python3-colcon-common-extensions \
      python3-pil \
      python3-rosdep \
      python3-vcstool \
      "ros-${ROS_DISTRO}-ament-lint-common" \
      "ros-${ROS_DISTRO}-desktop" \
      "ros-${ROS_DISTRO}-ros-gzharmonic" \
      wmctrl \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${PROJECT_ROOT}

COPY dependencies.repos ./dependencies.repos
RUN mkdir -p external src \
    && git config --global http.version HTTP/1.1 \
    && if [[ -n "${GITHUB_MIRROR_PREFIX}" ]]; then \
      git config --global \
        url."${GITHUB_MIRROR_PREFIX}https://github.com/".insteadOf \
        "https://github.com/"; \
    fi \
    && vcs import --shallow --retry 5 . < dependencies.repos

RUN bash external/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx \
    && python3 -m pip install --user "numpy<2" \
    && rm -rf /var/lib/apt/lists/*

RUN cmake \
      -S external/Micro-XRCE-DDS-Agent \
      -B .cache/micro-xrce-dds-agent-build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=${MICRO_XRCE_DDS_AGENT_PREFIX} \
    && cmake --build .cache/micro-xrce-dds-agent-build \
      -j "$(nproc)" \
    && cmake --build .cache/micro-xrce-dds-agent-build \
      --target install -j "$(nproc)"

RUN make -C external/PX4-Autopilot px4_sitl

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && rosdep init \
    && if [[ -n "${GITHUB_MIRROR_PREFIX}" ]]; then \
      sed -i \
        "s|https://raw.githubusercontent.com/|${GITHUB_MIRROR_PREFIX}https://raw.githubusercontent.com/|g" \
        /etc/ros/rosdep/sources.list.d/20-default.list; \
    fi

COPY . .

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && rosdistro_index_url="https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml" \
    && if [[ -n "${GITHUB_MIRROR_PREFIX}" ]]; then \
      rosdistro_index_url="${GITHUB_MIRROR_PREFIX}${rosdistro_index_url}"; \
    fi \
    && export ROSDISTRO_INDEX_URL="${rosdistro_index_url}" \
    && for attempt in {1..5}; do \
      if rosdep update; then break; fi; \
      if [[ "${attempt}" -eq 5 ]]; then exit 1; fi; \
      sleep "$((attempt * 5))"; \
    done \
    && rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -r -y \
    && colcon build --symlink-install --event-handlers console_direct+

RUN useradd --create-home --uid 1000 simulator \
    && chown -R simulator:simulator ${PROJECT_ROOT} \
    && chmod +x docker/entrypoint.sh

ENTRYPOINT ["/opt/project19/docker/entrypoint.sh"]
CMD ["ros2", "launch", "drone_bringup", "m4_inspection.launch.py", "use_rviz:=false", "use_gazebo_gui:=false"]
