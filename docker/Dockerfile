# ----------------------------------------------------------------------------
# Dockerfile für den STL27L LiDAR Treiber
# ----------------------------------------------------------------------------
# In diesem Container laufen der herstellerspezifische Treiber sowie eine ROS2-Bridge,
# die den LaserScan als Topic zur Verfügung stellt. Jede Zeile ist kommentiert, damit
# auch Einsteigerinnen und Einsteiger nachvollziehen können, was passiert.
# ----------------------------------------------------------------------------

FROM ros:humble

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        udev \
        wget \
        git && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/stl27l_driver

# Wir kopieren das ROS2-Paket (siehe stl27l_driver.py und setup.cfg) in den Container.
COPY stl27l_driver/ ./stl27l_driver/
COPY setup.cfg ./setup.cfg
COPY setup.py ./setup.py

RUN pip3 install --no-cache-dir .

# Beim Start laden wir die udev-Regeln neu und starten anschließend den Driver-Node.
CMD ["bash", "-c", "udevadm control --reload && udevadm trigger && ros2 run stl27l_driver driver_node"]
