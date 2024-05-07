FROM fishros2/ros:humble-desktop-full
LABEL authors="cmoon"

SHELL ["/bin/bash", "-c"]
WORKDIR /home/cmoon/cmoon_ws

RUN apt-get update && apt-get install -y python3-pip vim wget git && \
    rm -rf /var/lib/apt/lists/* && \
    update-alternatives --install /usr/bin/python python /usr/bin/python3 10

RUN git clone http://github.fishros.org/https://github.com/Cmoon-cyl/CmoonRos2.git && \
    mv CmoonRos2/cmoon_ws/src . && \
    mv CmoonRos2/requirements.txt . && \
    rm -rf CmoonRos2

RUN pip install --no-cache-dir -r requirements.txt && \
    rm requirements.txt

RUN mkdir -p /home/cmoon/cmoon_ws/src/controller/resource/weights && \
    wget -O yolov8n.pt https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt && \
    wget -O yolov8n-seg.pt https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n-seg.pt && \
    mv yolov8n.pt /home/cmoon/cmoon_ws/src/controller/resource/weights && \
    mv yolov8n-seg.pt /home/cmoon/cmoon_ws/src/controller/resource/weights

RUN colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/cmoon/cmoon_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /root/.bashrc && exec /bin/bash"]
