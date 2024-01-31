FROM fishros2/ros:humble-desktop-full
LABEL authors="cmoon"

SHELL ["/bin/bash", "-c"]
WORKDIR /home/cmoon

RUN apt-get update && apt-get install -y python3-pip vim wget \
    && rm -rf /var/lib/apt/lists/*

RUN wget -O requirements.txt http://github.fishros.org/https://raw.githubusercontent.com/Cmoon-cyl/CmoonRos2/master/requirements.txt \
    && pip3 install --no-cache-dir -r requirements.txt \
    && rm requirements.txt

COPY . /home/cmoon

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/cmoon/cmoon_ws/install/setup.bash" >> /root/.bashrc


ENTRYPOINT ["/bin/bash"]