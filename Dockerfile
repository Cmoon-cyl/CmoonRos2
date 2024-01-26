FROM fishros2/ros:humble-desktop-full
LABEL authors="cmoon"

SHELL ["/bin/bash", "-c"]

WORKDIR /home/cmoon
COPY . /home/cmoon

RUN apt-get update && apt-get install -y python3-pip vim
RUN pip3 install -r requirements.txt

RUN echo "source /opt/ros/humble/setup.bash" >> /home/cmoon/.bashrc && \
    echo "source /home/cmoon/cmoon_ws/install/setup.bash" >> /home/cmoon/.bashrc


ENTRYPOINT ["/bin/bash"]