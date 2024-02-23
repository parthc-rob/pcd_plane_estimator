FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=US/NewYork
ENV USERNAME user 


#COPY ./settings/.vimrc /home/$USERNAME/
COPY ./settings /home/$USERNAME/
RUN ls -a /home/$USERNAME/
RUN bash /home/$USERNAME/install_user.sh
RUN apt-get update && apt-get install -y ca-certificates apt-utils build-essential make

# Install relevant dependencies
RUN apt-get install -y libpcl-dev pcl-tools 
RUN apt-get install -y tmux vim cmake

#<dependencies>

RUN export uid=1000 gid=1000 && \
    mkdir -p /home/user && \
    echo "user:x:${uid}:${gid}:Developer,,,:/home/user:/bin/bash" >> /etc/passwd && \
    echo "user:x:${uid}:" >> /etc/group 

WORKDIR /code-sample

RUN apt-get install -y libboost-all-dev \
    libxext-dev \
    libx11-dev \
    x11proto-gl-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libgtest-dev \
    xvfb \
    x11-utils
    #libnvidia-common-455

# Here you copy in your code
# COPY . .

# Here you build your code
# RUN rm -rf ./build/ && ./build_cpp.sh && ./calculate_plane.sh

#Entrypoint command
COPY ./system_entrypoint.sh /
ENTRYPOINT ["/system_entrypoint.sh"]
CMD ["/bin/bash"]

