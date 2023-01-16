FROM tiryoh/ros-desktop-vnc:noetic
MAINTAINER Jay Bhagiya jaybhagiya@iisc.ac.in

# set time/language
ENV TZ=Asia/Kolkata
ENV LANG C.UTF-8
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Update the image and install required packages
RUN apt-get update -y \
	&& apt-get install -y wget git unzip build-essential gcc g++ clang

# it continues to remain in bash in future images
SHELL ["/bin/bash", "-c"]

# install required ros packages for simulations
RUN apt-get install -y \
	ros-noetic-turtlebot3 \
	ros-noetic-turtlebot3-msgs \
	ros-noetic-turtlebot3-simulations \
	ros-noetic-slam-karto \
	&& echo "ubuntu\n" | sudo rosdep update

# creaking catkin workspace for ros
RUN mkdir -p ~/catkin_ws/src \
	&& cd ~/catkin_ws/src

# getting autonomous exploration packages.
# RUN git clone https://github.com/FocasLab/autonomous_exploration_pkgs.git
COPY ./ /home/ubuntu/catkin_ws/src/

# building the packages
RUN source /opt/ros/noetic/setup.bash \
	&& cd ~/catkin_ws \
	&& rosdep install --from-paths src --ignore-src --rosdistro=noetic -y \
	&& catkin build \
	&& echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc \
	&& echo	"source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc