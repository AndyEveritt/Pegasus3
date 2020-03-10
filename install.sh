# install ROS kinetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# USB name links
sudo cp 99-usb-serial.rules /etc/udev/rules.d/

# install required ROS packages
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-hector-slam
sudo apt-get install ros-kinetic-gmapping
sudo apt-get install ros-kinetic-pointcloud-to-laserscan
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-robot-localization
sudo apt-get install ros-kinetic-usb-cam
sudo apt-get install ros-kinetic-gps-goal
sudo apt-get install ros-kinetic-swri-transform-util
sudo apt-get install ros-kinetic-mapviz ros-kinetic-mapviz-plugins ros-kinetic-tile-map ros-kinetic-multires-image
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-controller-manager
sudo apt-get install ros-kinetic-imu-tools
sudo apt-get install ros-kinetic-image-geometry
sudo apt-get install ros-kinetic-frontier-exploration
sudo apt-get install ros-kinetic-rosserial
sudo apt-get install ros-kinetic-rosserial-arduino

# setup Python 2 virtual environment
sudo apt install python-pip
sudo pip install virtualenv virtualenvwrapper
sudo rm -rf ~/get-pip.py ~/.cache/pip
echo -e "\n# virtualenv and virtualenvwrapper" >> ~/.bashrc
echo "export WORKON_HOME=$HOME/.virtualenvs" >> ~/.bashrc
echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
source ~/.bashrc

mkvirtualenv pegasus3 -p python2
workon pegasus3

# install Python 2 libraries in virtual environment
workon pegasus3
pip install catkin_pkg
pip install numpy
pip install rospkg
pip install empy
pip install imutils
pip install opencv-contrib-python
pip install pylint
pip uninstall serial
pip install pyserial
pip install defusedxml
pip install geographiclib
pip install utm
pip install matplotlib
pip install scipy

# image stuff
sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk-3-dev
sudo apt-get install python2.7-dev python3.5-dev

# install libfreenect2 (Kinect camera)
cd ~/
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libusb
sudo apt-get install libturbojpeg libjpeg-turbo8-dev
sudo apt-get install libglfw3-dev
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

# install xbox controller driver (also ps3)
sudo apt-get install xboxdrv
sudo xboxdrv --detach-kernel-driver --led 2
