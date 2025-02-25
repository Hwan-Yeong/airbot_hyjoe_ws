# AIRBOT META PACKAGE

## dependency

    $ sudo apt-get update
    $ sudo apt-get install python3-rosdep2
    $ sudo rosdep init
    $ rosdep update
    $ rosdep install --from-paths src --ignore-src -r -y
    $ sudo apt-get install ros-humble-vision-msgs
    $ sudo apt-get install ros-humble-pcl-ros 