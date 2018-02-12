# Ubiquity Sonar Node

Sonar Sensor Handling based on `pigpio` library.

To build:

```
cd ~/catkin_ws/src
git clone https://github.com/UbiquityRobotics/ubiquity_sonar.git
cd ..
catkin_make
source devel/setup.bash
```

The node needs to run as the user root to access GPIO, hence the
following unconventional steps:

```
sudo chown root ~/catkin_ws/devel/lib/ubiquity_sonar/ubiquity_sonar
sudo chmod 4755 ~/catkin_ws/devel/lib/ubiquity_sonar/ubiquity_sonar
```

To run:

```
roslaunch ubiquity_sonar ubiquity_sonar.launch
```


