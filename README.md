# Pi Sonar Node

Sonar Sensor Handling based on `pigpio` library.

To build:

```
cd ~/catkin_ws/src
git clone https://github.com/UbiquityRobotics/pi_sonar.git
cd ..
catkin_make
source devel/setup.bash
```

The node needs to run as the user root to access GPIO, hence the
following unconventional steps:

```
sudo chown root ~/catkin_ws/devel/lib/pi_sonar/pi_sonar
sudo chmod 4755 ~/catkin_ws/devel/lib/pi_sonar/pi_sonar
```

To run:

```
roslaunch pi_sonar ubiquity_sonar.launch
```

## For developers only
Update pigpio:

git subtree pull --squash --prefix=pigpio https://github.com/joan2937/pigpio.git {ref to pull}
