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

The node needs to access the pigpio daemon, hence the following steps to have it run at startup (if it does not do so already):

```
wget https://raw.githubusercontent.com/joan2937/pigpio/master/util/pigpiod.service
sudo cp pigpiod.service /etc/systemd/system
sudo systemctl enable pigpiod.service
sudo systemctl start pigpiod.service
```

To run:

```
roslaunch pi_sonar ubiquity_sonar.launch
```

## For developers only
Update pigpio:

git subtree pull --squash --prefix=pigpio https://github.com/joan2937/pigpio.git {ref to pull}
