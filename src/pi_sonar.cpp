/*
 * Copyright (c) 2018, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#ifdef __arm__

#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <boost/algorithm/string.hpp>


class Sonar {
public:
  // BCM GPIO pins
  int trigger_pin;
  int echo_pin;

  uint32_t start_tick;
  uint32_t elapsed_ticks;

  std::string frame;
  ros::Publisher pub;

  Sonar(int trigger_pin, int echo_pin, int id, ros::NodeHandle& nh)
  {
    this->trigger_pin = trigger_pin;
    this->echo_pin = echo_pin;

    start_tick = 0;
    elapsed_ticks = 0;

    frame = str(boost::format{"sonar_%1%"} % id);
    pub = nh.advertise<sensor_msgs::Range>(frame, 1);
  }
};


static std::vector<Sonar> sonars;


/* Trigger the next sonar */
void sonar_trigger()
{
   static int sonar = 0;

   int pin = sonars[sonar].trigger_pin;
   gpioWrite(pin, PI_ON);
   gpioDelay(10); /* 10us trigger pulse */
   gpioWrite(pin, PI_OFF);

   sonar++;
   if (sonar >= sonars.size()) {
     sonar = 0;
   }
}

/* Handle pin change */
void echo_callback(int pin, int level, uint32_t tick)
{
  for (auto& sonar : sonars) {
    if (pin == sonar.echo_pin) {
      if (level == PI_ON) {
        sonar.start_tick = tick;
      }
      else if (level == PI_OFF) {
        uint32_t elapsed = tick - sonar.start_tick;
        sonar.elapsed_ticks = elapsed;
        //printf("sonar %d elapsed %d\n", sonar, elapsed);
      }
      return;
     }
   }
   printf("Unexpected GPIO pin event, pin %d\n", pin);
}

int setup_gpio()
{
  if (gpioInitialise() < 0) {
    return false;
  }

  for (const auto& sonar : sonars) {
    gpioSetMode(sonar.trigger_pin, PI_OUTPUT);
    gpioWrite (sonar.trigger_pin, PI_OFF);

    gpioSetMode(sonar.echo_pin, PI_INPUT);

    /* monitor sonar echos */
    gpioSetAlertFunc(sonar.echo_pin, echo_callback);
  }

  /* update sonar 20 times a second, timer #0 */
  gpioSetTimerFunc(0, 50, sonar_trigger); /* every 50ms */

  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pi_sonar");
  ros::NodeHandle nh("~");

  double field_of_view;
  double min_range;
  double max_range;

  nh.param<double>("field_of_view", field_of_view, .43632347);
  nh.param<double>("min_range", min_range, .05);
  nh.param<double>("max_range", max_range, 10);

  ros::Publisher pub = nh.advertise<sensor_msgs::Range>("/sonars", 5);

  // pin numbers are specific to the hardware
  sonars.push_back(Sonar(20, 21, 0, nh));
  sonars.push_back(Sonar(12, 16, 1, nh));
  sonars.push_back(Sonar(23, 24, 2, nh));
  sonars.push_back(Sonar(27, 22, 3, nh));
  sonars.push_back(Sonar(19, 26, 4, nh));

  if (!setup_gpio()) {
    ROS_ERROR("Cannot initalize gpio");
    return 1;
  }

  ros::Rate rate(50);

  sensor_msgs::Range msg;
  msg.field_of_view = field_of_view;
  msg.min_range = min_range;
  msg.max_range = max_range;
  msg.radiation_type = sensor_msgs::Range::ULTRASOUND;

  ROS_INFO("Pi Sonar node ready");

  while (ros::ok()) {
    for (auto& sonar : sonars) {
      uint32_t elapsed_ticks = sonar.elapsed_ticks;
      if (elapsed_ticks != 0) {
        // clear so we don't publish again
        sonar.elapsed_ticks = 0;
        // seconds = ticks / 1000000
        // speed of sound = 343 m/s
        // for round trip, halve the distance
        msg.range = (float)elapsed_ticks * 0.0001715;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = sonar.frame;
        pub.publish(msg);
        sonar.pub.publish(msg);
      }
    }
    rate.sleep();
  }

  gpioTerminate();

  return 0;
}

#else

#include <stdio.h>

int main(int argc, char **argv) {
  fprintf(stderr, "pi_sonar only works on the Raspberry Pi\n");
  return 1;
}

#endif // __arm__
