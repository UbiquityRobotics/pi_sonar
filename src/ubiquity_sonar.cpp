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

#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <boost/algorithm/string.hpp>

typedef struct _sonar {
  // BCM GPIO pins
  int trigger_pin;
  int echo_pin;
  uint32_t start_tick;
  uint32_t elapsed_ticks;
  } sonar_t;

sonar_t sonars[] = {
  { 20,	21, 0, 0 },
  { 12,	16, 0, 0 },
  { 23,	24, 0, 0 },
  { 27,	22, 0, 0 },
  { 19,	26, 0, 0 }
};

static int nsonars = sizeof(sonars) / sizeof(sonars[0]);

/* Trigger the next sonar */
void sonar_trigger()
{
   static int sonar = 0;

   int pin = sonars[sonar].trigger_pin;
   gpioWrite(pin, PI_ON);
   gpioDelay(10); /* 10us trigger pulse */
   gpioWrite(pin, PI_OFF);

   sonar++;
   if (sonar >= nsonars) {
     sonar = 0;
   }
}

/* Handle pin change */
void echo_callback(int pin, int level, uint32_t tick)
{
  int sonar = -1;
  sonar_t *p_sonar = NULL;

   for (int i=0; i<nsonars; i++) {
     if (pin == sonars[i].echo_pin) {
       sonar = i;
       p_sonar = &sonars[i];
     }
   }
   if (sonar == -1) {
     printf("Unexpected GPIO pin event, pin %d\n", pin);
     return;
   }

   if (level == PI_ON) {
     p_sonar->start_tick = tick;
   }
   else if (level == PI_OFF) {
     uint32_t elapsed = tick - p_sonar->start_tick; 
     p_sonar->elapsed_ticks = elapsed;
     //printf("sonar %d elapsed %d\n", sonar, elapsed);
   }
}

int setup_gpio()
{
  if (gpioInitialise()<0) {
    return false;
  }

  sonar_t *p_sonar = sonars;
  for (int i=0; i<nsonars; i++, p_sonar++) {
    gpioSetMode(p_sonar->trigger_pin, PI_OUTPUT);
    gpioWrite (p_sonar->trigger_pin, PI_OFF);

    gpioSetMode(p_sonar->echo_pin, PI_INPUT);

    /* monitor sonar echos */
    gpioSetAlertFunc(p_sonar->echo_pin, echo_callback);
  }

  /* update sonar 20 times a second, timer #0 */
  gpioSetTimerFunc(0, 50, sonar_trigger); /* every 50ms */

  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ubiquity_sonar");
  ros::NodeHandle nh("~");

  std::vector<ros::Publisher> pubs;
  std::vector<std::string> frames;

  double field_of_view;
  double min_range;
  double max_range;

  nh.param<double>("field_of_view", field_of_view, .43632347);
  nh.param<double>("min_range", min_range, .05);
  nh.param<double>("max_range", max_range, 10);

  for (int i=0; i<nsonars; i++) {
    std::string frame = str(boost::format{"sonar_%1%"} % i);
    frames.push_back(frame);
    pubs.push_back(nh.advertise<sensor_msgs::Range>(frame, 1));
  }

  ros::Publisher pub =
        nh.advertise<sensor_msgs::Range>("/sonars", 5);

  if (!setup_gpio()) {
    ROS_ERROR("Cannot initalize gpio");
    return 1;
  }

  ros::Rate rate(50);

  while (ros::ok()) {
    sonar_t *p_sonar = sonars;
    for (int i=0; i<nsonars; i++, p_sonar++) {
      uint32_t elapsed_ticks = p_sonar->elapsed_ticks;
      if (elapsed_ticks != 0) {
        p_sonar->elapsed_ticks = 0;

        sensor_msgs::Range msg;
        msg.field_of_view = field_of_view;
        msg.min_range = min_range;
        msg.max_range = max_range;
        msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        // seconds = ticks / 1000000 
        // speed of sound = 343 m/s 
        // round trip = double the distance
        msg.range = (float)elapsed_ticks * 0.0001715;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frames[i];
        pub.publish(msg);
        pubs[i].publish(msg);
      }
    } 
    rate.sleep();
  }
    
  gpioTerminate();

  return 0;
}

