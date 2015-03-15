/*
 * SonarRange.h
 *
 *  Created on: Feb 12, 2014
 *      Author: kurt
 */

#ifndef RANGEMESSAGE_H_
#define RANGEMESSAGE_H_

#include <stdint.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Range.h>

namespace ur {

class RangeMessage {
private:
	int sonar_id;
    sensor_msgs::Range msg;

public:
	RangeMessage();
	virtual ~RangeMessage();

	int getSonarIdentity();
	void setSonarIdentity(int id);
	int getRadiationType();
	void setRadiationType(int i);
	float getFieldOfView();
	void setFieldOfView(float fov);
	float getMinimumRange();
	void setMinimumRange(float min);
	float getMaximumRange();
	void setMaximumRange(float max);
	float getRange();
	void setRange(float r);
	void publish(ros::Publisher topic, int counter);
};

} /* namespace ur */

#endif /* RANGEMESSAGE_H_ */
