/*
 * RangeMessage.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: kurt
 */

#include "RangeMessage.h"

namespace ur {

RangeMessage::RangeMessage() {
	sonar_id = 999;                   /* unset */
	msg.radiation_type = 0;			/* ULTRASOUND=0; INFRARED=1 */
	msg.field_of_view = 0.175;		/* 10 degrees * pi / 180 = ~.175 radians*/
	msg.min_range = 0.005;				/* in meters */
	msg.max_range = 5;				/* in meters */
	msg.range = 0;                	/* default */
}

RangeMessage::~RangeMessage() {
	// TODO Auto-generated destructor stub
}

int RangeMessage::getSonarIdentity()
{
	return (sonar_id);
}
void RangeMessage::setSonarIdentity(int id)
{
	sonar_id = id;
}

int RangeMessage::getRadiationType()
{
	return (msg.radiation_type);
}

void RangeMessage::setRadiationType(int i)
{
	msg.radiation_type = i;
}

float RangeMessage::getFieldOfView()
{
    return (msg.field_of_view);
}

void RangeMessage::setFieldOfView(float fov)
{
	msg.field_of_view = fov;
}

float RangeMessage::getMinimumRange()
{
	return (msg.min_range);
}

void RangeMessage::setMinimumRange(float min)
{
	msg.min_range = min;
}

float RangeMessage::getMaximumRange()
{
	return (msg.max_range);
}

void RangeMessage::setMaximumRange(float max)
{
	msg.max_range = max;
}

float RangeMessage::getRange()
{
	return msg.range;
}

void RangeMessage::setRange(float r)
{
	msg.range = r;
}

void RangeMessage::publish(ros::Publisher topic, int counter)
{

	  switch (sonar_id)
	  {
	     case 1 :
	    	 msg.header.frame_id = "hercules_skirt1";
	    	 break;
	     case 2 :
	    	 msg.header.frame_id = "hercules_skirt2";
	    	 break;
	     case 3 :
	    	 msg.header.frame_id = "hercules_skirt3";
	    	 break;
	     case 4 :
	    	 msg.header.frame_id = "hercules_skirt4";
	    	 break;
	     case 5 :
	    	 msg.header.frame_id = "hercules_skirt5";
	    	 break;
	     case 6 :
	    	 msg.header.frame_id = "hercules_skirt6";
	    	 break;
	     case 7 :
	    	 msg.header.frame_id = "hercules_skirt7";
	    	 break;
	     case 8 :
	    	 msg.header.frame_id = "hercules_skirt8";
	    	 break;
	     case 9 :
	    	 msg.header.frame_id = "hercules_skirt9";
	    	 break;
	     case 10 :
	    	 msg.header.frame_id = "hercules_skirt10";
	    	 break;
	     default :
	    	 break;
	  }
		 msg.header.stamp = ros::Time::now();
    	 msg.header.seq = counter;
    	 topic.publish(msg);

}

} /* namespace ur */
