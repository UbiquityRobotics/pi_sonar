#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <sstream>
#include <string>
#include <stdlib.h>
#include <exception>
#include <serial/serial.h>

#include "RangeMessage.h"

/**
 * This code is based on the WritngPublisherSubscriber(c++) tutorial for Hydro.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ubiquity_sonar");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  serial::Serial sonar_addr;


  /*
   * Serial port parameters
   */
  std::string sPort;
  int iBaud = 0;
  serial::Timeout to;
  int iBytesize = 0;
  std::string sParity;
  int iStopbits = 0;
  std::string sFlowControl;

  /*
   * Sonar Message Parameters
   *
   * Radiation type is hardcoded to ULTRASOUND.
   */
	double dFieldOfView;		//set in radians
	double dMinimumRange;	//set in meters
	double dMaximumRange;	//set in meters

  /*
   * Node Parameters
   */
  int iLoopRate;  // the number of times per second I want to loop



  if (!n.getParam("ubiquity/sonar/serial_comm_port", sPort))
  {
	  sPort.assign("/dev/ttyS0");
	  sonar_addr.setPort(sPort);
	  n.setParam("ubiquity/sonar/serial_comm_port", sPort);
  } else {
	  sonar_addr.setPort(sPort);
  }

  /* Obtain the baud rate.
   *
   * Invalid parameters are ignored and reset to the default of 115200
   */
  if (!n.getParam("/ubiquity/sonar/serial_baud_rate", iBaud))
  {
	  sonar_addr.setBaudrate(115200);
	  n.setParam("/ubiquity/sonar/serial_baud_rate", 115200);
  } else {
	  switch (iBaud)
	  {
	  case 110 :
	  case 300 :
	  case 600 :
	  case 1200 :
	  case 2400 :
	  case 4800 :
	  case 9600 :
	  case 14400 :
	  case 19200 :
	  case 28800 :
	  case 38400 :
	  case 56000 :
	  case 57600 :
	  case 115200 :
	  case 128000 :
	  case 153600 :
	  case 230400 :
	  case 256000 :
	  case 460800 :
	  case 921600 :
		  sonar_addr.setBaudrate(iBaud);
		  break;
	  default :
		  ROS_ERROR("Unsupported \"/ubiquity/sonar/serial_baud_rate\" parameter value \"%d\"", iBaud);
		  ROS_INFO("Setting \"/ubiquity/sonar/serial_baud_rate\" parameter to \"115200\"");
		  n.setParam("/ubiquity/sonar/baud_rate", 115200);
		  sonar_addr.setBaudrate(115200);
		  break;
	  }
  }

  //set the serial port timeout
  to.simpleTimeout(1000);
  sonar_addr.setTimeout(to);

  /*
   * Obtain the serial byte size (5,6,7, or 8 bits).
   * Invalid settings are ignored and the default 8 bits is used.
   */
  if (!n.getParam("/ubiquity/sonar/serial_byte_size", iBytesize))
  {
	  sonar_addr.setBytesize(serial::eightbits);
	  n.setParam("/ubiquity/sonar/serial_byte_size", serial::eightbits);
  } else {
	  switch (iBytesize)
	  {
	  	  case serial::fivebits :
	  		sonar_addr.setBytesize(serial::fivebits);
	  		break;
	  	  case serial::sixbits :
	  		sonar_addr.setBytesize(serial::sixbits);
	  		break;
	  	  case serial::sevenbits :
	  		sonar_addr.setBytesize(serial::sevenbits);
	  		break;
	  	  case serial::eightbits :
	  		sonar_addr.setBytesize(serial::eightbits);
	  		break;
	  	  default :
			ROS_ERROR("Unsupported \"/ubiquity/sonar/serial_byte_size\" parameter value \"%d\"", iBytesize);
			ROS_INFO("Setting \"/ubiquity/sonar/serial_byte_size\" parameter to \"8\"");
			n.setParam("/ubiquity/sonar/serial_byte_size", serial::eightbits);
	  		sonar_addr.setBytesize(serial::eightbits);
	  		break;
	  }
  }

  /*!
   * Obtain the parity for the serial port.
   * Invalid values are ignored and the default ("none") is used.
   */
  if (!n.getParam("/ubiquity/sonar/serial_parity", sParity))
  {
	  sonar_addr.setParity(serial::parity_none);
	  n.setParam("/ubiquity/sonar/serial_parity", "none");
	  sParity.assign("none");
  } else {  //case insensitive in the parameter file
	  std::transform(sParity.begin(), sParity.end(), 
                         sParity.begin(), ::tolower);
  }

  if (sParity.compare("none")) {
  	  sonar_addr.setParity(serial::parity_none);
  } else if (sParity.compare("odd")) {
  	  sonar_addr.setParity(serial::parity_odd);
  } else if (sParity.compare("even")) {
	  sonar_addr.setParity(serial::parity_even);
  } else { // default
	  ROS_ERROR("Unsupported \"/ubiquity/sonar/serial_parity\" parameter value \"%s\"", sParity.c_str());
	  ROS_INFO("Setting \"/ubiquity/sonar/serial_parity\" parameter to \"none\"");
	  n.setParam("parity", "none");
	  sonar_addr.setParity(serial::parity_none);
  }

  /*!
   * Obtain the number of stopbits for the serial port (1, 1.5, 2).
   *
   * Invalid parameters and the default value of 1 is used.
   * NOTE:  The current version of the serial package does not handle 1.5
   * TODO:  Add support when I see how they modified the next release of serial
   */
  if (!n.getParam("/ubiquity/sonar/serial_stop_bits", iStopbits))
  {  // make the default as one stop bit
	  sonar_addr.setStopbits(serial::stopbits_one);
	  n.setParam("/ubiquity/sonar/serial_stop_bits", 1);
  } else {
	  switch (iStopbits) {
	  case serial::stopbits_one :
		  sonar_addr.setStopbits(serial::stopbits_one);
          break;
	  case serial::stopbits_two :
		  sonar_addr.setStopbits(serial::stopbits_two);
          break;
	  default:
		  ROS_ERROR("Unsupported \"/ubiquity/sonar/serial_stop_bits\" parameter value \"%d\"", iStopbits);
		  ROS_INFO("Setting \"/ubiquity/sonar/serial_stop_bits\" parameter to \"1\"");
		  n.setParam("/ubiquity/sonar/serial_stop_bits", 1);
          sonar_addr.setStopbits(serial::stopbits_one);
		  break;
	  }
  }

  /*!
   * Obtain the flow control for the serial port.
   * Allowed values are:
   *    "none" (default),
   *    "software",
   *    "hardware"
   * Invalid values are ignored and the default is used
   */
  if (!n.getParam("/ubiquity/sonar/serial_flow_control", sFlowControl))
  {  // did not find key/value pair - setting yaml parameter to "none"
	  n.setParam("/ubiquity/sonar/serial_flow_control", "none");
	  sFlowControl.assign("none");
  } else { // case insensitive string in the parameter file
	  std::transform(sFlowControl.begin(), sFlowControl.end(), 
                         sFlowControl.begin(), ::tolower);
  }

  if (sFlowControl.compare("none")) {
	  sonar_addr.setFlowcontrol(serial::flowcontrol_none);
  } else if (sFlowControl.compare("hardware")) {
	  sonar_addr.setFlowcontrol(serial::flowcontrol_hardware);
  } else if (sFlowControl.compare("software")) {
	  sonar_addr.setFlowcontrol(serial::flowcontrol_software);
  } else { // default
	  ROS_ERROR("Unsupported \"/ubiquity/sonar/serial_flow_control\" parameter value \"%s\"", sFlowControl.c_str());
	  ROS_INFO("Setting \"/ubiquity/sonar/serial_flow_control\" parameter to \"1\"");
	  sonar_addr.setFlowcontrol(serial::flowcontrol_none);
  }

  //allocate the range message
  ur::RangeMessage msg;
  /*
   * Set the field of view, minimum range, and maximum range.
   */


  if (!n.getParam("/ubiquity/sonar/sonar_field_of_view", dFieldOfView))
  {  // set default to .175 radians (10 degrees)
	  n.setParam("/ubiquity/sonar/sonar_field_of_view", 0.175);
	  msg.setFieldOfView(0.175);
  } else {
	  msg.setFieldOfView((float) dFieldOfView);
  }
  if (!n.getParam("/ubiquity/sonar/sonar_minimum_range", dMinimumRange))
  {  // Set default to 1 cm (.01 meters)
	  n.setParam("/ubiquity/sonar/sonar_minimum_range", 0.01);
	  msg.setMinimumRange(0.01);
  } else {
	  msg.setMinimumRange((float) dMinimumRange);
  }
  if (!n.getParam("/ubiquity/sonar/sonar_maximum_range", dMaximumRange))
  {  // Set default to 4 meters
	  n.setParam("/ubiquity/sonar/sonar_maximum_range", 4);
	  msg.setMaximumRange(4);
  } else {
	  msg.setMaximumRange((float) dMaximumRange);
  }

  /*
   * Pick up the sleep value for the message loop.  Any integer is valid
   */
  if (!n.getParam("/ubiquity/sonar/loop_rate", iLoopRate))
  {
	  iLoopRate = 150;
	  n.setParam("/ubiquity/sonar/loop_rate", 150);
  }
  ros::Rate loop_rate(iLoopRate);

  /**
  * The advertise() function is how you tell ROS that you want to
  * publish on a given topic name. This invokes a call to the ROS
  * master node, which keeps a registry of who is publishing and who
  * is subscribing. After this advertise() call is made, the master
  * node will notify anyone who is trying to subscribe to this topic name,
  * and they will in turn negotiate a peer-to-peer connection with this
  * node.  advertise() returns a Publisher object which allows you to
  * publish messages on that topic through a call to publish().  Once
  * all copies of the returned Publisher object are destroyed, the topic
  * will be automatically unadvertised.
  *
  * The second parameter to advertise() is the size of the message queue
  * used for publishing messages.  If messages are published more quickly
  * than we can send them, the number here specifies how many messages to
  * buffer up before throwing some away.
  */

  ros::Publisher ubiquity_sonar = n.advertise<sensor_msgs::Range>("ubiquity_sonar", 100);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   * We need to maintain a count for each sonar on the sonar_addr.
   */
  int count[10] = {0,0,0,0,0,0,0,0,0,0};
  double reading;	//the LHS value (range reading from sonar[])

  /*
   * Allocate the buffer to capture a line of text from the arduino
   * Each line is in the format [sonar]=[distance reading in centimeters]CRLF
   *
   *    '1=324'
   */
  std::string sBuffer((size_t) 10, '\0');

  /*
   * Open the serial port
   */
  try
  {
	  sonar_addr.open();
  } catch (serial::PortNotOpenedException &e) {
	  ROS_ERROR("Serial port not opened: \"%s\"", e.what());
  } catch (serial::IOException &e) {
	  ROS_ERROR("IOException: \"%s\"", e.what());
  } catch (std::invalid_argument) {
	  ROS_ERROR("Attempted to open the serial port with an invalid argument");
  }

  if(!sonar_addr.isOpen())
  {
	  ROS_ERROR("unable to open serial port: \"%s\"", sPort.c_str());
	  return (-1);
  }


  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */


	sBuffer.assign(sonar_addr.readline(10, "\n"));
	unsigned pos = sBuffer.find("=");
	char *ptr;

	int ident;
	float r;

	ROS_INFO("Raw data: %s", sBuffer.c_str());
	//parse out the sensor identifier
	ident = (int) std::strtol(sBuffer.substr(0,(pos)).c_str(), &ptr, 10);

	/**
	 * Chris feeds me range values in centimeters.
	 * Need to convert to meters.
	 */
	r = (float) std::strtod(sBuffer.substr(pos+1).c_str(), &ptr)/100;

	msg.setSonarIdentity(ident);
	msg.setRange(r);

	if((ident >= 1) && (ident <= 10))
	  msg.publish( ubiquity_sonar, count[ ident - 1 ]++ );

    ros::spinOnce();

    loop_rate.sleep();
  }

  return (0);
}
