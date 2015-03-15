#Ubiquity Sonar Node

Node for publishing multiple sonars.  Written in C++, the node is dependent upon the ros-indigo-roscpp and ros-indigo-serial packages. 

The package uses ROS Parameters.  Every parameter has a default value.  If a valid parameter value is not found, the node will assume that default value.

Wayne's sonar array has ten sonars.  His firmware publishes over the serial port the ranges for each sonar in the format:

>[sonar_id]=[range in meters][crlf]

This package listens on a serial port and advertises/publishes an array of ten robot operating system (ROS) sensor_msgs/Range messages.

##Node Parameters

###Loop Rate
This node allows you to set the loop rate to check for and process a line of serial text.  

Parameter  | Default
---------- | --------
/ubiquity/sonar/loop_rate | 150

##Serial Parameters

###Port
The port parameter is a case sensitive string which points to the serial port of the host to which the arduino is connected.  

Parameter  | Default
---------- | --------
/ubiquity/sonar/serial_comm_port | /dev/ttyACM0

###Baud Rate
ROS serial supports the following baud rates:
110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 
38400, 56000, 57600, 115200, 128000, 153600, 230400, 256000, 
460800, 921600.

Parameter  | Default
---------- | --------
/ubiquity/sonar/serial_baud_rate | 115200

###Byte Size
ROS serial supports the following byte sizes:
5,6,7, and 8

Parameter  | Default
---------- | --------
/ubiquity/sonar/serial_byte_size | 8

###Parity
This parameter is a case insensitive string.  ROS serial supports the following parity settings:
"none", "odd", and "even"

Parameter  | Default
---------- | --------
/ubiquity/sonar/serial_parity | none

###Stop Bits
This node currently supports either 1 or 2 stop bits (1.5 is currently not supported).

Parameter  | Default
---------- | --------
/ubiquity/sonar/serial_stop_bits | 1

###Flow Control
This is a case insensitive string defining serial flow control.  Valid values are:
"none", "hardware", and "software".

Parameter  | Default
---------- | --------
/ubiquity/sonar/serial_flow_control | none

##Node Message
For the published messages [sensor_msgs/Range] [range_doc], the node forces the radiation type to be for sonar (ULTRASOUND).  It allows you to play with the field of view as well as the minimum and maximum ranges until we comfortable with the settings.

###Field of View
This parameter MUST be set **in radians**.  The node does not check validity.  

Parameter  | Default
---------- | --------
/ubiquity/sonar/sonar_field_of_view | 0.175

###Minimum Range
This parameter MUST be set in meters.  The node does not check validity. Default is one centimeter (0.01 meters).

Parameter  | Default
---------- | --------
/ubiquity/sonar/sonar_minimum_range | 0.01

###Maximum Range
This parameter MUST be set in meters.  The node does not check validity. Default is four meters.

Parameter  | Default
---------- | --------
/ubiquity/sonar/sonar_maximum_range | 4

[range_doc]: http://docs.ros.org/api/sensor_msgs/html/msg/Range.html "ROS Documentation of sensor_msgs/Range"



