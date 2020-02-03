// This program publishes imu data from usb port
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>        // sensor_msgs::Imu
#include <string>                   // std::string std::stod
#include <iostream>                 // std::stringstream std::cout
#include <serial/serial.h>          // serial::Serial serial::Timeout::simpleTimeout
#include <vector>                   // std::vector
#include <sstream>                  // std::stringstream
#include <exception>                // std::exception
#include <sstream>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/math/constants/constants.hpp>
//#include <chiki-briki_i_v_damki.h>
// constants
std::string TOPIC = "imu";

std::string PORT;// = "/dev/ttyUSB200";  // port name
const int BAUD = 2000000;            // for incoming data
int LINE_LENGHT = 156;               // lengh of string line coming from imu
int RATE = 10000;                   // frequency to publish at
int TIMOUT = 10;                    // delay in ms
int VALUES_NUMBER = 20;
uint16_t FRACT_NUMBER = 32767; 
double g = 9.81;
const double pi = boost::math::constants::pi<double>();
int TEMP_BUF_SIZE = 800;
int N_OF_IMUS = 4;
int N_OF_IMU_FIELDS = 7;
int POS_OF_FIRST_ACC_FIELD = 4;
int POS_OF_FIRST_GYRO_FIELD = 8;

/* Received line coFIELDS the following data separted by spaces:
- m s FIELDS								RTC time
- cnt 												count
- acc1x acc1y acc1z temp1 gyro1x gyro1y gyro1z 		IMU0 data
- acc2x acc2y acc2z temp2 gyro2x gyro2y gyro2z 		IMU1 data
- ...												IMUx data
- \n 												end of line
*/

std::vector<int16_t> string_to_ints(std::string str) { 
	std::stringstream ss;	 
	/* Storing the whole string into string stream */
	ss << str; 
	/* Running loop till the end of the stream */
	std::string temp; 
	int found; 
	
	std::vector<int16_t> imu_ints;        

	while (!ss.eof()) { 
		/* extracting word by word from stream */
		ss >> temp; 
		/* Checking the given word is integer or not */
		if (std::stringstream(temp) >> std::hex >> found) {
			//std::cout << static_cast<int16_t>(found) << " "; 
			imu_ints.push_back(static_cast<int16_t>(found));
		}
		/* To save from space at the end of string */
		temp = ""; 
	} 
	//std::cout << std::endl;
return imu_ints;
} 

ros::Time imu_ints_to_board_ts(std::vector<int16_t> imu_ints) {
	ros::Time board_ts;
	double secs;

	secs = imu_ints[0] * 60.0 + imu_ints[1] * 1.0 + (FRACT_NUMBER - imu_ints[2])/(FRACT_NUMBER + 1.0);
	board_ts = ros::Time(secs);

	return board_ts;
}

boost::numeric::ublas::vector<double> imu_ints_to_acc(std::vector<int16_t> imu_ints, uint8_t imu_n) {
	boost::numeric::ublas::vector<double> acc(3);
	boost::numeric::ublas::vector<int16_t> acc_ints(3);

	acc_ints[0] = imu_ints[N_OF_IMU_FIELDS * imu_n + POS_OF_FIRST_ACC_FIELD];
	acc_ints[1] = imu_ints[N_OF_IMU_FIELDS * imu_n + POS_OF_FIRST_ACC_FIELD+1];
	acc_ints[2] = imu_ints[N_OF_IMU_FIELDS * imu_n + POS_OF_FIRST_ACC_FIELD+2];

	acc = acc_ints/16384.0*g;
	return acc;
}

boost::numeric::ublas::vector<double> imu_ints_to_gyro(std::vector<int16_t> imu_ints, uint8_t imu_n) {
	boost::numeric::ublas::vector<double> gyro(3);
	boost::numeric::ublas::vector<int16_t> gyro_ints(3);
	gyro_ints[0] = imu_ints[N_OF_IMU_FIELDS * imu_n + POS_OF_FIRST_GYRO_FIELD];
	gyro_ints[1] = imu_ints[N_OF_IMU_FIELDS * imu_n + POS_OF_FIRST_GYRO_FIELD+1];
	gyro_ints[2] = imu_ints[N_OF_IMU_FIELDS * imu_n + POS_OF_FIRST_GYRO_FIELD+2];

	gyro = gyro_ints / 131.0 / 180.0 * pi;
	return gyro;
}

std::vector<std::string> split(std::string string_line, char delimeter) {
    // splits line into a vector of str values
    std::stringstream ss(string_line);
    std::string item;
    std::vector<std::string> splitted;
    while (std::getline(ss, item, delimeter))
        splitted.push_back(item);
    return splitted;
}

std::vector<double> strings_to_doubles_vector(std::vector<std::string> strings_vector) {
    // converts  vector of strings to vector of doubles
    std::vector<double> doubles_vector;
    double imu_val;
    std::string::size_type sz; // size_type

    for (int i = 0; i < strings_vector.size(); i++) {
        try {
            imu_val = std::stod(strings_vector[i], &sz); }
        catch(std::exception& ia) {
            imu_val = 0.0;} // todo what value should be used if "-" or "@" or " " is recieved from imu ?
        doubles_vector.push_back(imu_val);
    }
    return doubles_vector;
}

void publish_imu(ros::Publisher imu_pub, uint8_t imu_n, ros::Time ts, boost::numeric::ublas::vector<double> acc, boost::numeric::ublas::vector<double> gyro) {
    // publish_imu data [a in m/s^2] and [w in rad/s]
	std::string topic = TOPIC + std::to_string(imu_n);
    sensor_msgs::Imu imu_msg;

    imu_msg.header.frame_id = topic;
    imu_msg.header.stamp = ts;
    // linear_acceleration
    imu_msg.linear_acceleration.x = acc[0];
    imu_msg.linear_acceleration.y = acc[1];
    imu_msg.linear_acceleration.z = acc[2];
    // angular_velocity
    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];
    // Publish the message.
    imu_pub.publish(imu_msg);
}

int main(int argc, char **argv) {
	if (argc < 2) {
	    std::cout << "Please, specify serial device.For example, \"/dev/ttyUSB0\"" << std::endl;
	    return 0;
	}
	PORT = argv[1];
    bool board_starting_ts_is_read = false;
    std::string str;
    std::string num;
    std::stringstream ss;

    std::vector<int16_t> imu_ints;
    ros::Time sys_starting_ts, board_starting_ts, ts, ts_old;
    ros::Duration delta_ts;
    boost::numeric::ublas::vector<double> acc;
    boost::numeric::ublas::vector<double> gyro;
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    // Create a publisher object.
    //ros::Publisher imu_pub[N_OF_IMUS];
    //for (int i=0; i<N_OF_IMUS; i++) {
	//    imu_pub[i] = nh.advertise<sensor_msgs::Imu>(TOPIC + std::to_string(i), 10000);
    //}
    ros::Publisher imu_pub;
    imu_pub = nh.advertise<sensor_msgs::Imu>(TOPIC, 10000);

    // open port, baudrate, timeout in milliseconds
    serial::Serial my_serial(PORT, BAUD, serial::Timeout::simpleTimeout(TIMOUT));

    // check if serial port open
    std::cout << "Is the serial port open?";
    if(my_serial.isOpen())
        std::cout << " Yes." << "\n";
    else
        std::cout << " No." << "\n";

    sys_starting_ts = ros::Time::now();
    // Searching for start of the first complete line
    while(!board_starting_ts_is_read) {
        str = my_serial.readline(); 
        //std::cout << str.size() << std::endl;
		if (str.size() == LINE_LENGHT){
            imu_ints = string_to_ints(str);
            delta_ts = sys_starting_ts - imu_ints_to_board_ts(imu_ints);
            board_starting_ts_is_read = true;
        }
	}
    while(ros::ok()) {
        ss << my_serial.readline(); 
    	//std::cout << st.size() << std::endl;
		while(ss.tellp() - ss.tellg() > TEMP_BUF_SIZE) {
			std::getline(ss, str);
			//std::cout << str.size() << " " << LINE_LENGHT << std::endl;
			if (str.size() + 1 == LINE_LENGHT) {
	            imu_ints = string_to_ints(str);
                ts = imu_ints_to_board_ts(imu_ints) + delta_ts;
                //std::cout << 1.0/((ts-ts_old).toSec()) << std::endl;
				//ts_old = ts;			    
                for (int i=0; i<N_OF_IMUS; i++) {
		            acc = imu_ints_to_acc(imu_ints, i);
		            gyro = imu_ints_to_gyro(imu_ints, i);
					//publish_imu(imu_pub[i], i, ts, acc, gyro);
					publish_imu(imu_pub, i, ts, acc, gyro);
                }
	        }
	    }
    }
}
