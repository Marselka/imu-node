#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>
#include <string>                   
#include <iostream>                 
#include <serial/serial.h>          
#include <vector>                   
#include <sstream>                  
#include <exception>                
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/math/constants/constants.hpp>
//#include <chiki-briki_i_v_damki.h>

std::string IMU_TOPIC_PREFIX = "imu";
std::string IMU_TEMP_TOPIC_POSTFIX = "_temp";
std::string CAMERAS_TS_TOPIC = "cameras_ts";
std::string LIDAR_TS_TOPIC = "lidar_ts";

std::string IMU_TEMP_TOPIC = IMU_TOPIC_PREFIX + IMU_TEMP_TOPIC_POSTFIX;
std::string PORT;// = "/dev/ttyUSB200";  // port name
const int BAUD = 2000000;            
int RATE = 10000;                   
int TIMOUT = 10;                    
uint16_t FRACT_NUMBER = 10000-1; 
double G = 9.81;
const double PI = boost::math::constants::pi<double>();
int TEMP_BUF_SIZE = 200;
int NUM_OF_IMUS = -1;
int NUM_OF_TS_FIELDS = 3;
int NUM_OF_IMU_FIELDS = 7;
int PLD_STRT_INDX = 2; // payload starting index in received string line
int IMU_MEAS_STRT_INDX = 4;
std::string TS_SOURCE = "";

class FieldsCount{
  public:
    int count;   
    FieldsCount (int start = 0) {
    	count = start;
    }
    void add (int additive) {
    	count += additive;
    }
    int current (void) {
    	return count;
    }
};

std::vector<int16_t> string_to_ints(std::string str, int start_from = 0) { 
	std::stringstream ss;	 
	/* Storing the whole string into string stream */
	ss << str.substr(start_from); 
	/* Running loop till the end of the stream */
	std::string temp; 
	int found; 
	
	std::vector<int16_t> ints;        

	while (!ss.eof()) { 
		/* extracting word by word from stream */
		ss >> temp; 
		/* Checking the given word is integer or not */
		if (std::stringstream(temp) >> std::hex >> found) {
			ints.push_back(static_cast<int16_t>(found));
		}
		temp = ""; 
	} 
	return ints;
} 

std::vector<int16_t> subvector(std::vector<int16_t> const &initial_v, int starting_index) {
   std::vector<int16_t> sub_v(initial_v.begin() + starting_index, initial_v.end());
   return sub_v;
}

ros::Time ints_to_board_ts(std::vector<int16_t> input_ints, FieldsCount * fc_pointer, int first_element=0) {
	std::vector<int16_t> ints = subvector(input_ints, fc_pointer->current());
	
	double secs = static_cast<uint16_t>(ints[0]) * 60.0 + static_cast<uint16_t>(ints[1]) * 1.0 + (FRACT_NUMBER - static_cast<uint16_t>(ints[2]))/(FRACT_NUMBER + 1.0);
	ros::Time board_ts = ros::Time(secs);

	fc_pointer->add(NUM_OF_TS_FIELDS);
	return board_ts;
}

boost::numeric::ublas::vector<double> ints_to_imu_meas(std::vector<int16_t> input_ints, FieldsCount * fc_pointer, int first_element=0) {
	std::vector<int16_t> ints = subvector(input_ints, fc_pointer->current());
	boost::numeric::ublas::vector<double> imu_meas(NUM_OF_IMU_FIELDS);
	for (int i = 0; i < imu_meas.size(); i++) {
		// acc
		if (i < 3) {
			imu_meas(i) = ints[i] / 16384.0 * G;
		}
		// temperature
		else if (i == 3) {
			imu_meas(i) = ints[3] / 340.0 + 35.0;
		}
		// gyro
		else if (i >= 3) {
			imu_meas(i) =  ints[i] / 131.0 / 180.0 * PI;
		}
	}

	fc_pointer->add(NUM_OF_IMU_FIELDS);
	return imu_meas;
}

void publish_imu(ros::Publisher pub, uint8_t imu_n, ros::Time ts, boost::numeric::ublas::vector<double> imu_meas) {
    // publish_imu data [a in m/s^2] and [w in rad/s]
	std::string frame_id = IMU_TOPIC_PREFIX + std::to_string(imu_n);
    sensor_msgs::Imu msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = ts;
    // linear_acceleration
    msg.linear_acceleration.x = imu_meas[0];
    msg.linear_acceleration.y = imu_meas[1];
    msg.linear_acceleration.z = imu_meas[2];
    // angular_velocity
    msg.angular_velocity.x = imu_meas[4];
    msg.angular_velocity.y = imu_meas[5];
    msg.angular_velocity.z = imu_meas[6];
    // Publish the message.
    pub.publish(msg);
}

void publish_imu_temperature(ros::Publisher pub, uint8_t imu_n, ros::Time ts, boost::numeric::ublas::vector<double> imu_meas) {
    std::string frame_id = IMU_TOPIC_PREFIX + std::to_string(imu_n) + IMU_TEMP_TOPIC_POSTFIX;
    sensor_msgs::Temperature msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = ts;
    msg.temperature = imu_meas[3];
    
    pub.publish(msg);
}

void publish_cameras_ts(ros::Publisher pub, ros::Time ts) {
    std::string frame_id = CAMERAS_TS_TOPIC;
    sensor_msgs::TimeReference msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = ts;
    //msg.time_ref
    pub.publish(msg);
}

void publish_lidar_ts(ros::Publisher pub, ros::Time ts) {
    std::string frame_id = LIDAR_TS_TOPIC;
    sensor_msgs::TimeReference msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = ts;
    //msg.time_ref
    pub.publish(msg);
}

void pub_distributer(std::string str) {
}

int main(int argc, char **argv) {
	// Register signal and signal handler
    if (argc < 2) {
        std::cout << "Please, specify serial device. For example, \"/dev/ttyUSB0\"" << std::endl;
        return 0;
    }
    if (argc < 3) {
        std::cout << "Please, specify number of IMUs. For example, \"4\"" << std::endl;
        return 0;
    }
    
    PORT = argv[1];
    NUM_OF_IMUS = std::stoi(argv[2]);
    bool read_laptop_time = true;
    bool board_starting_ts_is_read = false;
    std::string str;
    
    ros::Time ts, laptop_ts, board_ts;
    ros::Duration delta_ts;
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "it_is_here");
    

    // Create a publisher object.
	ros::NodeHandle nh;
    ros::Publisher imu_pubs[NUM_OF_IMUS];

    for (uint imu_n = 0; imu_n < NUM_OF_IMUS; imu_n++) {
        imu_pubs[imu_n] = nh.advertise<sensor_msgs::Imu>(IMU_TOPIC_PREFIX + std::to_string(imu_n), RATE);
    }

	//ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(IMU_TOPIC_PREFIX, RATE);
	ros::Publisher imu_temp_pub = nh.advertise<sensor_msgs::Temperature>(IMU_TEMP_TOPIC, RATE);
	//ros::Publisher cameras_ts_pub = nh.advertise<sensor_msgs::TimeReference>(CAMERAS_TS_TOPIC, RATE);
	//ros::Publisher lidar_ts_pub = nh.advertise<sensor_msgs::TimeReference>(LIDAR_TS_TOPIC, RATE);
	
    // open port, baudrate, timeout in milliseconds
    serial::Serial serial(PORT, BAUD, serial::Timeout::simpleTimeout(TIMOUT));

    // check if serial port open
    std::cout << "Serial port is..." << std::flush;
    if(serial.isOpen())
        std::cout << " open." << std::endl;
    else
        std::cout << " not open!" << std::endl;

    // Clean from possibly broken string
    std::cout << "Waiting for data..." << std::flush;

    int ii = 0;

    while(serial.available()) {
        str = serial.readline();
    }
    while(serial.available() <= TEMP_BUF_SIZE);
    std::cout << " ok." << std::endl;
    
    while(true) {
        if(serial.available() > TEMP_BUF_SIZE) {
        	str = serial.readline(); 
        	if(str.at(str.size()-1)=='\n') {
        		break;
        	}
        }
    }
    //while(true);    
    int n_of_strings = 0;
    // Main loop
    //while(1);
    while(ros::ok()) {
        if(serial.available() > TEMP_BUF_SIZE) {
        	str = serial.readline();
            n_of_strings++;
        	//std::cout << str << std::endl;
            std::vector<int16_t> ints = string_to_ints(str, 0);
            
            FieldsCount fields_count_trash(0);
            board_ts = ints_to_board_ts(ints, &fields_count_trash);
            if (read_laptop_time == true) {
                laptop_ts = ros::Time::now();
                delta_ts = laptop_ts - board_ts;
                read_laptop_time = false;
            }
            ts = board_ts + delta_ts;
            //std::cout << ts << ' ' << laptop_ts << ' ' << board_ts << ' ' << delta_ts << ' ' << std::endl;

            //ros::Time ts = ros::Time::now();
            //std::cout << fields_count.current() << std::endl;
            
            FieldsCount fields_count(IMU_MEAS_STRT_INDX);
    		for(int imu_n = 0; imu_n < NUM_OF_IMUS; imu_n++) {
	   			boost::numeric::ublas::vector<double> imu_meas;
				imu_meas = ints_to_imu_meas(ints, &fields_count);

                //std::cout << fields_count.current() << std::endl;
            

				publish_imu(imu_pubs[imu_n], imu_n, ts, imu_meas);
				publish_imu_temperature(imu_temp_pub, imu_n, ts, imu_meas);
			}
            if (n_of_strings%1000==0) {
                std::cout << n_of_strings << ": " << str << std::endl;
            }
	    }
	}
}
