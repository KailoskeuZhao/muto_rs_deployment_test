#include <iostream>
#include <chrono>
#include <functional>
#include <memory>

#include <string>
#include <algorithm>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "CYdLidar.h"
#include "filters/NoiseFilter.h"

using namespace std;
using namespace std::chrono_literals;
using namespace ydlidar;

void LaserScanToMsg(
	const LaserScan &scan,
	sensor_msgs::msg::PointCloud2 &cloud)
{
	cloud.header.frame_id = "lidar_frame";

	cloud.height = 1;
	cloud.width = scan.points.size();

	sensor_msgs::msg::PointField field_x;
	field_x.name = "x";
	field_x.offset = 0;
	field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_x.count = 1;

	sensor_msgs::msg::PointField field_y;
	field_y.name = "y";
	field_y.offset = 4;
	field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_y.count = 1;

	sensor_msgs::msg::PointField field_z;
	field_z.name = "z";
	field_z.offset = 8;
	field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_z.count = 1;
/*
	sensor_msgs::msg::PointField field_i;
	field_i.name = "intensity";
	field_i.offset = 12;
	field_i.datatype = sensor_msgs::msg::PointField::FLOAT32;
	field_i.count = 1;
*/
	cloud.fields = {field_x, field_y, field_z};
	cloud.point_step = 12;
	cloud.row_step = cloud.point_step * cloud.width;

	cloud.data.resize(cloud.row_step);
	
	struct PointXYZ {
	 	    float x;
	   	    float y;
		    float z;
		    //float intensity;
		};

	for (std::size_t i = 0; i < cloud.width; ++i) {
		  auto* p = reinterpret_cast<PointXYZ*>(&cloud.data[i * cloud.point_step]);
		    
		    LaserPoint point = scan.points[i];

		    p->x = cos(point.angle) * point.range; 
		    p->y = sin(point.angle) * point.range;
		    p->z = 0;
		    //p->intensity = point.intensity;
		}
	

	cloud.is_bigendian = false;
	cloud.is_dense = true;

}

class LidarNode : public rclcpp::Node
{
	public:
	LidarNode() : Node("lidar_node"){
		      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/PointCloud",5);
		      timer_ = this->create_wall_timer(125ms, std::bind(&LidarNode::publish_lidar_info, this));
		      
		      ydlidar::os_init();

		      string port = "/dev/mylidar";
		      laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
		      std::string ignore_array;
		      ignore_array.clear();
		      laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

		      int baudrate = 512000;
		      laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
		      
		      int optval = TYPE_TOF;
		      laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

		      optval = YDLIDAR_TYPE_SERIAL;
		      laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

		      optval = 20;
		      laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

		      optval = 4;
		      laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

		      optval = 0;
		      laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

		      bool b_optvalue = false;
		      laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));

		      b_optvalue = false;
		      laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));

		      b_optvalue = false;
		      laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
		      b_optvalue = true;
		      laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

		      b_optvalue = false;
		      laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));

		      b_optvalue = false;
		      laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));

		      laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));


		        float f_optvalue = 180.0f;
			  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
			    f_optvalue = -180.0f;
			      laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

			  f_optvalue = 64.f;
			    laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
			      f_optvalue = 0.05f;
			        laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
			
			float frequency = 8.0;
			laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

			  bool ret = laser.initialize();

			    if (ret) {
				     	// Start the device scanning routine which runs on a separate thread and enable motor.
				         ret = laser.turnOn();
				     } else {//failed
				         RCLCPP_DEBUG(this->get_logger(), laser.DescribeError());
				     }                  
		}
	private:
			  void publish_lidar_info(){
				if((this->laser).doProcessSimple(this->scan)){
					sensor_msgs::msg::PointCloud2 msg;
					LaserScanToMsg(this->scan, msg);
					msg.header.stamp = this->get_clock()->now().to_msg();
					publisher_->publish(msg);
				}	  
			  }
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
	CYdLidar laser;
	LaserScan scan;
};

int main(int argc, char * argv[])
{
	  rclcpp::init(argc, argv);
	    rclcpp::spin(std::make_shared<LidarNode>());
	      rclcpp::shutdown();
	        return 0;
}
