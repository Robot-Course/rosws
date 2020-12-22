#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace everest::hwdrivers;
using namespace std::chrono_literals;

typedef struct _rslidar_data {
    _rslidar_data()
    {
        signal = 0;
        angle = 0.0;
        distance = 0.0;
    }
    uint8_t signal;
    float   angle;
    float   distance;
} RslidarDataComplete;

class ScanPublisher : public rclcpp::Node {
    public:
        ScanPublisher() : Node("delta_2a_lidar_publisher") {
            this->declare_parameter("serial_port");
            this->declare_parameter("frame_id");
            
            this->get_parameter_or<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
            this->get_parameter_or<std::string>("frame_id", frame_id, "lidar");

            CSerialConnection* serial_connect = new CSerialConnection();
            robotics_lidar = new C3iroboticsLidar();

            serial_connect->setBaud(baud_rate);
            serial_connect->setPort(serial_port.c_str());

            if(serial_connect->openSimple()) {
                RCLCPP_INFO(this->get_logger(), "[AuxCtrl] Open serial port sucessful!");
            } else {
                RCLCPP_INFO(this->get_logger(), "[AuxCtrl] Open serial port %s failed!", serial_port.c_str());
                throw -1;
            }

            RCLCPP_INFO(this->get_logger(), "[Main] 3iRoboticsLidar connected.");
             
            robotics_lidar->initilize(serial_connect);

            start_scan_time = this->now();
            publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 100);
            timer = this->create_wall_timer(50us, std::bind(&ScanPublisher::poll, this));
        }

        ~ScanPublisher() {
            delete robotics_lidar;
        }

    private:
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Time start_scan_time;
        rclcpp::Time end_scan_time;
        std::string serial_port;
        std::string frame_id;
        int baud_rate = 230400;
        C3iroboticsLidar* robotics_lidar;

        void publish(_rslidar_data *nodes, size_t node_count) {
            auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
            
            end_scan_time = this->now();
            double scan_duration = (end_scan_time - start_scan_time).seconds() * 1e-3;

            scan->header.stamp = start_scan_time;
            scan->header.frame_id = frame_id;
            scan->angle_min = DEG2RAD(0.0f);
            scan->angle_max = DEG2RAD(359.0f);
            scan->angle_increment = (scan->angle_max - scan->angle_min) / (360.0f - 1.0f);

            scan->scan_time = scan_duration;
            scan->time_increment = scan_duration / (double)(node_count - 1);
            scan->range_min = 0.15;
            scan->range_max = 5.0;

            scan->ranges.resize(360, std::numeric_limits<float>::infinity());
            scan->intensities.resize(360, 0.0);

            for (size_t i = 0; i < node_count; i++) {
                size_t current_angle = floor(nodes[i].angle);
                if(current_angle > 360.0)
                {
                    RCLCPP_INFO(this->get_logger(), "[Main] Lidar angle is out of range %d\n", (int)current_angle);
                    continue;
                }
                float read_value = (float) nodes[i].distance;
                if (read_value < scan->range_min || read_value > scan->range_max)
                    scan->ranges[360 - 1 - current_angle] = std::numeric_limits<float>::infinity();
                else
                    scan->ranges[360 - 1 - current_angle] = read_value;

                float intensities = (float) nodes[i].signal;
                scan->intensities[360 - 1 - current_angle] = intensities;
            }
            publisher->publish(*scan);
        }

        void poll() {
            TLidarGrabResult result = robotics_lidar->getScanData();
	        switch(result){
                case LIDAR_GRAB_ING: {
                    break;
                }
                case LIDAR_GRAB_SUCESS: {
                    TLidarScan lidar_scan = robotics_lidar->getLidarScan();
                    size_t lidar_scan_size = lidar_scan.getSize();
                    std::vector<RslidarDataComplete> send_lidar_scan_data;
                    send_lidar_scan_data.resize(lidar_scan_size);
                    RslidarDataComplete one_lidar_data;
                    for(size_t i = 0; i < lidar_scan_size; i++) {
                        one_lidar_data.signal = lidar_scan.signal[i];
                        one_lidar_data.angle = lidar_scan.angle[i];
                        one_lidar_data.distance = lidar_scan.distance[i];
                        send_lidar_scan_data[i] = one_lidar_data;
                    }

                    RCLCPP_INFO(this->get_logger(), "[Main] Receive Lidar count %u.\n", lidar_scan_size);
                    
                    this->publish(&send_lidar_scan_data[0], lidar_scan_size);

                    start_scan_time = end_scan_time;
                    break;
                }
                case LIDAR_GRAB_ERRO: {
                    RCLCPP_INFO(this->get_logger(), "[Main] LIDAR_ERROR!\n");
                    break;
                }
                case LIDAR_GRAB_ELSE: {
                    RCLCPP_INFO(this->get_logger(), "[Main] LIDAR_GRAB_ELSE!\n");
                    break;
                }
            }
        }

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanPublisher>());
    rclcpp::shutdown();
    return 0;
}
