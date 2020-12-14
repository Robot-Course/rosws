#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"

#include "serial/serial.h"
#include "Trilateration/Position.h"
#include "Trilateration/Trilateration.h"
#include "kalman.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class UWBPublisher : public rclcpp::Node
{
public:
    UWBPublisher(const char *uwb_device = "/dev/ttyUSB0",
                 const char *base_frame_id = "base_footprint",
                 const char *odom_frame_id = "odom",
                 const char *uwb_frame_id = "uwb",
                 const char *imu_topic = "/imu",
                 int baud = 115200
                 ) : Node("uwb_publisher"), 
                 uwb_device_(uwb_device, baud, serial::Timeout::simpleTimeout(1000)),
                 station1_(2.23, 0), station2_(0, 0), station3_(0, 6.00), station4_(2.23, 6.00),
                 kalman1_(0.125, 32, 1023, 0), kalman2_(0.125, 32, 1023, 0), kalman3_(0.125, 32, 1023, 0), kalman4_(0.125, 32, 1023, 0) 
    {
        this->base_frame_id_ = base_frame_id;
        this->odom_frame_id_ = odom_frame_id;
        this->uwb_frame_id_ = uwb_frame_id;
        this->latest_tf_ = tf2::Transform::getIdentity();
        this->broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        this->uwb_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("uwb", rclcpp::QoS(100));

        this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        this->imu_listener_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(), std::bind(&UWBPublisher::imu_callback, this, _1));
    }

private:
    serial::Serial uwb_device_;
    const char *base_frame_id_;
    const char *odom_frame_id_;
    const char *uwb_frame_id_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::Transform latest_tf_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr uwb_publisher_;

    Trilateration worker_;
    Kalman kalman1_;
    Kalman kalman2_;
    Kalman kalman3_;
    Kalman kalman4_;
    Pos2d station1_;
    Pos2d station2_;
    Pos2d station3_;
    Pos2d station4_;

    std::vector<std::string> split(const std::string &s, const char delim)
    {
        std::vector<std::string> elems;

        std::stringstream ss(s);
        std::string item;

        while (std::getline(ss, item, delim))
        {
            elems.push_back(item);
        }

        return elems;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::string buffer = uwb_device_.readline();
        std::vector<std::string> s = split(buffer, ',');

        double _x = std::stod(s[0]) / 100;
        double _y = std::stod(s[1]) / 100;
        double d1 = kalman1_.getFilteredValue(std::stod(s[2]) / 100);
        double d2 = kalman2_.getFilteredValue(std::stod(s[3]) / 100);
        double d3 = kalman3_.getFilteredValue(std::stod(s[4]) / 100);
        double d4 = kalman4_.getFilteredValue(std::stod(s[5]) / 100);
        RCLCPP_INFO(this->get_logger(), "Get: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", _x, _y, d1, d2, d3, d4);
        PosAndDistance2dVec beacons;
        Pos2d position;
        beacons.push_back(PosAndDistance2d(station1_, d1));
        beacons.push_back(PosAndDistance2d(station2_, d2));
        beacons.push_back(PosAndDistance2d(station3_, d3));
        beacons.push_back(PosAndDistance2d(station4_, d4));
        worker_.CalculateLocation2d(beacons, position);

        RCLCPP_INFO(this->get_logger(), "Filtered: %.2f, %.2f", position[0], position[1]);
        geometry_msgs::msg::PointStamped uwb_msg;
        uwb_msg.header.frame_id = base_frame_id_;
        uwb_msg.header.stamp = msg->header.stamp;
        uwb_msg.point.x = position[0];
        uwb_msg.point.y = position[1];
        uwb_msg.point.z = 0.0;
        uwb_publisher_->publish(uwb_msg);
        
        tf2::Quaternion orientation;
        tf2::impl::Converter<true, false>::convert(msg->orientation, orientation);

        geometry_msgs::msg::PoseStamped odom_to_uwb;
        
        tf2::Transform tmp_tf(orientation, tf2::Vector3(position[0], position[1], 0.0));
        geometry_msgs::msg::PoseStamped tmp_pose_stamped;
        tmp_pose_stamped.header.frame_id = base_frame_id_;
        tmp_pose_stamped.header.stamp = msg->header.stamp;
        tf2::toMsg(tmp_tf.inverse(), tmp_pose_stamped.pose);
        tf_buffer_->transform(tmp_pose_stamped, odom_to_uwb, odom_frame_id_, tf2::durationFromSec(1.0));
        tf2::impl::Converter<true, false>::convert(odom_to_uwb.pose, latest_tf_);

        geometry_msgs::msg::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = uwb_frame_id_;
        tmp_tf_stamped.header.stamp = get_clock().get()->now() + tf2::durationFromSec(1.0);
        tmp_tf_stamped.child_frame_id = odom_frame_id_;
        tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
        broadcaster_->sendTransform(tmp_tf_stamped);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //std::make_shared<UWBPublisher>()->loop();
    rclcpp::spin(std::make_shared<UWBPublisher>());
    rclcpp::shutdown();
    return 0;
}