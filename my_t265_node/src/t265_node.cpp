#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <librealsense2/rs.hpp>

class T265Node : public rclcpp::Node
{
public:
    T265Node() : Node("t265_node"), tf_broadcaster_(this)
    {
        RCLCPP_INFO(this->get_logger(), "Iniciando nodo T265...");

        // Configuración del pipeline de la cámara
        cfg_.enable_stream(RS2_STREAM_ACCEL);
        cfg_.enable_stream(RS2_STREAM_GYRO);
        cfg_.enable_stream(RS2_STREAM_POSE);
        pipe_.start(cfg_);

        // Crear publicadores
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("rs_t265/odom", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("rs_t265/imu", 10);

        // Temporizador para publicar datos periódicamente
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&T265Node::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Nodo T265 iniciado correctamente.");
    }

private:
    void timer_callback()
    {
        auto frames = pipe_.wait_for_frames();

        // Obtener datos de IMU
        if (auto gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
        {
            auto gyro_data = gyro_frame.as<rs2::motion_frame>().get_motion_data();
            imu_msg_.angular_velocity.x = gyro_data.x;
            imu_msg_.angular_velocity.y = gyro_data.y;
            imu_msg_.angular_velocity.z = gyro_data.z;
        }

        if (auto accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
        {
            auto accel_data = accel_frame.as<rs2::motion_frame>().get_motion_data();
            imu_msg_.header.stamp = this->now();
            imu_msg_.header.frame_id = "t265_frame";
            imu_msg_.linear_acceleration.x = accel_data.x;
            imu_msg_.linear_acceleration.y = accel_data.y;
            imu_msg_.linear_acceleration.z = accel_data.z;

            // Publicar el mensaje de IMU
            imu_publisher_->publish(imu_msg_);
        }

        // Obtener datos de pose
        if (auto pose_frame = frames.first_or_default(RS2_STREAM_POSE))
        {
            auto pose_data = pose_frame.as<rs2::pose_frame>().get_pose_data();
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "t265_frame";

            // Posición
            odom_msg.pose.pose.position.x = -pose_data.translation.z;
            odom_msg.pose.pose.position.y = -pose_data.translation.x;
            odom_msg.pose.pose.position.z = pose_data.translation.y;

            // Orientación
            odom_msg.pose.pose.orientation.x = -pose_data.rotation.z;
            odom_msg.pose.pose.orientation.y = -pose_data.rotation.x;
            odom_msg.pose.pose.orientation.z = pose_data.rotation.y;
            odom_msg.pose.pose.orientation.w = pose_data.rotation.w;

            // Velocidades
            odom_msg.twist.twist.linear.x = -pose_data.velocity.z;
            odom_msg.twist.twist.linear.y = -pose_data.velocity.x;
            odom_msg.twist.twist.linear.z = pose_data.velocity.y;

            odom_msg.twist.twist.angular.x = -pose_data.angular_velocity.z;
            odom_msg.twist.twist.angular.y = -pose_data.angular_velocity.x;
            odom_msg.twist.twist.angular.z = pose_data.angular_velocity.y;

            // Publicar odometría
            odom_publisher_->publish(odom_msg);

            // Publicar transformación TF2
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id = "t265_frame";
            tf_msg.header.stamp = this->now();

            tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
            tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
            tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

            tf_broadcaster_.sendTransform(tf_msg);
        }
    }

    // Publicadores
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // IMU message placeholder
    sensor_msgs::msg::Imu imu_msg_;

    // RealSense pipeline
    rs2::pipeline pipe_;
    rs2::config cfg_;

    // Temporizador
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<T265Node>());
    rclcpp::shutdown();
    return 0;
}
