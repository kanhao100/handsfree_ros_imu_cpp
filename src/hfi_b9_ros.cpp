
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <serial/serial.h>
#include <cmath>
#include <tf/transform_broadcaster.h>

// 宏定义预编译
#define acc_factor  1.0000000 / 32768.0 * 16.0 * 9.8
#define angv_factor  1.0000000 / 32768.0 * 2000 * M_PI / 180
#define ang_factor 1.0000000 / 32768.0 * M_PI

serial::Serial imu_serial;
std::string imu_port;
int imu_baudrate;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
ros::Publisher imu_pub;
ros::Publisher mag_pub;

//校验和函数
bool checkSum(uint8_t* data, uint8_t check) {
    int sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += (data[i] & 0xff);
    }
    return (sum & 0xff) == check;
}

//处理串口数据函数
void handleSerialData(uint8_t* data) {
    uint8_t buff[11] = {0}; // 缓存数组
    bool pub_flag[4] = {true, true, true}; // 是否发布标志,依次为加速度、角速度、角度
    for (int i = 0; i < 4; i += 1)
    {
        for (int j = 0; j < 11; j += 1)
        {
            buff[j] = data[j + i*11];
        }

        if (buff[0] != 0x55) {
            ROS_WARN("Header Wrong");
            return;
        }

        switch (buff[1] & 0xff) {
            case 0x51: {
                if (pub_flag[0]) {
                    if (checkSum(buff, buff[10])) {
                        int16_t ax = ((uint16_t)buff[2]) | ((uint16_t)buff[3] << 8);
                        int16_t ay = ((uint16_t)buff[4]) | (uint16_t)buff[5] << 8;
                        int16_t az = ((uint16_t)buff[6]) | (uint16_t)buff[7] << 8;
                        imu_msg.linear_acceleration.x = ax * acc_factor;
                        imu_msg.linear_acceleration.y = ay * acc_factor;
                        imu_msg.linear_acceleration.z = az * acc_factor;
                        //std::cout << imu_msg.linear_acceleration.x << " " << imu_msg.linear_acceleration.y << " " << imu_msg.linear_acceleration.z << " ";
                    } else {
                        ROS_WARN("0x51 Verification Failed");
                    }
                    pub_flag[0] = false;
                }
                break;
            }
            case 0x52: {
                if (pub_flag[1]) {
                    if (checkSum(buff, buff[10])) {
                        int16_t gx = ((uint16_t)buff[2]) | ((uint16_t)buff[3] << 8);
                        int16_t gy = ((uint16_t)buff[4]) | ((uint16_t)buff[5] << 8);
                        int16_t gz = ((uint16_t)buff[6]) | ((uint16_t)buff[7] << 8);

                        imu_msg.angular_velocity.x = gx * angv_factor;
                        imu_msg.angular_velocity.y = gy * angv_factor;
                        imu_msg.angular_velocity.z = gz * angv_factor;
                        //std::cout << imu_msg.angular_velocity.x << " " << imu_msg.angular_velocity.y << " " << imu_msg.angular_velocity.z << " " << std::endl;
                    } else {
                        ROS_WARN("0x52 Verification Failed");
                    }
                    pub_flag[1] = false;
                }
                break;
            }
            case 0x53: {
            if (pub_flag[2]) {
                if (checkSum(buff, buff[10])) {
                    int16_t roll_raw = ((uint16_t)buff[2]) | ((uint16_t)buff[3] << 8);
                    int16_t pitch_raw = ((uint16_t)buff[4]) | ((uint16_t)buff[5] << 8);
                    int16_t yaw_raw = ((uint16_t)buff[6]) | ((uint16_t)buff[7] << 8);
                    double roll = roll_raw * ang_factor;
                    double pitch = pitch_raw * ang_factor;
                    double yaw = yaw_raw * ang_factor;
                    tf::Quaternion orientation1;
                    orientation1 = tf::createQuaternionFromRPY(roll, pitch, yaw);
                    imu_msg.orientation.x = orientation1.x();
                    imu_msg.orientation.y = orientation1.y();
                    imu_msg.orientation.z = orientation1.z();
                    imu_msg.orientation.w = orientation1.w();

                } else {
                    ROS_WARN("0x53 Verification Failed");
                }
                pub_flag[2] = false;
            }
            break;
            }
            case 0x54: {
                if (pub_flag[3]) {
                    if (checkSum(buff, buff[10])) {
                        int16_t mx = ((uint16_t)buff[2]) | ((uint16_t)buff[3] << 8);
                        int16_t my = ((uint16_t)buff[4]) | ((uint16_t)buff[5] << 8);
                        int16_t mz= ((uint16_t)buff[6]) | ((uint16_t)buff[7] << 8);
                        mag_msg.magnetic_field.x = mx;
                        mag_msg.magnetic_field.y = my;
                        mag_msg.magnetic_field.z = mz;
                    } else {
                        ROS_WARN("0x54 Verification Failed");
                    }
                    pub_flag[3] = false;
                }
                break;
            }
            default: {
                ROS_WARN("ALL Verification Failed", buff[1]);
                break;
            }
        }
    }

    if (pub_flag[0] || pub_flag[1] || pub_flag[2]|| pub_flag[3]) {
        return;
    }
    pub_flag[0] = pub_flag[1] = pub_flag[2] = pub_flag[3] = true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "handsfree");
    ros::NodeHandle nh("~");

    nh.param<std::string>("port", imu_port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", imu_baudrate, 921600);

    imu_serial.setPort(imu_port);
    imu_serial.setBaudrate(imu_baudrate);
    imu_serial.setTimeout(serial::Timeout::max(), 50, 0, 50, 0);

    try {
        imu_serial.open();
    } catch (const serial::IOException& e) {
        ROS_ERROR("Unable to open serial port %s", imu_port.c_str());
        return -1;
    }

    if (imu_serial.isOpen()) {
        ROS_INFO("Serial port %s opened successfully", imu_port.c_str());
    }

    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 10);
    mag_pub = nh.advertise<sensor_msgs::MagneticField>("/mag", 10);

    ros::Rate loop_rate(200);

    while (ros::ok()) {
        if (imu_serial.available() > 0) {
            size_t n = imu_serial.available();
            if(n!=0)
            {
                uint8_t buffer[44];
                n = imu_serial.read(buffer, n);
                handleSerialData(buffer);

                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = "base_link";
                imu_pub.publish(imu_msg);

                mag_msg.header.stamp = ros::Time::now();
                mag_msg.header.frame_id = "base_link";
                mag_pub.publish(mag_msg);
            }
        }
        loop_rate.sleep();
    }

    imu_serial.close();

    return 0;
}