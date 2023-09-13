
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
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
ros::Publisher imu_pub;

//校验和函数
inline bool checkSum(uint8_t* data, uint8_t check) {
    int sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += (data[i] & 0xff);
    }
    return (sum & 0xff) == check;
}

//处理串口数据函数
void handleSerialData(uint8_t* data) {
    uint8_t buff[11] = {0};
    int ok_flag = 0;
    for (int i = 0; i < 3; i += 1)
    {
        for (int j = 0; j < 11; j += 1)
        {
            if(j == 0){
                if (data[j + i*11] != 0x55) {
                    ROS_WARN("Header Wrong");
                    return;
                 }
            }
            buff[j] = data[j + i*11];
        }

        switch (buff[1] & 0xff) {
            case 0x51: {
                if (checkSum(buff, buff[10])) {
                    int16_t ax = ((uint16_t)buff[2]) | ((uint16_t)buff[3] << 8);
                    int16_t ay = ((uint16_t)buff[4]) | (uint16_t)buff[5] << 8;
                    int16_t az = ((uint16_t)buff[6]) | (uint16_t)buff[7] << 8;
                    imu_msg.linear_acceleration.x = ax * acc_factor;
                    imu_msg.linear_acceleration.y = ay * acc_factor;
                    imu_msg.linear_acceleration.z = az * acc_factor;
                    //std::cout << imu_msg.linear_acceleration.x << " " << imu_msg.linear_acceleration.y << " " << imu_msg.linear_acceleration.z << " ";
                    ok_flag+=1;
                } else {
                    ROS_WARN("0x51 Verification Failed");
                }
                break;
            }
            case 0x52: {
                if (checkSum(buff, buff[10])) {
                    int16_t gx = ((uint16_t)buff[2]) | ((uint16_t)buff[3] << 8);
                    int16_t gy = ((uint16_t)buff[4]) | ((uint16_t)buff[5] << 8);
                    int16_t gz = ((uint16_t)buff[6]) | ((uint16_t)buff[7] << 8);

                    imu_msg.angular_velocity.x = gx * angv_factor;
                    imu_msg.angular_velocity.y = gy * angv_factor;
                    imu_msg.angular_velocity.z = gz * angv_factor;
                    //std::cout << imu_msg.angular_velocity.x << " " << imu_msg.angular_velocity.y << " " << imu_msg.angular_velocity.z << " " << std::endl;
                    ok_flag+=1;
                } else {
                    ROS_WARN("0x52 Verification Failed");
                }
                break;
            }
            case 0x53: {
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
                    ok_flag+=1;
                } else {
                    ROS_WARN("0x53 Verification Failed");
                }
                break;
            }
            default: {
                ROS_WARN("ALL Verification Failed", buff[1]);
                ok_flag = 0;
                break;
            }
        }

    }

    if(3 == ok_flag){
        imu_msg.header.stamp = ros::Time::now();
        imu_pub.publish(imu_msg);
    }

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "handsfree");
    ros::NodeHandle nh("~");

    nh.param<std::string>("port", imu_port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", imu_baudrate, 921600);

    imu_serial.setPort(imu_port);
    imu_serial.setBaudrate(imu_baudrate);
    imu_serial.setTimeout(serial::Timeout::max(), 50, 0, 50, 0);

    imu_msg.header.frame_id = "base_link";
    
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

    ros::Rate loop_rate(200);

    size_t bytesRead = 0;
    std::vector<uint8_t> buffer(33);
    while (ros::ok()) {
        while (bytesRead < buffer.size()) {
            size_t bytesRemaining = buffer.size() - bytesRead;
            if(imu_serial.available()<bytesRemaining){
                loop_rate.sleep();
                continue;
            }

            size_t bytesReadNow = imu_serial.read(buffer.data() + bytesRead, bytesRemaining);
            if (bytesReadNow > 0) {
                bytesRead += bytesReadNow;
            } else {
                ROS_ERROR("Error reading data through serial port serial port %s", imu_port.c_str());
                break;
            }
        }

         // 查找数据包的起始位置
        auto startIt = std::find(buffer.begin(), buffer.end(), 0x55);
        if (startIt != buffer.end()) {
            size_t startIndex = std::distance(buffer.begin(), startIt);
            size_t alignedBytes = buffer.size() - startIndex;

            if(startIndex!=0){
                // 对齐数据
                std::rotate(buffer.begin(), startIt, buffer.end());
                // 保证数据正确
                imu_serial.read(buffer.data() + alignedBytes, startIndex);
            }
            handleSerialData(buffer.data());
        }
    }

    imu_serial.close();
    return 0;
}