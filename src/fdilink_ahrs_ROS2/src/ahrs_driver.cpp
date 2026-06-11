#include <ahrs_driver.h>
//#include <Eigen/Eigen>
rclcpp::Node::SharedPtr nh_=nullptr;

bool publish_acc_ = true;
bool publish_gyro_ = true;
bool publish_euler_ = true;

namespace FDILink
{
ahrsBringup::ahrsBringup()
: rclcpp::Node ("ahrs_bringup")
{
  //保留原有必需参数
  this->declare_parameter("if_debug_",false);
  this->get_parameter("if_debug_", if_debug_);

  this->declare_parameter<std::int8_t>("device_type_",1);
  this->get_parameter("device_type_",  device_type_);

  this->declare_parameter<std::string>("imu_topic","/imu");
  this->get_parameter("imu_topic",  imu_topic);

  this->declare_parameter<std::string>("imu_frame_id_","gyro_link");
  this->get_parameter("imu_frame_id_",   imu_frame_id_);

  this->declare_parameter<std::string>("Euler_angles_topic","/euler_angles");
  this->get_parameter("Euler_angles_topic", Euler_angles_topic);

  this->declare_parameter<std::string>("serial_port_","/dev/fdilink_ahrs");
  this->get_parameter("serial_port_", serial_port_);

  this->declare_parameter<std::int64_t>("serial_baud_",921600);
  this->get_parameter("serial_baud_", serial_baud_);

  //新增三轴数据发布开关参数
  this->declare_parameter<bool>("publish_acc", true);
  this->get_parameter("publish_acc", publish_acc_);
  this->declare_parameter<bool>("publish_gyro", true);
  this->get_parameter("publish_gyro", publish_gyro_);
  this->declare_parameter<bool>("publish_euler", true);
  this->get_parameter("publish_euler", publish_euler_);

  //只保留需要的两个发布者，删除gps/mag/twist/odom发布定义
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic.c_str(), 10);
  Euler_angles_pub_ = create_publisher<geometry_msgs::msg::Vector3>(Euler_angles_topic.c_str(), 10);

  //串口初始化原样保留，无改动
  try
  {
    serial_.setPort(serial_port_);
    serial_.setBaudrate(serial_baud_);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
    serial_.setTimeout(time_out);
    serial_.open();
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to open port ");
    exit(0);
  }
  if (serial_.isOpen())
  {
    RCLCPP_INFO(this->get_logger(),"Serial Port initialized");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to initial Serial port ");
    exit(0);
  }
  processLoop();
}

ahrsBringup::~ahrsBringup()
{
  if (serial_.isOpen())
    serial_.close();
}

void ahrsBringup::processLoop()
{
  RCLCPP_INFO(this->get_logger(),"ahrsBringup::processLoop: start");
  while (rclcpp::ok())
  {
    if (!serial_.isOpen())
    {
      RCLCPP_WARN(this->get_logger(),"serial unopen");
    }
    uint8_t check_head[1] = {0xff};
    size_t head_s = serial_.read(check_head, 1);
    if (if_debug_){
      if (head_s != 1)
      {
        RCLCPP_ERROR(this->get_logger(),"Read serial port time out! can't read pack head.");
      }
      std::cout << std::endl;
      std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
    }
    if (check_head[0] != FRAME_HEAD)
    {
      continue;
    }
    uint8_t head_type[1] = {0xff};
    size_t type_s = serial_.read(head_type, 1);
    if (if_debug_){
      std::cout << "head_type:  " << std::hex << (int)head_type[0] << std::dec << std::endl;
    }
    if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && head_type[0] != TYPE_GEODETIC_POS && head_type[0] != 0x50 && head_type[0] != TYPE_GROUND&& head_type[0] != 0xff)
    {
      RCLCPP_WARN(this->get_logger(),"head_type error: %02X",head_type[0]);
      continue;
    }
    uint8_t check_len[1] = {0xff};
    size_t len_s = serial_.read(check_len, 1);
    if (if_debug_){
      std::cout << "check_len: "<< std::dec << (int)check_len[0]  << std::endl;
    }
    if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)
    {
      RCLCPP_WARN(this->get_logger(),"head_len error (imu)");
      continue;
    }else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)
    {
      RCLCPP_WARN(this->get_logger(),"head_len error (ahrs)");
      continue;
    }else if (head_type[0] == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
    {
      RCLCPP_WARN(this->get_logger(),"head_len error (insgps)");
      continue;
    }else if (head_type[0] == TYPE_GEODETIC_POS && check_len[0] != GEODETIC_POS_LEN)
    {
      RCLCPP_WARN(this->get_logger(),"head_len error (GEODETIC_POS)");
      continue;
    }
    else if (head_type[0] == TYPE_GROUND || head_type[0] == 0x50)
    {
      uint8_t ground_sn[1];
      size_t ground_sn_s = serial_.read(ground_sn, 1);
      if (++read_sn_ != ground_sn[0])
      {
        if ( ground_sn[0] < read_sn_)
        {
          if(if_debug_){
            RCLCPP_WARN(this->get_logger(),"detected sn lost.");
          }
          sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
          read_sn_ = ground_sn[0];
        }
        else
        {
          if(if_debug_){
            RCLCPP_WARN(this->get_logger(),"detected sn lost.");
          }
          sn_lost_ += (int)(ground_sn[0] - read_sn_);
          read_sn_ = ground_sn[0];
        }
      }
      uint8_t ground_ignore[500];
      size_t ground_ignore_s = serial_.read(ground_ignore, (check_len[0]+4));
      continue;
    }
    uint8_t check_sn[1] = {0xff};
    size_t sn_s = serial_.read(check_sn, 1);
    uint8_t head_crc8[1] = {0xff};
    size_t crc8_s = serial_.read(head_crc8, 1);
    uint8_t head_crc16_H[1] = {0xff};
    uint8_t head_crc16_L[1] = {0xff};
    size_t crc16_H_s = serial_.read(head_crc16_H, 1);
    size_t crc16_L_s = serial_.read(head_crc16_L, 1);
    if (if_debug_){
      std::cout << "check_sn: "     << std::hex << (int)check_sn[0]     << std::dec << std::endl;
      std::cout << "head_crc8: "    << std::hex << (int)head_crc8[0]    << std::dec << std::endl;
      std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
      std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
    }
    if (head_type[0] == TYPE_IMU)
    {
      imu_frame_.frame.header.header_start   = check_head[0];
      imu_frame_.frame.header.data_type      = head_type[0];
      imu_frame_.frame.header.data_size      = check_len[0];
      imu_frame_.frame.header.serial_num     = check_sn[0];
      imu_frame_.frame.header.header_crc8    = head_crc8[0];
      imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
      if (CRC8 != imu_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(),"header_crc8 error");
        continue;
      }
      if(!frist_sn_){
        read_sn_  = imu_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      ahrsBringup::checkSN(TYPE_IMU);
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      ahrs_frame_.frame.header.header_start   = check_head[0];
      ahrs_frame_.frame.header.data_type      = head_type[0];
      ahrs_frame_.frame.header.data_size      = check_len[0];
      ahrs_frame_.frame.header.serial_num     = check_sn[0];
      ahrs_frame_.frame.header.header_crc8    = head_crc8[0];
      ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
      if (CRC8 != ahrs_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(),"header_crc8 error");
        continue;
      }
      if(!frist_sn_){
        read_sn_  = ahrs_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      ahrsBringup::checkSN(TYPE_AHRS);
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      insgps_frame_.frame.header.header_start   = check_head[0];
      insgps_frame_.frame.header.data_type      = head_type[0];
      insgps_frame_.frame.header.data_size      = check_len[0];
      insgps_frame_.frame.header.serial_num     = check_sn[0];
      insgps_frame_.frame.header.header_crc8    = head_crc8[0];
      insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
      if (CRC8 != insgps_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(),"header_crc8 error");
        continue;
      }
      else if(if_debug_)
      {
        std::cout << "header_crc8 matched." << std::endl;
      }
      ahrsBringup::checkSN(TYPE_INSGPS);
    }
    else if (head_type[0] == TYPE_GEODETIC_POS)
    {
      Geodetic_Position_frame_.frame.header.header_start   = check_head[0];
      Geodetic_Position_frame_.frame.header.data_type      = head_type[0];
      Geodetic_Position_frame_.frame.header.data_size      = check_len[0];
      Geodetic_Position_frame_.frame.header.serial_num     = check_sn[0];
      Geodetic_Position_frame_.frame.header.header_crc8    = head_crc8[0];
      Geodetic_Position_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      Geodetic_Position_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(Geodetic_Position_frame_.read_buf.frame_header, 4);
      if (CRC8 != Geodetic_Position_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(),"header_crc8 error");
        continue;
      }
      if(!frist_sn_){
        read_sn_  = Geodetic_Position_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      ahrsBringup::checkSN(TYPE_GEODETIC_POS);
    }
    if (head_type[0] == TYPE_IMU)
    {
      uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1));
      uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
      if (if_debug_)
      {
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(),"check crc16 faild(imu).");
        continue;
      }
      else if(imu_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(),"check frame end.");
        continue;
      }
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1));
      uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
      if (if_debug_){
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(),"check crc16 faild(ahrs).");
        continue;
      }
      else if(ahrs_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(),"check frame end.");
        continue;
      }
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      uint16_t head_crc16_l = insgps_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = insgps_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1));
      uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
      if (if_debug_){
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(),"check crc16 faild(ahrs).");
        continue;
      }
      else if(insgps_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(),"check frame end.");
        continue;
      }
    }
    else if (head_type[0] == TYPE_GEODETIC_POS)
    {
      uint16_t head_crc16_l = Geodetic_Position_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = Geodetic_Position_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(Geodetic_Position_frame_.read_buf.read_msg, (GEODETIC_POS_LEN + 1));
      uint16_t CRC16 = CRC16_Table(Geodetic_Position_frame_.frame.data.data_buff, GEODETIC_POS_LEN);
      if (if_debug_){
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(),"check crc16 faild(gps).");
        continue;
      }
      else if(Geodetic_Position_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(),"check frame end.");
        continue;
      }
    }

    if (head_type[0] == TYPE_IMU)
    {
      sensor_msgs::msg::Imu imu_data;
      imu_data.header.stamp = rclcpp::Node::now();
      imu_data.header.frame_id = imu_frame_id_.c_str();
      imu_data.orientation.w = 0;
      imu_data.orientation.x = 0;
      imu_data.orientation.y = 0;
      imu_data.orientation.z = 0;
      if (publish_gyro_) {
        imu_data.angular_velocity.x = imu_frame_.frame.data.data_pack.gyroscope_x;
        imu_data.angular_velocity.y = imu_frame_.frame.data.data_pack.gyroscope_y;
        imu_data.angular_velocity.z = imu_frame_.frame.data.data_pack.gyroscope_z;
      }
      if (publish_acc_) {
        imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
        imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
        imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
      }
      imu_pub_->publish(imu_data);
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      if (publish_euler_) {
        geometry_msgs::msg::Vector3 Euler_angles_2d;
        Euler_angles_2d.x = ahrs_frame_.frame.data.data_pack.Roll* (180.0 / PI);
        Euler_angles_2d.y = ahrs_frame_.frame.data.data_pack.Pitch* (180.0 / PI);
        Euler_angles_2d.z = ahrs_frame_.frame.data.data_pack.Heading* (180.0 / PI);
        Euler_angles_pub_->publish(Euler_angles_2d);
      }
    }
    else if (head_type[0] == TYPE_GEODETIC_POS || head_type[0] == TYPE_INSGPS)
    {
      //丢弃数据，不发布任何话题
    }
  }
}

void ahrsBringup::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
{
  double temp1 = magy * cos(roll) + magz * sin(roll);
  double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
  magyaw = atan2(-temp1, temp2);
  if(magyaw < 0)
  {
    magyaw = magyaw + 2 * PI;
  }
}

void ahrsBringup::checkSN(int type)
{
  switch (type)
  {
  case TYPE_IMU:
    if (++read_sn_ != imu_frame_.frame.header.serial_num)
    {
      if ( imu_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
    }
    read_sn_ = imu_frame_.frame.header.serial_num;
    break;
  case TYPE_AHRS:
    if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
    {
      if ( ahrs_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
    }
    read_sn_ = ahrs_frame_.frame.header.serial_num;
    break;
  case TYPE_INSGPS:
    if (++read_sn_ != insgps_frame_.frame.header.serial_num)
    {
      if ( insgps_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
    }
    read_sn_ = insgps_frame_.frame.header.serial_num;
    break;
  case TYPE_GEODETIC_POS:
    if (++read_sn_ != Geodetic_Position_frame_.frame.header.serial_num)
    {
      if ( Geodetic_Position_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - Geodetic_Position_frame_.frame.header.serial_num);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(Geodetic_Position_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
    }
    read_sn_ = Geodetic_Position_frame_.frame.header.serial_num;
    break;
  default:
    break;
  }
}

} //namespace FDILink

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  FDILink::ahrsBringup bp;
  return 0;
}
