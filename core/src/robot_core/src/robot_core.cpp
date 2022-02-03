#include "robot_core.h"
#include "rclcpp/rclcpp.hpp"
#include "Quaternion_Solution.h"

using std::placeholders::_1;
using namespace std;
//rclcpp::Node::SharedPtr node_handle = nullptr;
/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    RobotCore robotCore;
    robotCore.control();
    return 0;
}
/**************************************
Date: January 28, 2021
Function: Data conversion function
功能: 数据转换函数
**************************************/
short RobotCore::imuTrans(uint8_t dataHigh,uint8_t dataLow)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |=  dataHigh<<8;
    transition_16 |=  dataLow;
    return transition_16;
}
float RobotCore::odomTrans(uint8_t dataHigh,uint8_t dataLow)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |=  dataHigh<<8;  //Get the high 8 bits of data   //获取数据的高8位
    transition_16 |=  dataLow;      //Get the lowest 8 bits of data //获取数据的低8位
    data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
    return data_return;
}

void RobotCore::controlCallback(const robot_msg::msg::Control::SharedPtr cmdControl)
{
    short  transition;  //intermediate variable //中间变量
    sendData.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7BAkm_Cmd_Vel_Sub
    sendData.tx[1] = 0; //set aside //预留位
    sendData.tx[2] = 0; //set aside //预留位

    transition=1;
    transition = cmdControl->throttle*1000;
    sendData.tx[4] = transition;
    sendData.tx[3] = transition>>8;

    transition=2;
    transition = cmdControl->servo;
    sendData.tx[6] = transition;
    sendData.tx[5] = transition>>8;

    sendData.tx[7]=checkSum(7,SEND_DATA_CHECK); //For the BBC check bits, see the checkSum function //BBC校验位，规则参见Check_Sum函数
    sendData.tx[8]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
    
    // RCLCPP_INFO(this->get_logger(),"throttle: %f, servo: %d", cmdControl->throttle, cmdControl->servo);

    try
    {
        stm32Serial.write(sendData.tx,sizeof (sendData.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
    }
}

/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/
//void RobotCore::Publish_Voltage()
//{
//    std_msgs::msg::Float32 voltage_msgs; //Define the data type of the power supply voltage publishing topic //定义电源电压发布话题的数据类型
//    voltage_msgs.data = Power_voltage; //The power supply voltage is obtained //电源供电的电压获取
//    voltage_publisher->publish(voltage_msgs); //Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
//}

void RobotCore::publishReport()
{
    robot_msg::msg::Report reportMsg;

    reportMsg.throttle_a = reportData.throttle_a;
    reportMsg.throttle_b = reportData.throttle_b;
    reportMsg.servo = reportData.servo;
    reportMsg.voltage = reportData.voltage;
    reportMsg.velocity_x = reportData.velocity_x;
    reportMsg.velocity_y = reportData.velocity_y;
    reportMsg.velocity_z = reportData.velocity_z;
    reportMsg.acceleration_x = reportData.acceleration_x;
    reportMsg.acceleration_y = reportData.acceleration_y;
    reportMsg.acceleration_z = reportData.acceleration_z;
    reportMsg.angular_x = reportData.angular_x;
    reportMsg.angular_y = reportData.angular_y;
    reportMsg.angular_z = reportData.angular_z;
    // RCLCPP_INFO(this->get_logger(),"thro_A: %f, thro_B: %f, servo: %d", reportMsg.throttle_a, reportMsg.throttle_b, reportMsg.servo);
    reportPublisher->publish(reportMsg);
}

void RobotCore::publishPosition()
{

    // RCLCPP_INFO(this->get_logger(),"pos_x: %f, pos_y: %f, pos_z: %f", position.x, position.y, position.z);

    robot_msg::msg::Position positionMsg;

    positionMsg.x = position.x;
    positionMsg.y = position.y;
    positionMsg.z = position.z;
    positionPublisher->publish(positionMsg);
}

/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BBC check
Input parameter: countNumber: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BBC校验
输入参数： countNumber：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/

unsigned char RobotCore::checkSum(unsigned char countNumber,unsigned char mode)
{
    unsigned char sum=0,k;

    if(mode==0) //Receive data mode //接收数据模式
    {
        for(k=0;k<countNumber;k++)
        {
            sum=sum^receiveData.rx[k]; //By bit or by bit //按位异或
        }
    }
    if(mode==1) //Send data mode //发送数据模式
    {
        for(k=0;k<countNumber;k++)
        {
            sum=sum^sendData.tx[k]; //By bit or by bit //按位异或
        }
    }
    return sum; //Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
Date: January 28, 2021
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
功能: 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/

bool RobotCore::getSensorData()
{
    short transition_16=0, j=0, deaderPos=0, tailPos=0; //Intermediate variable //中间变量
    uint8_t receiveDataPr[RECEIVE_DATA_SIZE]={0}; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
    stm32Serial.read(receiveDataPr,sizeof (receiveDataPr)); //Read the data sent by the lower computer through the serial port //通过串口读取下位机发送过来的数据
    //Record the position of the head and tail of the frame //记录帧头帧尾位置
    for(j=0;j<30;j++)
    {
        if(receiveDataPr[j]==FRAME_HEADER)
            deaderPos=j;
        else if(receiveDataPr[j]==FRAME_TAIL)
            tailPos=j;
    }

    if(tailPos==(deaderPos+29))
    {
        //If the end of the frame is the last bit of the packet, copy the packet directly to receiveData.rx
        //如果帧尾在数据包最后一位，直接复制数据包到Receive_Data.rx
        memcpy(receiveData.rx, receiveDataPr, sizeof(receiveDataPr));
    }
    else if(deaderPos==(1+tailPos))
    {
        //如果帧头在帧尾后面，纠正数据位置后复制数据包到Receive_Data.rx
        // If the header is behind the end of the frame, copy the packet to receiveData.rx after correcting the data location
        for(j=0;j<30;j++)
            receiveData.rx[j]=receiveDataPr[(j+deaderPos)%30];
    }
    else
    {
        //其它情况则认为数据包有错误
        // In other cases, the packet is considered to be faulty
        return false;
    }

    receiveData.Frame_Header= receiveData.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
    receiveData.Frame_Tail= receiveData.rx[29];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

    if (receiveData.Frame_Header == FRAME_HEADER ) //Judge the frame header //判断帧头
    {
        if (receiveData.Frame_Tail == FRAME_TAIL) //Judge the end of the frame //判断帧尾
        {
            //BBC check passes or two packets are interlaced //BBC校验通过或者两组数据包交错
            if (receiveData.rx[28] == checkSum(28,READ_DATA_CHECK)||(deaderPos==(1+tailPos)))
            {
                receiveData.Flag_Stop=receiveData.rx[1]; //set aside //预留位
                reportData.velocity_x = odomTrans(receiveData.rx[2],receiveData.rx[3]);
                reportData.velocity_y = odomTrans(receiveData.rx[4],receiveData.rx[5]);
                reportData.velocity_z = odomTrans(receiveData.rx[6],receiveData.rx[7]);
//                Mpu6050_Data.accele_x_data = imuTrans(receiveData.rx[8],receiveData.rx[9]);   //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度
//                Mpu6050_Data.accele_y_data = imuTrans(receiveData.rx[10],receiveData.rx[11]); //Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
//                Mpu6050_Data.accele_z_data = imuTrans(receiveData.rx[12],receiveData.rx[13]); //Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
//                Mpu6050_Data.gyros_x_data = imuTrans(receiveData.rx[14],receiveData.rx[15]);  //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度
//                Mpu6050_Data.gyros_y_data = imuTrans(receiveData.rx[16],receiveData.rx[17]);  //Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度
//                Mpu6050_Data.gyros_z_data = imuTrans(receiveData.rx[18],receiveData.rx[19]);  //Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度
                reportData.acceleration_x = imuTrans(receiveData.rx[8],receiveData.rx[9]) / ACCEl_RATIO; //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度
                reportData.acceleration_y = imuTrans(receiveData.rx[10],receiveData.rx[11]) / ACCEl_RATIO;
                reportData.acceleration_z = imuTrans(receiveData.rx[12],receiveData.rx[13]) / ACCEl_RATIO;
                reportData.angular_x = imuTrans(receiveData.rx[14],receiveData.rx[15]) * GYROSCOPE_RATIO; //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度
                reportData.angular_y = imuTrans(receiveData.rx[16],receiveData.rx[17]) * GYROSCOPE_RATIO;
                reportData.angular_z = imuTrans(receiveData.rx[18],receiveData.rx[19]) * GYROSCOPE_RATIO;

                //Get the battery voltage
                //获取电池电压
                transition_16 = 0;
                transition_16 |=  receiveData.rx[20]<<8;
                transition_16 |=  receiveData.rx[21];
                // Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001; //Unit conversion millivolt(mv)->volt(v) //单位转换毫伏(mv)->伏(v)
                reportData.voltage = transition_16/1000+(transition_16 % 1000)*0.001f;

                // Get motor A throttle measurement
                transition_16 = 0;
                transition_16 |=  receiveData.rx[22]<<8;
                transition_16 |=  receiveData.rx[23];
                reportData.throttle_a = ((float)transition_16 * reportData.voltage) / 7200.f;

                // Get motor B throttle measurement
                transition_16 = 0;
                transition_16 |=  receiveData.rx[24]<<8;
                transition_16 |=  receiveData.rx[25];
                reportData.throttle_b = ((float)transition_16 * reportData.voltage) / 7200.f;

                transition_16 = 0;
                transition_16 |=  receiveData.rx[26]<<8;
                transition_16 |=  receiveData.rx[27];
                reportData.servo = transition_16;

                return true;
            }
        }
    }
    return false;
}
/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void RobotCore::control()
{
    while(rclcpp::ok())
    {
        if (true == getSensorData())
        {
            //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
            position.x += (reportData.velocity_x * cos(position.z)  - reportData.velocity_y * sin(position.z)) * 0.05;
            position.y += (reportData.velocity_x * sin(position.z)  + reportData.velocity_y * cos(position.z)) * 0.05;
            //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad
            position.z += reportData.velocity_z * 0.05 * 1.15;
            // position.z = 0;
            publishReport();
            publishPosition();
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }
}

/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
RobotCore::RobotCore() : rclcpp::Node ("robot_core") {
    memset(&receiveData, 0, sizeof(receiveData));
    memset(&sendData, 0, sizeof(sendData));
    memset(&reportData, 0, sizeof(reportData));
    memset(&position, 0, sizeof(position));

    int serial_baud_rate = 115200;
    usartPortName = "/dev/stm32_port";

    reportPublisher = create_publisher<robot_msg::msg::Report>("report", 10);
    positionPublisher = create_publisher<robot_msg::msg::Position>("position", 10);

    controlSubscriber = create_subscription<robot_msg::msg::Control>(
            "controlCommand", 100, std::bind(&RobotCore::controlCallback, this, _1));

    try
    {
        // Attempts to initialize and open the serial port //尝试初始化与开启串口
        stm32Serial.setPort(usartPortName); //Select the serial port number to enable //选择要开启的串口号
        stm32Serial.setBaudrate(serial_baud_rate); //Set the baud rate //设置波特率
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
        stm32Serial.setTimeout(_time);
        stm32Serial.open(); //Open the serial port //开启串口
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(),"robot can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
    }
    if(stm32Serial.isOpen())
    {
        RCLCPP_INFO(this->get_logger(),"robot serial port opened"); //Serial port opened successfully //串口开启成功提示
    }
}

/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
RobotCore::~RobotCore()
{
    //Sends the stop motion command to the lower machine before the turn_on_robot object ends
    //对象turn_on_robot结束前向下位机发送停止运动命令

    short  transition;  //intermediate variable //中间变量

    for (int i = 0; i < 10; ++i) {
        sendData.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7BAkm_Cmd_Vel_Sub
        sendData.tx[1] = 0; //set aside //预留位
        sendData.tx[2] = 0; //set aside //预留位

        sendData.tx[4] = 0;
        sendData.tx[3] = 0;

        transition=1500;
        sendData.tx[6] = transition;
        sendData.tx[5] = transition>>8;

        sendData.tx[7]=checkSum(7,SEND_DATA_CHECK); //For the BBC check bits, see the checkSum function //BBC校验位，规则参见Check_Sum函数
        sendData.tx[8]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D

        try
        {
            stm32Serial.write(sendData.tx,sizeof (sendData.tx)); //Send data to the serial port //向串口发数据
        }
        catch (serial::IOException& e)
        {
            RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
        }
        for (int j = 0; j < 1000; ++j) {

        }
    }

    stm32Serial.close(); //Close the serial port //关闭串口
    RCLCPP_INFO(this->get_logger(),"Shutting down"); //Prompt message //提示信息
}
