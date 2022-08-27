#ifndef SERIAL_H_INCLUDE
#define SERIAL_H_INCLUDE

#include <atomic>
#include <fcntl.h>   /*文件控制定义*/
#include <unistd.h>  /*Unix 标准函数定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <cstring>
#include <cerrno>    /*错误号定义*/
#include <sys/ioctl.h>
#include "common.h"

/**
 * @param 串口类
 * @details Manifold2G: /dev/ttyS0(对应uart0接口) /dev/ttyTHS2(对应uart1接口)
 *          Manifold2C: /dev/ttyS0(对应uart0接口)
 *          NUC:        /dev/ttyUSB0 或 /dev/ttyUSB1
 */
class SerialPort {
public:
    bool need_init = true;  //是否连接成功

    /**
     * @brief 打开串口
     */
    SerialPort() {
        openPort("/dev/ttyUSB0");
        configurePort();
    }

    /**
     * @brief 打开串口
     * @param path_device 串口路径
     */
    SerialPort(const char* path_device):
        path_dev(path_device) {
        openPort(path_device);
        configurePort();
    }
    SerialPort(std::string path_device):
        SerialPort(path_device.c_str()) {
    }
    ~SerialPort() = default;

    /**
     * @brief 打开串口，产生fd
     * @param  path_dev 串口路径
     * @return 打开成功or失败
     */
    bool openPort(const char* path_dev);
    bool openPort(std::string path_dev) {
        return openPort(path_dev.c_str());
    }

    /**
     * @brief 配置串口
     */
    int configurePort();

    /**
     * @brief 重启串口
     */
    void restartPort();

    /**
     * @brief 发送数据
     * @param vision_data 要发送的数据
     * @return 发送成功/失败
     */
    bool sendData(VisionData &vision_data);

    /**
     * @brief 获取数据
     * @param task_data 数据包
     */
    bool receiveData(TaskData& task_data);

    /**
     * @brief 将 4 个 uchar 合并转换为 float
     * @param data 首地址指针
     * @return float 数
     */
    float exchangeData(uchar *data);

    /**
     * @brief 解算四元数数据
     * @param task_data 数据包
     * @param data 四元数首地址指针
     */
    bool getQuat(TaskData& task_data, uchar *data);

    /**
     * @brief 四元数转欧拉角
     * @param task_data 数据包
     */
    bool quatToEuler(TaskData& task_data);

    /**
     * @brief 解算云台欧拉角
     * @param task_data 数据包
     * @param data 云台欧拉角首地址指针
     */
    bool getEuler(TaskData& task_data, uchar *data);

private:
    std::array<float, 4> quat;          //  四元数姿态
    std::array<float, 3> euler;         //  欧拉角姿态[单位:degree]
    std::array<uchar, 21> read_buffer;
    int fd;					            //  串口描述配置文件
    std::string path_dev;               //  串口路径
    int connect_cnt = 0;                //  重连次数
};

#endif //SERIAL_H_INCLUDE
