#include "uart_serial.h"
#include "utils.h"

bool SerialPort::openPort(const char* path_dev) {
    fd = open(path_dev, O_RDWR | O_NOCTTY | O_NDELAY);   //根据路径打开文件，并根据参数决定打开方式，
    //若成功则返回文件描述符，若失败则再次尝试打开
    if (fd != -1) {
        fcntl(fd, F_SETFL, 0);  //设置文件描述符状态标志，强制每次写(write)操作都添加在文件大的末尾
        fmt::print(fmt::fg(fmt::color::green), "[SerialPort] Open SerialPort success\n");
        need_init = false;
        return true;
    }
    else {
        //搜索串口号尾数0～4
        while (connect_cnt < 3)
            openPort("/dev/ttyUSB" + std::to_string(connect_cnt++));
        fmt::print(fmt::fg(fmt::color::red), "[SerialPort] Open SerialPort fail\n");
        return false;
    }
}


int SerialPort::configurePort() {
    struct termios port_settings;               //structure to store the port settings in

    tcgetattr(fd, &port_settings);
    tcflush(fd, TCIOFLUSH);                     //清空缓冲区的内容
    cfsetispeed(&port_settings, B460800);       //设置接受和发送的波特率
    cfsetospeed(&port_settings, B460800);
    tcsetattr(fd, TCSANOW, &port_settings);     //使设置立即生效
    tcflush(fd, TCIOFLUSH);

    tcgetattr(fd, &port_settings);
    port_settings.c_cflag |= (CLOCAL | CREAD);  //接受数据
    port_settings.c_cflag &= ~CSIZE;            //设置数据位数

    port_settings.c_cflag |= CS8;

    //设置奇偶校验位double
    port_settings.c_cflag &= ~PARENB;           //  Clear parity enable
    port_settings.c_iflag &= ~INPCK;            //  Enable parity checking

    //设置停止位
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_iflag |= INPCK;             //  Set input parity option

    tcflush(fd, TCIFLUSH);                      //清除输入缓存区
    port_settings.c_cc[VTIME] = 150;            //设置超时15 seconds
    port_settings.c_cc[VMIN] = 0;               //最小接收字符
    port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  //Input原始输入
    port_settings.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    port_settings.c_iflag &= ~(ICRNL | IGNCR);
    port_settings.c_oflag &= ~OPOST;            //Output禁用输出处理

    tcsetattr(fd, TCSANOW, &port_settings);
    return fd;
}

void SerialPort::restartPort() {
    close(fd);
    openPort(path_dev);
    configurePort();
}

bool SerialPort::sendData(VisionData& data) {

    uchar bytes_send[] = { 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE };

    short* bytes = reinterpret_cast <short*> (bytes_send + 1);
    bytes[0] = data.yaw * 100;
    bytes[1] = data.pitch * 100;
    bytes[2] = data.is_middle * 100;

    if (8 == write(fd, bytes_send, 8))
        return true;
    return false;
}

float SerialPort::exchangeData(uchar *data) {
    float float_data;
    float_data = *((float*)data);
    return float_data;
};

bool SerialPort::getQuat(TaskData& task_data, uchar *data) {

    uchar* f1 = &data[0];
    uchar* f2 = &data[4];
    uchar* f3 = &data[8];
    uchar* f4 = &data[12];

    quat[0] = exchangeData(f1);
    quat[1] = exchangeData(f2);
    quat[2] = exchangeData(f3);
    quat[3] = exchangeData(f4);

#ifdef SHOW_MCU_DATA
    fmt::print(fmt::fg(fmt::color::white), "quat: {} {} {} {} \n\n", quat[0], quat[1], quat[2], quat[3]);
#endif

    task_data.quat = {quat[0], quat[1], quat[2], quat[3]};
    return true;
}

bool SerialPort::quatToEuler(TaskData& task_data) {

    euler[0] = Tool::radian2Angle(atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                                         2.0f * (quat[0] * quat[0] + quat[1] * quat[1]) - 1.0f));
    euler[1] = Tool::radian2Angle(atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                                         2.0f * (quat[0] * quat[0] + quat[3] * quat[3]) - 1.0f));
    euler[2] = Tool::radian2Angle((asinf(-2.0f * (quat[1] * quat[3] - quat[0] * quat[2]))));

    task_data.euler = {euler[0], euler[1], 0};

    return true;
}

bool SerialPort::getEuler(TaskData& task_data, uchar *data) {

    uchar* f1 = &data[0];
    uchar* f2 = &data[4];

    euler[0] = exchangeData(f1);
    euler[1] = exchangeData(f2);

    task_data.euler = {euler[0], euler[1], 0};

    return true;
}

bool SerialPort::receiveData(TaskData& task_data) {

    Data& data = Data::getData();
    uchar head = static_cast<uchar>(0xFF);
    uchar end  = static_cast<uchar>(0xFE);

    int read_status = 0;

    int result = ioctl(fd, FIONREAD, &read_status);
    if (result == -1)
        return false;

    read_status = read(fd, &read_buffer, 21);

    if(read_status == -1 || read_status == 0) {
        //重启串口
        need_init = true;
        fmt::print(fmt::fg(fmt::color::yellow), "[SerialPort]  Serial offline, trying to reconnect...\n");
        restartPort();
        return false;
    }
    if(read_buffer[0] == head && read_buffer[20] == end) {
        data.mcu.color_enemy = static_cast<int>(read_buffer[1]);
        data.mcu.mode_detect = static_cast<int>(read_buffer[2]);
        data.mcu.speed_bullet = static_cast<int>(read_buffer[3]);

#ifdef SHOW_MCU_DATA
        std::cout << data.mcu;
#endif
        getQuat(task_data, &read_buffer[4]);
        quatToEuler(task_data);
        return true;
    }
    return false;
}
