#ifndef THREAD_TASK_H_INCLUDE
#define THREAD_TASK_H_INCLUDE

#include "video_capture.h"
#include "armor_detector.h"
#include "coord_solver.h"
#include "uart_serial.h"
#include "common.h"
#include "uart_serial.h"
#include "filter.h"
#include "buff_detector.h"
#include "data_plot.h"

/**
 *@brief: 帧率控制
 */
class RateControl {
public:
    RateControl() = default;
    ~RateControl() = default;
    RateControl(int r);
    double start_time;

    /**
     *@brief: 设定时间睡眠以达到帧率控制
     */
    void sleep();

private:
    Timer t;
    int loop_rate;
};

class ThreadTask {
public:

    /**
     * @brief 线程管理构造函数，用于线程中变量初始化
     */
    ThreadTask();
    ~ThreadTask() = default;

    /**
     * @brief 全图展示
     * @param fps 帧率
     */
    void showImage(int fps);

    /**
     * @brief 生产者线程
     * @param factory 数据包
     * @param receive_factory 数据包
     */
    void Producer(MsgFilter<TaskData> &factory, MsgFilter<TaskData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start);     //  生产者线程,获取图像

    /**
     * @brief 消费者线程
     * @param task_factory 数据包
     * @param transmit_factory 数据包
     */
    void Consumer(MsgFilter<TaskData> &task_factory, MsgFilter<VisionData> &transmit_factory);     //  消费者线程,处理图像线程

    /**
     * @brief 数据接收线程
     * @param receive_factory 数据包
     */
    void dataReceiver(MsgFilter<TaskData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start);   //  用于接收电控发来的数据

    /**
     * @brief 数据发送线程
     * @param transmit_factory 数据包
     */
    void dataTransmitter(MsgFilter<VisionData> &transmit_factory);

private:
    RateControl rate_control{100};  // 帧率控制
    Setting    setting;             // 设置类
    Timer      fps;                 // 帧率计算
    SaveVideo  video_save;          // 视频保存
};

#endif //THREAD_TASK_H_INCLUDE
