///*******************************************************************************************///
///               ##                     ########   #####           #######################   ///
///              ####                  ##########   #######         #######################   ///
///             ######               ########         ######        #######################   ///
///              ######             #######             #####                                 ///
///           #   ######            ######                                                    ///
///          ###   ######           #####                           ####  #################   ///
///         #####   ######          #####                           ####  #################   ///
///        ######    ######         #####                           ####  #################   ///
///       ######      ######        ######                                                    ///
///      ######   ###########       #######             #####                                 ///
///     ######   #############       ########         ######        #######################   ///
///    ######            ######         #########   #######         #######################   ///
///   ######               #####          #######   #####           #######################   ///
///*******************************************************************************************///
///
///*******************************************************************************************///
///                              ____  __  __        ______     __                            ///
///                             |  _ \|  \/  |      / ___\ \   / /                            ///
///                             | |_) | |\/| |_____| |    \ \ / /                             ///
///                             |  _ <| |  | |_____| |___  \ V /                              ///
///                             |_| \_\_|  |_|      \____|  \_/                               ///
///                                                                                           ///
///*******************************************************************************************///
/////** @copyright Copyright (c) 2022 DGUT_ACE **/////

#include "thread_task.h"

void control() {

    cv::Mat img_trackbar = cv::Mat::zeros(1, 300, CV_8UC1);  // 预处理
    Parameter& param = Parameter::getParameter();
    cv::namedWindow("Parameter");
    cv::moveWindow("Parameter", 1500, 10);
    cv::createTrackbar("armor_gray_th",   "Parameter", &param.armor.th_gray_red,   255, nullptr);
    cv::createTrackbar("armor_color_th",  "Parameter", &param.armor.th_color_red,  255, nullptr);
    cv::createTrackbar("armor_gray_th",   "Parameter", &param.armor.th_gray_blue,  255, nullptr);
    cv::createTrackbar("armor_color_th",  "Parameter", &param.armor.th_color_blue, 255, nullptr);

    cv::createTrackbar("armor_purple_th", "Parameter", &param.armor.th_purple,     255, nullptr);
    cv::createTrackbar("armor_green_th",  "Parameter", &param.armor.th_green,      255, nullptr);

    cv::createTrackbar("buff_gray_th",    "Parameter", &param.buff.th_gray_red,    255, nullptr);
    cv::createTrackbar("buff_color_th",   "Parameter", &param.buff.th_color_red,   255, nullptr);
    cv::createTrackbar("buff_gray_th",    "Parameter", &param.buff.th_gray_blue,   255, nullptr);
    cv::createTrackbar("buff_color_th",   "Parameter", &param.buff.th_color_blue,  255, nullptr);

    cv::imshow("Parameter", img_trackbar);
}

int main() {

    XInitThreads();
    
    control();
    std::cout << "\x1B[2J\x1B[H"; // 清空终端区显示

    system("echo 111111 | sudo -S chmod 777 /dev/ttyUSB*");// 赋予串口权限

    auto task = ObjManager<ThreadTask>::bind("ThreadTask");// 开启相关线程

    //LOG::setDestination("../log/" + std::to_string(FileOperation::getFileSizeInOrder("../log/", ".log")) + ".log");
    //LOG::info("Connected to !");

    auto time_start = std::chrono::steady_clock::now();
    MsgFilter<TaskData> task_factory(5);
    MsgFilter<VisionData> data_transmiter(10);
    MsgFilter<TaskData> data_receiver(100);

    std::thread task_producer(&ThreadTask::Producer, task, std::ref(task_factory), std::ref(data_receiver), time_start);
    std::thread task_consumer(&ThreadTask::Consumer, task, std::ref(task_factory), std::ref(data_transmiter));
    std::thread receiver(&ThreadTask::dataReceiver, task, std::ref(data_receiver), time_start);
    std::thread transmitter(&ThreadTask::dataTransmitter, task, std::ref(data_transmiter));

    task_producer.join();
    task_consumer.join();
    transmitter.join();
    receiver.join();

    return 0;
}
