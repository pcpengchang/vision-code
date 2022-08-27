#include "thread_task.h"
using namespace std::chrono;

RateControl::RateControl(int r) {
    loop_rate = r;
    start_time = 0;
}

void RateControl::sleep() {
    double period = 1.f / loop_rate;
    double cur_time = t.now();
    double expect_end_time = start_time + period;
    if (expect_end_time > cur_time)
        usleep((expect_end_time - cur_time) * 1000000);
    start_time = t.now();
}

ThreadTask::ThreadTask() {
    setting.setParameter("../configs/car_param.xml");
    setting.setVideoParameter(DetectMode::ARMOR, EnemyColor::BLUE);

    fmt::print(fmt::fg(fmt::color::green), "[CAMERA] Set param finished\n");
}

void ThreadTask::Producer(MsgFilter<TaskData> &factory, MsgFilter<TaskData> &receive_factory, _V2::steady_clock::time_point time_start) {

    //std::string path_video = "/dev/video0";
    std::string path_video = "/home/pengchang/video/armor.avi";
    //std::string path_video = "/home/pengchang/video/2022.avi";
    //std::string path_video = "/home/pengchang/video/1.mp4";

    auto cap = ObjManager<Capture>::bind("Capture", CaptureDriver::CV, path_video);

    while(true) {
        TaskData task_data, task_quat;
        auto time_cap = steady_clock::now();

        *cap >> task_data.img;
        task_data.timestamp = (int)(duration<double, std::milli>(time_cap - time_start).count());
        cv::waitKey(7);

        if (task_data.img.empty())
            continue;

        // 订阅下位机数据
        if (!receive_factory.subscribe(task_quat, task_data.timestamp))
            continue;
        task_data.quat = task_quat.quat; //经过同步

        // 发布任务数据
        factory.publish(task_data);
    }
}

void ThreadTask::Consumer(MsgFilter<TaskData> &task_factory, MsgFilter<VisionData> &transmit_factory) {

    Data& data = Data::getData();

    DataPlot data_plot(600, 600);

    auto detector_armor = ObjManager<ArmorDetector>::bind("ArmorDetector", setting);
    auto detector_buff  = ObjManager<BuffDetector>::bind("BuffDetector");

    auto solver_armor   = ObjManager<ArmorAngleSolver>::bind("ArmorAngleSolver", setting);
    auto solver_buff    = ObjManager<BuffAngleSolver>::bind("BuffAngleSolver", setting);

    while (true) {

        fps.start();

        TaskData task_data;
        VisionData vision_data;

        // 订阅任务数据
        task_factory.subscribe(task_data);

        // 图像、姿态与时间戳对齐
        auto &[img, quat, euler, time_stamp] = task_data;
        data.quat = quat;
        data.img_show = img.clone();
        quat = {1, 0, 0, 0};

        // 自瞄模式状态机
        if (data.mcu.mode_detect == DetectMode::ARMOR) {
            //fmt::print(fmt::fg(fmt::color::pale_violet_red), "[CONSUMER] Mode switched to ARMOR\n");
            detector_armor -> detect(img, time_stamp);
            solver_armor -> getAngle(vision_data, time_stamp);
        }
        else {
            //fmt::print(fmt::fg(fmt::color::pale_violet_red), "[CONSUMER] Mode switched to DETECT_BUFF\n");
            detector_buff -> detect(img, time_stamp);
            solver_buff -> getAngle(vision_data);
        }

        // 角度限幅
        vision_data.yaw = Tool::limitValue(vision_data.yaw, 4);
        vision_data.pitch = Tool::limitValue(vision_data.pitch, 4);

        // 发布视觉数据
        transmit_factory.publish(vision_data);

        //rate_control.sleep();

        fps.getFPS();

#ifdef SAVE_VIDEO
        video_save.saveVideo(img);
#endif

#ifdef SHOW_VISION_DATA
        std::cout << vision_data;
#endif

#ifdef SHOW_DRAW_DATA
        std::vector<float> data_s = {static_cast<float>(data.data_1) * 1, static_cast<float>(data.data_2) * 1};
        std::vector<std::string> data_name = {"m_pw", "p_pw"};
        data_plot.dataCollectProc(data_s, data_name);
#endif

#ifdef SHOW_IMG
        showImage(fps.fps);
#endif
    }

}

void ThreadTask::dataReceiver(MsgFilter<TaskData> &receive_factory, _V2::steady_clock::time_point time_start) {

    auto serial = ObjManager <SerialPort>::bind("SerialPort");

    while (true) {

        TaskData task_data;

#ifdef USE_SERIAL
        //  若串口离线则跳过数据发送
        if (serial->need_init) {
            usleep(5000);
            continue;
        }
        //  如果没收到下位机的数据，则阻塞直到第一次收到数据
        while (!serial->receiveData(task_data));
#endif
        auto time_cap = steady_clock::now();
        auto timestamp = (int)(duration<double, std::milli>(time_cap - time_start).count());

        // 发布下位机数据
        receive_factory.publish(task_data, timestamp);
    }
}

void ThreadTask::dataTransmitter(MsgFilter<VisionData> &transmit_factory) {

    auto serial = ObjManager <SerialPort>::bind("SerialPort");

    while (true) {
        VisionData vision_data;

        // 订阅视觉数据
        transmit_factory.subscribe(vision_data);

#ifdef USE_SERIAL
        //  若串口离线则跳过数据发送
        if (serial->need_init) {
            usleep(5000);
            continue;
        }
#endif
        serial->sendData(vision_data);
    }

}

void ThreadTask::showImage(int fps) {
    Data& data = Data::getData();

    putText(data.img_show, fmt::format("FPS: {}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,255,0});
    line(data.img_show, cv::Point2f(data.img_show.size().width / 2, 0), cv::Point2f(data.img_show.size().width / 2, data.img_show.size().height), {0,255,0}, 1);
    line(data.img_show, cv::Point2f(0, data.img_show.size().height / 2), cv::Point2f(data.img_show.size().width, data.img_show.size().height / 2), {0,255,0}, 1);
    Tool::drawCross(data.img_show, cv::Point2f(data.img_show.size().width / 2, data.img_show.size().height / 2));
    cv::circle(data.img_show, cv::Point2f(data.img_show.size().width / 2, data.img_show.size().height / 2), 5, BGR::all(255), 2);

    //Tool::extractVideo(data.img_show);
    cv::imshow("dst", data.img_show);
    cv::waitKey(1);
}
