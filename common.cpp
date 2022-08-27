#include "common.h"

void Setting::setParameter(const std::string& path_config) {

    cv::FileStorage fs;

    try {
        fs.open(path_config, cv::FileStorage::READ);

        fs["h_buff"] >> h_buff;
        fs["r_buff"] >> r_buff;
        fs["dist_buff"] >> dist_buff;
        fs["h_car"] >> h_car;
        fs["h_platform"] >> h_platform;

        fs["T_cp"] >> t_cam_ptz;
        fs["T_ic"] >> t_imu_cam;
        fs["T_ci"] >> t_cam_imu;
        fs["Intrinsic"] >> intrinsic;
        fs["Coeff"] >> coeff;

        if (intrinsic.cols != 3 || intrinsic.rows != 3) {
            throw std::runtime_error("Parameter 'intrinsic' invalid format!");
        }
        if (coeff.cols != 5 || coeff.rows != 1) {
            throw std::runtime_error("Parameter 'coeff' invalid format!");
        }

        fs.release();
    } catch (cv::Exception &e) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: Parameter read fail!");
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: {}", e.what());
    }

    path_classifier_svm   = "../configs/other/svm2.xml";
    path_classifier_mlp   = "../configs/other/fc.onnx";
    path_classifier_lenet = "../configs/other/Lenet5_v4.onnx";

    fmt::print(fmt::fg(fmt::color::green), "[setting] Set param finished\n");
}

void Setting::setVideoParameter(int mode_detect, int target_color) {

    Data& data = Data::getData();

    data.mcu.mode_detect = mode_detect;
    data.mcu.color_enemy = target_color;

}
