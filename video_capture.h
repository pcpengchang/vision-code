#ifndef CAPTURE_VIDEO_H_INCLUDE
#define CAPTURE_VIDEO_H_INCLUDE

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <CameraApi.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <future>
#include "common.h"
#include "Galaxy/DxImageProc.h"
#include "Galaxy/GxIAPI.h"
#include "MindVision/CameraApi.h"
#include "MindVision/CameraDefine.h"
#include "MindVision/CameraStatus.h"

/**
 * @brief 相机驱动类型
 *@CAPTURE_ +:
 *	@CV	     0，普通CV
 *	@GALAXY  1，大恒139
 *  @MV      2，迈德威视
 */
enum CaptureDriver {
    CV     = 0,
    GALAXY = 1,
    MV     = 2
};

/**
 * @brief Galaxy相机驱动类
 * 实现对工业摄像头的基本参数设定和初始化，并将当前帧的数据转化为opencv的Mat类型
 */
class GalaxyCamera {
public:
    GalaxyCamera() = default;
    ~GalaxyCamera();

    /**
    * @brief 设置相机参数
    * @return int
    */
    int open();

    /**
     * @brief 设置曝光
     * @param exposure_time 具体曝光值
     */
    void setExposureTime(int exposure_time);

    /**
     * @brief 设置增益
     * @param value 选择通道 0-B,1-G,2-R,3-All
     * @param gain 具体增益值 范围0-16
     */
    void setGain(int value, int gain);

    /**
     * @brief 设置自动白平衡
     * @param value 0是表示关闭，1为开启
     */
    void setWhiteBalanceAuto(int value);

    /**
     * @brief 设置手动白平衡, 设置之前必须先关闭自动白平衡
     * @param value 0是表示关闭，1为开启
     */
    void setWhiteBalance(int value, float value_number);

    /**
     *@brief: 获取图像给外部
     *@param: img 图像
     */
    GalaxyCamera& operator>>(cv::Mat& img);

private:
    GX_STATUS status;
    GX_DEV_HANDLE hDevice;
    GX_OPEN_PARAM stOpenParam;
    uint32_t nDeviceNum;
    GX_FRAME_DATA stFrameData;
    cv::Mat src;
    void* m_rgb_image;
};

/**
 * @brief MindVision相机驱动类
 * 实现对工业摄像头的基本参数设定和初始化，并将当前帧的数据转化为opencv的Mat类型
 */
class MVCamera {
public:
    MVCamera() = default;
    ~MVCamera();

    /**
    * @brief 设置相机参数
    * @return int
    */
    int open();

    /**
     * @brief 设置曝光
     * @param exposure_time 具体曝光值
     */
    void setExposureTime(int exposure_time);

    /**
     * @brief 设置增益
     * @param gain RGB通道增益
     */
    void setGain(int gain_r, int gain_g, int gain_b);

    /**
     *@brief: 获取图像给外部
     *@param: img 图像
     */
    MVCamera& operator>>(cv::Mat& img);

private:
    int  iCameraCounts  = 1;
    int  iStatus        = -1;
    int  hCamera;

    tSdkCameraDevInfo   tCameraEnumList;
    tSdkCameraCapbility tCapability;
    tSdkFrameHead       sFrameInfo;
    tSdkImageResolution pImageResolution;
    BYTE*               pbyBuffer;
    cv::Mat src;
    cv::VideoCapture cap_;
};

class Capture {
public:
    Capture(int type_driver, const std::string& path_video);

    ~Capture() {
        switch (type_driver) {
        case CV:
            delete cap.cap_cv;
        case GALAXY:
            delete cap.cap_galaxy;
        case MV:
            delete cap.cap_mv;
        }
    }

    /**
     * @brief 打开相机
     * @param type_driver 驱动类型
     * @param path_video 视频路径
     */
    void open(int type_driver, const std::string& path_video);

    Capture& operator>>(cv::Mat& img);

private:
    bool armor_ready;   // 自瞄相机参数切换标志位
    bool buff_ready;    // 大小符相机参数切换标志位
    int type_driver;	// 驱动类型

    union _Capture {
        cv::VideoCapture* cap_cv;
        GalaxyCamera* cap_galaxy;
        MVCamera* cap_mv;
        _Capture(): cap_cv(nullptr) { }
        ~_Capture() = default;
    } cap;
};

/**
 * @brief 相机异常类
 * @details 相机在比赛中可能出现连接不稳定等异常，写一异常类帮助相机重新连接；同时防止因为相机异常造成的程序崩溃
 */
class CameraException : public std::exception {
public:
    CameraException() = default;

    /**
     * @brief 自定义构造函数, 需要给出异常信息
     * @param error 异常描述信息
     */
    CameraException(const std::string &error) : e_what(error) {}

    ~CameraException() throw() {}

    /**
     * @brief 异常规格说明：不抛出异常
     * @return 异常信息字符串
     * @note 该函数为std::exception类中的覆盖
     */
    virtual const char *what() const throw() {
        return e_what.c_str();
    }
private:
    // / 异常信息字符串
    std::string e_what;
};


/**
 * @brief 视频录制
 */
class SaveVideo {
public:
    SaveVideo() = default;
    ~SaveVideo();

    /**
     * @brief 比赛视频录制
     */
    void saveVideo(const cv::Mat& img);

private:
    cv::VideoWriter writer;
    std::future<void> write_video;

    int frame_cnt = 0;
    bool flag_save = true;
    bool is_first_loop = true;
};

#endif // CAPTURE_VIDEO_H_INCLUDE
