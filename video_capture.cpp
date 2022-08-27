#include "video_capture.h"

#define ASSERT_WARNING(expr, info, ...) do{                      \
    if((expr) == false){                                         \
        fmt::print(fg(fmt::color::orange),                       \
                   "[WARNING] " #expr info "\n", ##__VA_ARGS__); \
    }                                                            \
}while(0)

#define ASSERT_THROW(expr, info, ...) do{                       \
    if((expr) == false){                                        \
        fmt::print(fg(fmt::color::orange),                      \
                       "[WARNING] " #info "\n", ##__VA_ARGS__); \
        throw CameraException();                                \
    }                                                           \
}while(0)

// --------------------galaxy-----------------------------

GalaxyCamera::~GalaxyCamera() {
    status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);

    //释放图像缓冲区buffer
    free(stFrameData.pImgBuf);

    //关闭设备链接
    status = GXCloseDevice(hDevice);

    //释放库
    status = GXCloseLib();
}

int GalaxyCamera::open() {

    // 初始化库
    status = GXInitLib();
    ASSERT_THROW(status == GX_STATUS_SUCCESS, "InitLib Error!");

    // 枚举设备列表
    nDeviceNum = 0;
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    ASSERT_THROW(status == GX_STATUS_SUCCESS && nDeviceNum > 0, "Update device list Error!");

    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE; // 独占方式打开设备
    stOpenParam.openMode = GX_OPEN_INDEX;  // 通过设备序号(从 1 开始,1、2、3、4...)打开设备
    stOpenParam.pszContent = "1";   // 设备序号

    // 打开设备
    hDevice = NULL;
    status = GXOpenDevice(&stOpenParam, &hDevice);
    ASSERT_THROW(status == GX_STATUS_SUCCESS, "Open Device failed!");

    // 清空缓冲队列
    int64_t nPayLoadSize = 0;

    // 申请内存
    status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
    ASSERT_THROW(status == GX_STATUS_SUCCESS && nPayLoadSize > 0, "Get payload size failed!");

    // 申请帧存
    stFrameData.pImgBuf = malloc(static_cast<size_t>(nPayLoadSize));
    ASSERT_THROW(stFrameData.pImgBuf != NULL, "Failed to allocate memory for image buffer");

    // 申请处理后图片内存
    m_rgb_image = malloc(nPayLoadSize * 3);
    ASSERT_THROW(m_rgb_image != NULL, "Failed to allocate memory for rgb frame data");

    // 设置采集模式连续采集
    status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    status = GXSetInt(hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 1);
    ASSERT_THROW(status == GX_STATUS_SUCCESS, "Set Enum Acq mode failed!");

    // 设置曝光值
    setExposureTime(1500);

    // 设置连续自动白平衡
    setWhiteBalanceAuto(1);

    // 设置相机分辨率
    int64_t nWidth   = 640;
    int64_t nHeight  = 480;
    int64_t nOffsetX = 320;
    int64_t nOffsetY = 272;
    status = GXSetInt(hDevice, GX_INT_WIDTH, nWidth);
    status = GXSetInt(hDevice, GX_INT_HEIGHT, nHeight);
    status = GXSetInt(hDevice, GX_INT_OFFSET_X, nOffsetX);
    status = GXSetInt(hDevice, GX_INT_OFFSET_Y, nOffsetY);

    // 发送开始采集命令
    status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);

    ASSERT_THROW(status == GX_STATUS_SUCCESS, "Init camera failed.");

    return status;
}

void GalaxyCamera::setExposureTime(int exposure_time) {
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_time);
    ASSERT_THROW(status == GX_STATUS_SUCCESS, "Set exposure_time failed!");
}

void GalaxyCamera::setGain(int value, int gain) {
    switch (value) {
    case 0:
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
        break;
    case 1:
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
        break;
    case 2:
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
        break;
    default:
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        break;
    }
    status = GXSetFloat(hDevice, GX_FLOAT_GAIN, gain);
    ASSERT_THROW(status == GX_STATUS_SUCCESS, "Set gain failed!");
}

void GalaxyCamera::setWhiteBalanceAuto(int value) {
    status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, value);
    ASSERT_THROW(status == GX_STATUS_SUCCESS, "Set white_balance failed!");
}

void GalaxyCamera::setWhiteBalance(int value, float value_number) {
    //选择白平衡通道
    switch (value) {
    case 0:
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
        break;
    case 1:
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
        break;
    case 2:
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
        break;
    }
    status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, static_cast<float>(value_number));
    ASSERT_THROW(status == GX_STATUS_SUCCESS, "Set white_balance failed!");
}

GalaxyCamera& GalaxyCamera::operator>>(cv::Mat& img) {
    GXGetImage(hDevice, &stFrameData, 100);
    ASSERT_WARNING(stFrameData.nStatus == GX_FRAME_STATUS_SUCCESS, "Get image error!");

    if (stFrameData.nStatus == GX_FRAME_STATUS_SUCCESS) {
        // 图像转码
        DxRaw8toRGB24(stFrameData.pImgBuf, m_rgb_image, stFrameData.nWidth,
                      stFrameData.nHeight, RAW2RGB_NEIGHBOUR3, DX_PIXEL_COLOR_FILTER(BAYERBG), false);

        src = cv::Mat(stFrameData.nHeight, stFrameData.nWidth, CV_8UC3, m_rgb_image);
        src.copyTo(img);
    }
    return *this;
}



// --------------------MindVision-----------------------------
MVCamera::~MVCamera() {
    CameraUnInit(hCamera);
    hCamera = -1;
}

MVCamera& MVCamera::operator>>(cv::Mat& img) {

    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 100) == CAMERA_STATUS_SUCCESS) {
        src = cv::Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3);
        CameraImageProcess(hCamera, pbyBuffer, src.data, &sFrameInfo);
    }
    CameraReleaseImageBuffer(hCamera, pbyBuffer);

    src.copyTo(img);
    return *this;
}

int MVCamera::open() {

    // 初始化SDK
    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    ASSERT_THROW(iStatus == CAMERA_STATUS_SUCCESS && iCameraCounts > 0, "no device found!");

    // 相机初始化.初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    ASSERT_THROW(iStatus == CAMERA_STATUS_SUCCESS && hCamera >= 0, "no camera found!");

    // 设置相机分辨率
    CameraGetImageResolution(hCamera, &pImageResolution);

    pImageResolution.iIndex      = 0xFF;
    pImageResolution.iWidthFOV   = 640;
    pImageResolution.iHeightFOV  = 480;
    pImageResolution.iWidth      = 640;
    pImageResolution.iHeight     = 480;
    pImageResolution.iHOffsetFOV = static_cast<int>((1280 - 640) * 0.5);
    pImageResolution.iVOffsetFOV = static_cast<int>((1024 - 480) * 0.5);

    CameraSetImageResolution(hCamera, &pImageResolution);

    // 设置曝光时间
    setExposureTime(600);

    // 设置帧率 [0-2],默认0
    //CameraSetFrameSpeed(hCamera, 2);

    // 设置RGB通道增益
    setGain(145, 130, 105);

    // 设置图像降噪使能 or not
    //CameraSetNoiseFilter(hCamera, 1);
    // 设置图像锐化程度[0-100],默认0
    //CameraSetSharpness(hCamera, 0);

    // 设置对比度, 默认100
    //CameraSetContrast(hCamera, 100);
    // 设置饱和度, 默认100
    //CameraSetSaturation(hCamera, 100);

    // 设置图像处理的输出格式，彩色黑白都支持RGB24位
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);

    // 让SDK进入工作模式，开始接收来自相机发送的图像数据。如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像
    CameraPlay(hCamera);
    CameraReleaseImageBuffer(hCamera, pbyBuffer);

    return iStatus;
}

void MVCamera::setExposureTime(int exposure_time) {
    iStatus = CameraSetAeState(hCamera, FALSE);
    iStatus = CameraSetExposureTime(hCamera, exposure_time);
    ASSERT_THROW(iStatus == CAMERA_STATUS_SUCCESS, "Set exposure_time failed!");
}

void MVCamera::setGain(int gain_r, int gain_g, int gain_b) {
    iStatus = CameraSetGain(hCamera, gain_r, gain_g, gain_b);
    ASSERT_THROW(iStatus == CAMERA_STATUS_SUCCESS, "Set gain failed!");
}

// ---------------------------------------------------------------

Capture::Capture(int type_driver, const std::string& path_video):
    type_driver(type_driver) {

    armor_ready = false;
    buff_ready = false;

    try {
        open(type_driver, path_video);
    } catch (CameraException &e1) {
        fmt::print(fmt::fg(fmt::color::red), "[CAMERA] Open Camera failed\n");
        for (int i = 0; i < 5; ++i) {
            try {
                open(type_driver, path_video);
            }
            catch (CameraException &e2) {
                fmt::print(fmt::fg(fmt::color::red), "[CAMERA] Open Camera failed\n");
                sleep(1);
            }
        }
    }
}

Capture& Capture::operator>>(cv::Mat& img) {
    Data& data = Data::getData();

    // 根据自瞄模式, 设置相机白平衡，曝光等参数
    switch (type_driver) {
    case CV:
        *(cap.cap_cv) >> img;
        //Tool::extractVideo(img);
        //cv::resize(img, img, {720, 540});
        break;
    case GALAXY:
        if (data.mcu.mode_detect == DetectMode::ARMOR && !armor_ready) {
            cap.cap_galaxy -> setExposureTime(1500);
            armor_ready = true;
            buff_ready = false;
        }
        else if(data.mcu.mode_detect != DetectMode::ARMOR && !buff_ready) {
            cap.cap_galaxy -> setExposureTime(3000);
            armor_ready = false;
            buff_ready = true;
        }
        *(cap.cap_galaxy) >> img;
        break;
    case MV:
        if (data.mcu.mode_detect == DetectMode::ARMOR && !armor_ready) {
            cap.cap_mv -> setExposureTime(600);
            armor_ready = true;
            buff_ready = false;
        }
        else if(data.mcu.mode_detect != DetectMode::ARMOR && !buff_ready) {
            cap.cap_mv -> setExposureTime(600);
            armor_ready = false;
            buff_ready = true;
        }
        *(cap.cap_mv) >> img;
        break;
    }
    return *this;
}

void Capture::open(int type_driver, const std::string &path_video) {

    switch (type_driver) {
    case CV:
        cap.cap_cv = new cv::VideoCapture(path_video);
        fmt::print(fmt::fg(fmt::color::green), "[CAMERA] Open USB Camera success\n");
        break;
    case GALAXY:
        cap.cap_galaxy = new GalaxyCamera();
        cap.cap_galaxy -> open();
        fmt::print(fmt::fg(fmt::color::green), "[CAMERA] Open GALAXY Camera success\n");
        break;
    case MV:
        cap.cap_mv = new MVCamera();
        cap.cap_mv -> open();
        fmt::print(fmt::fg(fmt::color::green), "[CAMERA] Open MV Camera success\n");
        break;
    }
}

void SaveVideo::saveVideo(const cv::Mat &img) {
    //强制关机会出现未更新文件头的情况，数据不会丢失
    if(flag_save) {
        char now[64];
        std::time_t tt = time(nullptr);
        struct tm *ttime = localtime(&tt);
        strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);  //  以时间为文件名
        std::string now_string(now);
        std::string path(std::string("../video/" + now_string).append(".avi"));

        //此处调节帧率,帧数越高读到的信息量越少
        writer.open(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20.0, cv::Size(img.cols, img.rows));
        if (!writer.isOpened()) {
            fmt::print(fmt::fg(fmt::color::red), "[RECORD] Can't open VideoWriter! \n");
            flag_save = true;
            return;
        }
        fmt::print(fmt::fg(fmt::color::green), "[RECORD] Created directory :{}\n", path);
        //system(std::string("sudo chmod 777 ").append(path).c_str());
    }
    flag_save = false;
    //writer << img;

    frame_cnt++;
    if(frame_cnt % 10 == 0) {
        frame_cnt = 0;
        //异步读写加速,避免阻塞生产者
        if (is_first_loop)
            is_first_loop = false;
        else
            write_video.wait();
        write_video = std::async(std::launch::async, [&]() {
            writer.write(img);
        });
    }
}

SaveVideo::~SaveVideo() {
    writer.release();
}
