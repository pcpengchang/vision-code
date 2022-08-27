#ifndef COMMON_H_INCLUDE
#define COMMON_H_INCLUDE

#include "utils.h"

/**
 * @brief 敌方颜色
 *@Enemy_ +:
 *  @RED  0，红色
 *  @BLUE 1，蓝色
 */
enum EnemyColor {
    RED  = 0,
    BLUE = 1
};

/**
 * @brief 装甲板类型
 *@ARMOR_ +:
 *  @SMALL 0，小装甲板
 *  @BIG   1，大装甲板
 *  @BUFF  2，能量机关装甲板
 */
enum ArmorType {
    SMALL = 0,
    BIG   = 1,
    BUFF  = 2
};

/**
 * @brief 自瞄模式
 *@DETECT_ +:
 *	@ARMOR       0, 装甲板自瞄
 *	@SMALL_BUFF  1, 小能量机关
 *  @BIG_BUFF    2, 大能量机关
 */
enum DetectMode {
    ARMOR      = 0,
    SMALL_BUFF = 1,
    BIG_BUFF   = 2
};

/**
 * @brief 跟踪器模式
 *@Tracker_ +:
 *	@LOST       0, 目标丢失
 *	@DETECTING  1, 目标识别中
 *  @TRACKING   2, 目标跟踪中
 *  @TEMP_LOST  3, 目标短暂丢失
 */
enum TrackerState {
    LOST      = 0,
    DETECTING = 1,
    TRACKING  = 2,
    TEMP_LOST = 3
};

/**
 * @brief 线程交互数据包，将图像、陀螺仪和时间戳对应
 * @details pitch零参考为水平面， yaw零参考为陀螺仪的零点
 */
struct TaskData {
    cv::Mat img;                //  640 X 480 原图
    Eigen::Quaterniond quat;    //  四元数姿态
    std::array<float, 3> euler; //  欧拉角姿态[单位:degree]
    int timestamp;              //  时间戳 单位：ms

    /**
     * @brief 重载流输出运算符
     * @param out 标准输出流
     * @param task_data TaskData 结构体
     * @return 标准输出流
     */
    friend std::ostream &operator<<(std::ostream &out,
                                    const TaskData &task_data) {
        out << "timestamp: " << task_data.timestamp << std::endl;
        return out;
    }
} __attribute__((packed));

/**
 * @brief 发送数据包
 * @details 发送与电控通讯间的相应所需数据
 */
struct VisionData {
    float pitch;       //  俯仰角
    float yaw;         //  偏航角
    float dist;        //  目标距离
    int is_middle;     //  1表示目标进入了可以开火的范围，0则表示目标尚未进入可开火的范围

    /**
     * @brief 重载流输出运算符
     * @param out 标准输出流
     * @param vision_data VisionData 结构体
     * @return 标准输出流
     */
    friend std::ostream &operator<<(std::ostream &out,
                                    const VisionData &vision_data) {
        out << "Yaw: " << vision_data.yaw << std::endl
            << "Pitch: " << vision_data.pitch << std::endl
            << "Dist: " << vision_data.dist << std::endl
            << "\n------------------------------------\n";
        return out;
    }
} __attribute__((packed));

/**
 * @brief 程序各部分的中间交互数据，可读可写
 */
struct Data {
public:
    /**
     * @brief 单例对象
     */
    static Data& getData() {
        static Data data;
        return data;
    }
private:
    Data() = default;
    ~Data() = default;

public:
    int cnt_lost;               // 目标丢失计数
    double data_1;              // 波形图数据 1
    double data_2;              // 波形图数据 2
    cv::Mat img_show;           // 640 X 480 原图
    TrackerState tracker_state; // 跟踪器

    float yaw;                  // 偏航角
    Eigen::Quaterniond quat;    // 四元数姿态
    std::array<float, 3> euler; // 欧拉角姿态[单位:degree]

    /**
     * @brief 装甲板数据包
     */
    struct ArmorData {
        int type_armor = ArmorType::SMALL;   // 装甲板大小类型
        std::vector<cv::Point2f> pts_2d;	 // 目标装甲板四个角点
        bool is_anti_spin;                   // 是否是反陀螺模式
    } armor;

    /**
     * @brief 能量机关数据包
     */
    struct BuffData {
        float angle;    // 目标扇叶角度
    } buff;

    /**
     * @brief 接收数据包
     * @details 接收与电控通讯间的相应所需数据
     */
    struct McuData {
        int color_enemy = EnemyColor::BLUE;	 // 敌方颜色，从串口接收或手动设置
        int mode_detect = DetectMode::ARMOR; // 目标类型，从串口接收或手动设置
        int speed_bullet = 20;               // 子弹速度(m/s)

        friend std::ostream &operator<<(std::ostream &out,
                                        const McuData &mcu_data) {
            out << "color_enemy: " << mcu_data.color_enemy << std::endl
                << "mode_detect: " << mcu_data.mode_detect << std::endl
                << "speed_bullet: " << mcu_data.speed_bullet << std::endl;
            return out;
        }
    } mcu;

} __attribute__((packed));


/**
 * @brief 参数类，由控制线程输入，可读可写
 */
struct Parameter {
public:
    /**
     * @brief 单例对象
     */
    static Parameter& getParameter() {
        static Parameter param;
        return param;
    }
private:
    Parameter() = default;
    ~Parameter() = default;

public:

    //  装甲板识别参数
    struct ArmorParameter {
        int th_gray   = 25;      //灰度二值化阈值
        int th_purple = 16;	     //灰度二值化阈值-紫色
        int th_green = 16;	     //色彩分离二值化阈值-绿色

        int th_color_red = 16;	 //色彩分离二值化阈值-红色
        int th_gray_red  = 25;   //灰度二值化阈值-红色

        int th_color_blue = 16;	 //色彩分离二值化阈值-蓝色
        int th_gray_blue  = 25;  //灰度二值化阈值-蓝色
    } armor;

    //  能量机关识别参数
    struct BuffParameter {
        int th_color_red = 15;	 //色彩分离二值化阈值-红色
        int th_gray_red  = 78;   //灰度二值化阈值-红色

        int th_color_blue = 15;	 //色彩分离二值化阈值-蓝色
        int th_gray_blue  = 78;  //灰度二值化阈值-蓝色
    } buff;

} __attribute__((packed));

/**
 * @brief 设置类，程序初始的配置，所有线程只读
 */
class Setting {
public:
    Setting() = default;
    ~Setting() = default;

    /**
     * @brief 参数初始化 从配置文件中读入参数节点
     * @param path_config 配置路径
     */
    void setParameter(const std::string& path_config);

    /**
     * @brief 视频参数初始化
     * @param mode_detect 识别模式
     * @param target_color 目标颜色
     */
    void setVideoParameter(int mode_detect, int target_color);

public:

    std::string path_classifier_svm;	//  SVM
    std::string path_classifier_mlp;    //  MLP
    std::string path_classifier_lenet;  //  Lenet

    cv::Mat intrinsic;                  //  内参矩阵
    cv::Mat coeff;                      //  畸变矩阵

    cv::Mat t_cam_ptz;	                //  相机到云台的变换矩阵
    cv::Mat t_imu_cam;                  //  陀螺仪到相机的变换矩阵
    cv::Mat t_cam_imu;                  //  相机到陀螺仪的变换矩阵

    //  能量机关打击模型参数
    int h_buff;			                //  R标 中心高度
    int r_buff;		                    //  半径
    int dist_buff;		                //  与己方激活点距离
    int h_platform;                     //  己方激活点高度
    int h_car;				            //  己方枪管离麦轮接触面高度
};


#endif	//COMMON_H_INCLUDE
