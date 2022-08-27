#ifndef SOLVE_ANGLE_H_INCLUDE
#define SOLVE_ANGLE_H_INCLUDE

#include "common.h"
#include "filter.h"
#include <opencv2/core/eigen.hpp>

/**
 * @brief 通用位姿解算, 解决空间绕轴旋转位姿问题
 * @details 相机坐标系
 *          云台坐标系（z轴与摄像机光轴重合，指向镜头外侧；y轴与z轴在同一竖直平面内并垂直z轴向下；
 *          x轴垂直yoz平面向右；该坐标系为右手坐标系）
 *          陀螺仪坐标系
 *          惯性坐标系
 */
class CoordSolver {
public:
    CoordSolver() = default;
    ~CoordSolver() = default;

    /**
     * @brief 带标定参数初始化
     * @param 标定参数
     */
    CoordSolver(Setting& setting);

    /**
     * @brief 绘制坐标系
     * @param rvec  旋转矩阵
     * @param tvec  平移矩阵
     */
    void drawCoordinate(cv::Mat& rvec, cv::Mat& tvec);

    /**
     * @brief PNP通用解算
     * @details 空间坐标系（右手系）转换为相机坐标系下
     * @details 暂时不需要使用旋转矩阵
     */
    Eigen::Vector3d PNP();

    /**
     * @brief 将世界坐标系内一点，投影到图像中
     * @details 相机坐标系内坐标--->图像坐标系内像素坐标
     * @param xyz 目标三维坐标
     */
    void reProjectPoint(const Eigen::Vector3d &xyz);

    /**
     * @brief 角度和距离复位清零
     * @param 视觉数据包
     */
    static void clearVisionData(VisionData& vision_data);

    /**
     * @brief 坐标系转换
     * @details 将云台坐标系绕x轴向下旋转大小等于原云台绝对pitch轴角的角度，相当于将坐标向上旋转相同的角度
     * @param x1 y2     空间点原始坐标
     * @param theta     空间点绕轴旋转theta度，角度制范围在-180到180
     */
    static void transAngle(double &x1, double &x2, double theta);

    /**
     * @brief 坐标系转换
     * @param x 三维坐标x
     * @param y 三维坐标y
     * @param z 三维坐标z
     * @param ptz_pitch 云台绝对pitch轴角度
     * @param ptz_yaw 云台绝对yaw轴角度
     */
    static void transAngle(double &x, double &y, double &z,
                           double ptz_pitch, double ptz_yaw);

    /**
     * @brief 将相机坐标系转化为惯性坐标系
     * @param point_camera 相机坐标系下坐标
     * @param rmat 由陀螺仪四元数解算出的旋转矩阵
     * @return 惯性坐标系下坐标
     * **/
    Eigen::Vector3d camToWorld(const Eigen::Vector3d &point_camera, const Eigen::Matrix3d &rmat);

    /**
     * @brief 将惯性坐标系转化为相机坐标系
     * @param point_world 惯性坐标系下坐标
     * @param rmat 由陀螺仪四元数解算出的旋转矩阵
     * @return 相机坐标系下坐标
     * **/
    Eigen::Vector3d worldToCam(const Eigen::Vector3d &point_world, const Eigen::Matrix3d &rmat);

    /**
     * @brief 计算目标Pitch 子弹下坠重力补偿
     * @param xyz 目标坐标
     * @return 重力补偿
     */
    double getPitchOffset(Eigen::Vector3d &xyz, double depth);

    /**
     * @brief 剔除异常数据
     * @param point_world 原始数据
     */
    Eigen::Vector3d wrongDataKiller(Eigen::Vector3d &point_world);

public:
    int bad_cnt = 0;                  // 异常数据计数
    Eigen::Vector3d last_pw;          // 上一次的目标惯性坐标
    cv::Mat intrinsic;				  // 相机内参矩阵 CV-Mat
    Eigen::Matrix3d eigen_intrinsic;  // 相机内参矩阵 EIGEN-Matrix
    cv::Mat dis_coeff;		          // 相机畸变矩阵 CV-Mat

    Eigen::Vector3d t_cam_ptz;        // 相机坐标系转云台坐标系 平移矩阵
    Eigen::Matrix4d t_imu_cam;        // 陀螺仪坐标系到相机坐标系的平移矩阵
    Eigen::Matrix4d t_cam_imu;        // 相机坐标系到陀螺仪坐标系的平移矩阵
};

/**
 * @brief 装甲板角度解算
 * @details 包含普通卡尔曼、扩展卡尔曼和曲线拟合三种预测方式, 默认使用扩展卡尔曼
 */
class ArmorAngleSolver {
public:
    ArmorAngleSolver() = default;

    /**
     * @brief 带标定参数初始化
     * @param 标定参数
     */
    ArmorAngleSolver(Setting& setting);

    ~ArmorAngleSolver() = default;

    /**
     * @brief 计算目标Yaw Pitch角度 所需补偿
     * @param vision_data 视觉数据包
     * @param time_stamp 时间戳
     */
    void getAngle(VisionData& vision_data, int time_stamp);

    /**
     * @brief 反陀螺模式 根据不同距离来设置滑动窗口大小，锁定目标
     * @param vision_data 视觉数据包 云台偏差角
     */
    void getTopAngle(VisionData& vision_data);

    /**
     * @brief 惯性坐标曲线拟合, 预测用
     * @param result 拟合结果
     * @param time_stamp 时间戳
     */
    bool curveFitting(Eigen::Vector3d &result, int time_stamp);

    /**
     * @brief 反陀螺模式 云台偏差角 根据历史惯性坐标来设置滑动窗口大小，锁定目标
     * @param pw 惯性坐标
     */
    [[deprecated("TODO: waitting for test")]]void getTopAngle2(Eigen::Vector3d &pw);

private:

    struct CURVE_FITTING_COST {
        CURVE_FITTING_COST (double x, double y): _x (x), _y(y) {}
        template <typename T>
        bool operator()(const T* const params, T* residual) const {
            residual[0] = T (_y) - params[0] * T(_x); // f(x) = a0 + a1 * x
            //residual[0] = T (_y) - params[0] * T(_x) - params[1] * T(_x) * T(_x); // f(x) = a0 + a1 * x + a2 * x^2
            return true;
        }
        const double _x, _y;    // x,y数据
    };

    //目标信息
    struct TargetInfo {
        Eigen::Vector3d xyz;
        int timestamp;
    };

    // 历史信息队列
    std::deque<TargetInfo> history_info;
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> z_history;

    CoordSolver coord_solver;

    Kalman kf;           // 普通卡尔曼
    CVKalman cv_kf;      // 普通卡尔曼
    AdaptiveKalman ekf;  // 扩展卡尔曼

    float last_yaw;      // 上一次的yaw
    float last_pitch;    // 上一次的pitch
    int last_time_stamp;
};


/**
 * @brief 能量机关角度解算
 */
class BuffAngleSolver {
public:
    BuffAngleSolver() = default;

    /**
     * @brief 带标定参数初始化
     * @param 标定参数
     */
    BuffAngleSolver(Setting& setting);

    ~BuffAngleSolver() = default;

    /**
     * @brief 计算目标Yaw Pitch角度 所需补偿
     * @param
     */
    void getAngle(VisionData& vision_data);

    /**
     * @brief 枪管复位归中
     */
    void barrelReturnCenter(VisionData& vision_data);

private:
    CoordSolver coord_solver;

    // 能量机关打击模型参数
    int h_buff;		    // R标 中心高度
    int r_buff;		    // 半径
    int dist_buff;	    // 与己方激活点距离
    int h_car;	        // 己方激活点高度
    int h_platform;     // 己方枪管离麦轮接触面高度

};

#endif //SOLVE_ANGLE_H_INCLUDE
