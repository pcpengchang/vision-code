#ifndef FILTER_H_INCLUDE
#define FILTER_H_INCLUDE

#include <ctime>
#include <future>
#include <random>
#include <vector>

#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <ceres/jet.h>

#include "utils.h"
#include "common.h"

//  x(k|k-1) = AX(k-1|k-1)+BU(k)
//  p(k|k-1) = Ap(k-1|k-1)A'+Q
//  kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
//  x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
//  p(k|k)   = (I-kg(k)H)P(k|k-1)
/**
 * @brief 一阶卡尔曼滤波器
 */
class firstKalman {
public:

    /**
     * @brief 初始化滤波器参数
     */
    firstKalman();

    /**
    * @brief  保护数据
    * @param Q  过程噪声
    * @param R  测量噪声
    * @param t  公式数据
    * @param x0 公式数据
    * @param p0 公式数据
    */
    firstKalman(float Q, float R, float t, float x0, float p0);

    /**
    * @brief 针对单个数据的一阶卡尔曼
    * @param _data  传入需要滤波的数据
    * @return float 返回滤波完毕的数据
    */
    float run(float _data);

    /**
    * @brief 针对两个数据具有一定相关性的一阶卡尔曼
    * @param _data1 传入需要滤波的数据
    * @param _data2 传入需要滤波的数据
    * @return float 传出处理后数据
    */
    float mergeRun(float _data1, float _data2);

public:
    float R_;     //测量噪声
    float Q_;     //过程噪声

private:
    float p_pre_;
    float x_pre_;
    float x_;
    float p_;
    float kg_;
    float t_;
};

/**
 * @brief 角度 KF预测
 */
template<int V_Z = 1, int V_X = 3>
class AngleKalman {
public:

    //  USE:
    //  Eigen::Matrix<double, 1, 1> z_k{vision_data.yaw};
    //  Kalman::Matrix_x1d state = kalman.update(z_k, time_stamp);
    //  std::cout << state(0, 0) << std::endl;

    using Matrix_zzd = Eigen::Matrix<double, V_Z, V_Z>;
    using Matrix_xxd = Eigen::Matrix<double, V_X, V_X>;
    using Matrix_zxd = Eigen::Matrix<double, V_Z, V_X>;
    using Matrix_xzd = Eigen::Matrix<double, V_X, V_Z>;
    using Matrix_x1d = Eigen::Matrix<double, V_X, 1>;
    using Matrix_z1d = Eigen::Matrix<double, V_Z, 1>;

public:
    ~AngleKalman() = default;

    AngleKalman() {
        A << 1, 1, 0, 1;
        H << 1, 0;       //  0.01
        R << 2, 0, 0, 2; //  0.01 100
        Q << 10000000;   //  4
        Matrix_x1d open{0, 0};
        reset(A, H, R, Q, open, 0);
    }

    void reset(Matrix_xxd A, Matrix_zxd H, Matrix_xxd R, Matrix_zzd Q, Matrix_x1d open, double t) {
        this->A = A;
        this->H = H;
        this->P = Matrix_xxd::Zero();
        this->R = R;
        this->Q = Q;
        x_k1 = open;
        last_t = t;
    }

    void reset(Matrix_x1d open, double t) {
        x_k1 = open;
        last_t = t;
    }

    void reset(double x, double t) {
        x_k1(0, 0) = x;
        last_t = t;
    }

    Matrix_x1d update(Matrix_z1d z_k, double t) {
        //  设置转移矩阵中的时间项
        for (int i = 1; i < 2; i++) {
            A(i - 1, i) = t - last_t;
        }
        last_t = t;

        //  预测下一时刻的值, x的先验估计由上一个时间点的后验估计值和输入信息给出
        Matrix_x1d p_x_k = A * x_k1;

        //求先验协方差 p(n|n-1)=A^2*p(n-1|n-1)+q
        P = A * P * A.transpose() + Q;

        //计算kalman增益 Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        //修正结果，最优后验估计, 即计算滤波值  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
        x_k1 = p_x_k + K * (z_k - H * p_x_k);

        //更新后验估计, 计算后验均方差
        P = (Matrix_xxd::Identity() - K * H) * P;

        return x_k1;
    }

private:
    Matrix_x1d x_k1; //  k-1时刻的滤波值，即是k-1时刻的值
    Matrix_xzd K;    //  Kalman增益
    Matrix_xxd A;    //  转移矩阵
    Matrix_zxd H;    //  观测矩阵
    Matrix_xxd R;    //  预测过程噪声偏差的方差
    Matrix_zzd Q;    //  测量噪声偏差
    Matrix_xxd P;    //  估计误差协方差
    double last_t{0};
};

/**
 * @brief 惯性坐标 KF预测 备用
 */
class Kalman {
public:

    Kalman();
    ~Kalman() = default;

    /**
     * @brief 初始化滤波器参数
     * @param position 第一帧的装甲板绝对坐标
     */
    void initFilter(Eigen::VectorXd& position);

    /**
     * @brief 预测函数 输入对应的观测量返回预测量
     * @param position 三维坐标
     * @param time_stamp 时间戳
     */
    cv::Point3f predict(cv::Point3f position, int time_stamp);

    /**
     * @brief 核心算法
     */
    void update();

private:
    Eigen::VectorXd X;	  //状态矩阵包含 位置、速度
    Eigen::MatrixXd F;	  //状态转移矩阵
    Eigen::MatrixXd P;	  //状态协方差矩阵
    Eigen::MatrixXd Q;	  //过程噪声矩阵
    Eigen::MatrixXd H;	  //测量矩阵
    Eigen::MatrixXd R;	  //测量噪声矩阵
    Eigen::VectorXd Z;

    float target_change_threshold = 0.25; // 目标切换距离阈值
    int init_count_threshold;             // 第一帧预测次数
    cv::Point3f last_position;            // 上一次的目标坐标
    int last_time = 0;
};


/**
 * @brief 惯性坐标  KF预测
 * @details 构造函数传入测量量和状态量维度 现有 3×9
 *          需要其他维度的 自行扩展initFilter和predict
 */
class CVKalman {
public:
    CVKalman();
    ~CVKalman() = default;

    /**
     * @brief 初始化滤波器参数
     * @param position 第一帧的装甲板绝对坐标
     */
    void initFilter(cv::Point3f position);

    /**
     * @brief 预测函数 输入对应的观测量返回预测量
     * @param position 三维坐标
     */
    cv::Point3f predict(cv::Point3f position);

public:
    int init_count_threshold;       // 第一帧预测次数
    float control_freq = 18;        // 控制频率 用于计算dt
    float predict_coe;              // 预测量增益时间 用于增大预测量

    float target_change_threshold;  // 目标切换距离阈值

private:
    cv::Mat measurement;            // 观测矩阵
    cv::Point3f last_position;      // 上一次的目标坐标
    cv::KalmanFilter KF;            // 滤波器
};

/**
 * @brief 惯性坐标 EKF预测
 */
class AdaptiveKalman {
    using MatrixXX = Eigen::Matrix<double, 6, 6>;
    using MatrixZX = Eigen::Matrix<double, 3, 6>;
    using MatrixXZ = Eigen::Matrix<double, 6, 3>;
    using MatrixZZ = Eigen::Matrix<double, 3, 3>;
    using VectorX  = Eigen::Matrix<double, 6, 1>;
    using VectorZ  = Eigen::Matrix<double, 3, 1>;

public:

    explicit AdaptiveKalman(const VectorX &X0 = VectorX::Zero())
        : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity()), R(MatrixZZ::Identity()) {}

    ~AdaptiveKalman() = default;

    /**
     * @brief 初始化滤波器参数
     */
    void initFilter();

    /**
     * @brief 核心算法
     */
    void update();

    /**
     * @brief 预测函数 输入对应的工具和观测量返回预测量
     * @param Xr 三维坐标
     * @param time_stamp 时间戳
     */
    Eigen::Vector3d predict(Eigen::Vector3d position, int time_stamp);

public:
    int last_time = 0;
    int init_count_threshold;       // 第一帧预测次数
    double target_change_threshold; // 目标切换距离阈值
    double predict_coe;             // 预测量增益时间 用于增大预测量

    MatrixXX Q;          // 预测过程协方差
    MatrixZZ R;          // 观测过程协方差

private:
    VectorX Xp;          // 预测状态变量
    VectorX Xe;          // 估计状态变量
    VectorX Xr;          // 当前观测量
    VectorZ Zp;          // 预测观测量
    MatrixXX P;          // 状态协方差
    MatrixXX P_;         // 先验状态协方差
    MatrixXX F;          // 预测雅可比
    MatrixZX H;          // 观测雅可比
    MatrixXZ K;          // 卡尔曼增益

    VectorZ Yr;          // 观测量转非线性量

    /**
     * @brief 匀速直线运动模型, 状态x0经线性模型转换后得到新状态x1
     */
    struct PredictTool {
        double dt;    // 距离上帧时长 s
        template <class T>
        void operator()(const T x0[6], T x1[6]) {
            x1[0] = x0[0] + dt * x0[1];
            x1[1] = x0[1];
            x1[2] = x0[2] + dt * x0[3];
            x1[3] = x0[3];
            x1[4] = x0[4] + dt * x0[5];
            x1[5] = x0[5];
        }
    } predict_tool;

    /**
     * @brief 三维坐标转球坐标 xyz -> pyd
     */
    struct CoorTransfromTool {
        template<class T>
        void operator()(const T x[6], T pyd[3]) {
            T xyz[3] = {x[0], x[2], x[4]};
            pyd[0] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]));  //  pitch
            pyd[1] = ceres::atan2(xyz[1], xyz[0]);  //  yaw
            pyd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);  //  distance
        }
    } coor_tool;
};

/**
 * @brief 粒子滤波
 */
class ParticleFilter {
public:
    ParticleFilter() = default;
    ~ParticleFilter() = default;

    /**
     * @brief 初始化滤波器参数
     */
    bool initFilter();

    /**
     * @brief 进行一次预测
     * @return Eigen::VectorXd 预测结果
     */
    Eigen::VectorXd predict();

    /**
     * @brief 进行一次更新
     * @param measure 测量值
     */
    bool update(Eigen::VectorXd measure);

    /**
     * @brief 生成正态分布矩阵
     * @param matrix
     * @return true / false
     */
    bool randomlizedGaussianColwise(Eigen::MatrixXd &matrix);

    /**
     * @brief 重采样
     */
    bool resample();

public:
    bool is_ready;

    int num_particle;         //粒子数
    float process_noise;
    float observe_noise;

private:
    using Matrix_11d = Eigen::Matrix<double, 1, 1>;
    Matrix_11d process_noise_cov;   //过程噪声
    Matrix_11d observe_noise_cov;   //观测噪声

    Eigen::MatrixXd matrix_particle;
    Eigen::MatrixXd matrix_weights;

};


#endif //  FILTER_HPP_INCLUDE

