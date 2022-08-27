#ifndef BUFF_DETECTOR_H_INCLUDE
#define BUFF_DETECTOR_H_INCLUDE

#include "common.h"
#include "filter.h"

/**
 * @brief 扇叶类型
 *@Fan_ +:
 *	@UNKOWN	  0，未知类型
 *	@INACTION 1，未激活
 *  @ACTION   2，已激活
 */
enum FanType {
    UNKOWN 	 = 0,
    INACTION = 1,
    ACTION   = 2
};

/**
 * @brief 扇叶类
 */
struct Fan {
    cv::RotatedRect rect_small;  //装甲板矩形
    cv::RotatedRect rect_big;    //扇叶矩形

    cv::Point2f point_small[4];
    cv::Point2f point_big[4];

    std::vector<cv::Point2f> pts_2d;

    int type = FanType::UNKOWN; //扇叶类型
    float angle;                //扇叶角度
    double rate;                //扇叶亮度
};

/**
 * @brief 能量机关预测类
 *            ↑
 *           270
 *            |
 *            |
 *            |          360
 * 180--------|------------→
 *            |          0
 *            |
 *            |
 *            90
 */
class BuffPredictor {
public:
    BuffPredictor();
    ~BuffPredictor() {};

    bool predict(double speed, float &result);

    /**
     * @brief 预测
     * @return 是否预测成功
    */
    bool predict(double speed, int timestamp, float &result);

    /**
     * @brief 计算角度提前量
     * @param params 拟合方程参数
     * @param t0 积分下限
     * @param t1 积分上限
     * @param mode 模式
     * @return 角度提前量(rad)
    */
    double calcAimingAngleOffset(double params[4], double t0, double t1);

    /**
     * @brief 计算RMSE指标
     */
    double evalRMSE(double params[4]);

private:
    //spd = 0.785 * sin (1.884 * t) + 1.305
    //spd = a * sin (w * t) + b
    //  spd 的单位为 rad/s,t 的单位为 s,a 的取值范围为 0.780~1.045,
    //ω的取值范围为 1.884~2.000, b 始终满足 b=2.090- a

    //非线性拟合代价函数
    struct CURVE_FITTING_COST {
        CURVE_FITTING_COST (double x, double y) : _x ( x ), _y ( y ) {}
        //  残差的计算
        template <typename T>
        //  模型参数，有3维   残差
        bool operator() (const T* params, T* residual) const {
            residual[0] = T (_y) - params[0] * ceres::sin(params[1] * T (_x) + params[2]) - params[3];
            //  f(x) = a * sin(ω * t + θ) + b
            return true;
        }
        const double _x, _y;    //  x,y数据

    };
    struct CURVE_FITTING_COST_PHASE {
        CURVE_FITTING_COST_PHASE (double x, double y, double a, double omega, double dc) : _x (x), _y (y), _a(a), _omega(omega), _dc(dc) {}
        //  残差的计算
        template <typename T>
        //  模型参数，有1维   残差
        bool operator() (const T* phase, T* residual) const {
            residual[0] = T (_y) - T (_a) * ceres::sin(T(_omega) * T (_x) + phase[0]) - T(_dc);
            //  f(x) = a * sin(ω * t + θ)
            return true;
        }
        const double _x, _y, _a, _omega, _dc;    //  x,y数据
    };

    //目标信息
    struct TargetInfo {
        double speed;
        int timestamp;
    };

private:
    double params[4];

    double rotate_speed_sum;                   //平均转速
    int rotate_sign;                           //旋转方向
    std::deque<TargetInfo> history_info;       //时间序列
    const int max_timespan = 20000;            //最大时间跨度，大于该时间重置预测器(ms)
    const double max_rmse = 0.4;               //TODO:回归函数最大Cost
    const int history_deque_len_cos = 250;     //大符全部参数拟合队列长度
    const int history_deque_len_phase = 100;   //大符相位参数拟合队列长度

    const double delay_small = 0.1f;           //小符发弹延迟 s
    const int delay_big = 100;                 //大符发弹延迟 ms

    TargetInfo last_target;                    //最后目标
    ParticleFilter pf;
    firstKalman kf;

    ceres::Solver::Options options;            // 求解器

    bool is_params_confirmed;
};

/**
 * @brief 能量机关检测类
 */
class BuffDetector {
public:
    BuffDetector() = default;
    ~BuffDetector() = default;

    /**
     * @brief 能量机关识别的总任务
     * @return 是否识别到目标扇叶
     */
    bool detect(cv::Mat& src, const int time_stamp);


    void setROI();

    /**
     * @brief 图像转(1. 灰度图 2. 颜色通道相减图 3. 单颜色通道图 & 灰度图)转二值图
     */
    void imgProcess();

    /**
     * @brief 原图转灰度图转二值图(自适应阈值)
     */
    void imgProcessAdaptive();

    /**
     * @brief 查找目标
     * @param state 能量机关状态 (1:匀速 2:正弦)
     */
    bool findTarget();

    /**
     * @brief 更新装甲板的四个顶点编号
     * @param 扇叶类
     */
    void update2DPoints(Fan &fan, cv::RotatedRect &rect_big);

    /**
     * @brief 计算能量机关旋转方向
     * @param angle 输入扇叶角度
     */
    void getRotatedDirection(float angle);

    /**
     * @brief 获取最终目标（矩形、顶点）
     */
    void getTargetPoint();

    /**
     * @brief 计算旋转速度
     */
    void getRotatedSpeed(int cur_time);

    /**
     * @brief 寻找能量机关中心R
     */
    void findCenter();

    /**
     * @brief 切割能量机关装甲板图像
     * @param rect 待切割的矩形
     * @return 装甲板图像
     */
    cv::Mat armorCut(cv::RotatedRect &rect);

    /**
     * @brief 判断输入的矩形区域是否超出边界，是否满足裁剪要求
     * @param rect 待判断的矩形
     * @return 1:符合要求 0:不符合要求
     */
    bool isResectable(cv::RotatedRect &rect);

    /**
     * @brief 计算扇叶的角度
     * @details 水平线右上为360°，右下为0°，顺时针增长
     * @param fan 扇叶类
     */
    void calcAngle(Fan &fan);

    //此处顶点坐标皆为裁剪后的图像中的装甲板坐标,
    //需在坐标上加上ROI坐标偏移向量才能得到原图坐标系下的装甲板坐标
    /**
     * @brief 得到roi点在原图的位置
     * @param pt roi上要转换的点
     * @return 原图上的点
     */
    template<class _Pt>
    _Pt convertPt(_Pt pt) {
        return pt + static_cast<_Pt>(pt_roi);
    }

    /**
     * @brief 将相对roi的矩形转换到原图上
     * @param rect roi上要转换的矩形
     * @return 原图上的矩形
     */
    template<class _Rect>
    _Rect convertRect(_Rect rect) {
        rect.x += pt_roi.x;
        rect.y += pt_roi.y;
        return rect;
    }

    /**
     * @brief 将相对roi的旋转矩形转换到原图上
     * @param rrect roi上要转换的旋转矩形
     * @return 原图上的旋转矩形
     */
    template<class _Rrect>
    _Rrect convertRrect(_Rrect rrect) {
        rrect.center = convertPt(rrect.center);
        return rrect;
    }

private:
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<float> angles;

    cv::Point   pt_roi;           //roi图像偏移，即roi左上角点在源图的位置
    cv::Point2f pt_r;             //拟合的圆心坐标
    cv::Point2f pt_arm;           //预测的靶心坐标

    bool is_detected;             //目标识别标志位
    bool center_is_detected;      //R标识别标志位
    bool same_target;             //扇叶跳变标志位

    float angle_diff;             //前后两帧角度差
    float angle_last;             //上一帧扇叶角度
    float predict_angle;          //预测偏移角
    float predict_angle_last;     //上一帧预测偏移角

    int rot_direction;            //旋转方向
    std::string s_rot_direction;  //临时变量  puttext旋转方向

    float last_time;              //上一帧时间戳
    float cur_speed;              //目标扇叶转速

    cv::Rect rect_last;           //上次的ROI区域
    cv::Mat img;                  //图像
    cv::Mat img_roi;              //图像roi区域
    cv::Mat img_bin;              //颜色通道相减图像

    //膨胀核
    cv::Mat element  = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat element2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    //扇叶  大符封装
    std::vector<Fan> targets;
    Fan target;
    BuffPredictor predictor;
};

#endif //  BUFF_DETECT_H
