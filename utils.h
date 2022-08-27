#ifndef UTILS_H_INCLUDE
#define UTILS_H_INCLUDE

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cxxabi.h>
#include <mutex>
#include <sys/time.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "log.h"
#include "obj_manager.h"

using BGR = cv::Scalar;

/**
 * @brief 循环队列
 */
template <typename valueType, int queue_size>
class CircleQueue {
public:
    CircleQueue() : head(0), tail(0), length(0), sum(0) {};

    ~CircleQueue() = default;

    inline bool isEmpty() const {
        return (head == tail);
    }

    void push(const valueType &value) {
        if(value > 0)
            vote_cnt[0]++;
        else
            vote_cnt[1]++;
        values[tail] = value;
        tail = (tail + 1) % queue_size;
        if (head == tail) {
            head = (head + 1) % queue_size;
        }
        length = (tail - head + queue_size) % queue_size;
    }

    /* 删掉最旧的数据 */
    bool popHead() {
        if (isEmpty()) {
            std::cout << "warning at popHead(): queue empty!" << std::endl;
            return false; //  为空，不删除数据
        }
        else {
            if(values[head] > 0)
                vote_cnt[0]--;
            else
                vote_cnt[1]--;
            head = (head + 1) % queue_size;
            length = (tail - head + queue_size) % queue_size;
            return true;
        }
    }

    /* 获取倒数第n个数据 */
    bool back(valueType &value, int n) const {
        //  保护
        if (isEmpty()) {
            std::cout << "warning at back(value, n): queue empty!" << std::endl;
            return false;
        }
        if (n > this->length) {
            std::cout << "warning at back(value, n): ask too much data!" << std::endl;
            return false;
        }
        int end = (tail + size - n) % size;
        value = values[end];
        return true;
    }

    int cal() {
        if(abs(vote_cnt[0] - vote_cnt[1]) < 200 || length < size)
            return 2;
        return vote_cnt[0] > vote_cnt[1] ? 0 : 1;
    }

    void clear() {
        head = tail = length = 0;
    }

public:
    valueType values[queue_size];
    int size = queue_size; //  循环队列容量
    int length;            //  记录存储数据的长度
    int head;
    int tail;
    int vote_cnt[2];
    valueType sum;
};

/**
 * @brief 基于opencv highgui的可销毁窗口
 */
class DestoryableWindow {
public:
    DestoryableWindow() { }

    /**
     * @brief 构造函数
     * @param name_win  窗口名字
     * @param is_create 是否要创建窗口
     */
    DestoryableWindow(std::string name_win, bool is_create = false)
        :name(name_win), flag(is_create) { }

    /**
     * @brief 创建窗口
     * @return 是否创建成功
     */
    bool create();

    /**
     * @brief 销毁窗口
     *@输出:
     * @return 是否销毁成功
     */
    bool destory();

    /**
     * @brief 显示图片，若flag为false会自动创建窗口
     * @param img 要显示的图片
     */
    void showImage(const cv::Mat& img);
public:
    std::string name;   //窗口名字
    bool flag;          //是否被使用标志
};

/**
 * @brief 基于opencv的简易计时器
 * @details getTickFrequency()用于返回CPU的频率。单位是秒，也就是一秒内所经的计时周期数
 *          getTickCount()用于返回从操作系统启动到当前所经的计时周期数
 */
class Timer {
public:

    Timer() {
        t_begin = cv::getTickCount();
    };
    ~Timer() = default;

    /**
     * @brief 开始计时
     */
    void start() {
        t_begin = cv::getTickCount();
    }

    /**
     * @brief 结束本次计时 单位ms
     */
    double end() {
        double t_end = cv::getTickCount();
        double time = (t_end - t_begin) / cv::getTickFrequency() * 1000;
        counter.update(time);
        return time;
    }

    /**
     * @brief 结束本次计时 单位ms
     */
    void _end(std::string text) {
        double t_end = cv::getTickCount();
        double time = (t_end - t_begin) / cv::getTickFrequency() * 1000;
        counter.update(time);
        std::cout << text << "_[spend_time]: " << time << std::endl;
    }

    /**
     *@brief: 返回以微妙为单位的时间戳
     */
    unsigned int getTimeUSecNow() {
        timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec * 1000000 + tv.tv_usec;
    }

    /**
     * @brief: 获取当前时间
     */
    double now() {
        timeval tv;
        gettimeofday(&tv, NULL);
        return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
    }

    /**
     * @brief 计算平均时间
     * @param cnt 历史100次的时间
     */
    double getAverageTime() {
        return counter.tol / counter.q.size();
    }

    /**
     * @brief 帧率计算
     */
    void getFPS() {
        time2 = cv::getTickCount();
        time  = (time2 - t_begin) / cv::getTickFrequency();
        fps   = 1.f / time;

        if (++cnt > 10) {
            total   += time;
            average  = total / (cnt - 10);
        }

        //std::cout << "[fps]" << fps << std::endl;
    }

public:
    double fps;       //帧率
    double average;   //平均帧率（去除前10帧）

private:
    double t_begin;   //开始的时间
    int    cnt;       //计算次数
    double time2;     //记录第二次时间点
    double time;      //记录时间段

    double total;     //总帧率

    class AverageTimeCounter {
    public:
        uint cnt;               //最大数量
        double tol;             //总数
        std::queue<double> q;   //时间队列

        /**
         * @param count 待计算平均用时的最大数量，即队列的大小
         */
        AverageTimeCounter(uint count = 100): cnt(count) { }

        /**
         * @brief 更新队列和总数
         * @param 要更新的时间
         */
        void update(double time) {
            q.push(time);
            tol += time;
            while (q.size() > cnt) {
                tol -= q.front();
                q.pop();
            }
        }
    } counter;   //计时器内部类对象
};

template <typename T>
struct LastInfoHelper {
    LastInfoHelper(T c_info, T &r_info) : current_info(c_info), ref_info(r_info) {};

    ~LastInfoHelper() {
        ref_info = current_info;
    }

    T current_info;
    T &ref_info;
};

/**
 * @brief 工具类
　* 存放坐标转换工具，欧式距离计算工具等
 */
class Tool {
public:
    /**
     * @brief 计算二维点的欧氏距离
     * @param pt1, pt2 要计算的两个点
     * @return 距离
     */
    template<class T1, class T2>
    static double getDistance2D(cv::Point_<T1> pt1, cv::Point_<T2> pt2) {
        return sqrtf(powf((pt1.x - pt2.x), 2) + powf((pt1.y - pt2.y), 2));
    }

    /**
     * @brief 计算三维点的欧式距离
     * @param pt1, pt2 要计算的两个点
     * @return 距离
     */
    template<class T1, class T2>
    static double getDistance3D(cv::Point3_<T1> pt1, cv::Point3_<T2> pt2) {
        return sqrtf(powf((pt1.x - pt2.x), 2) + powf((pt1.y - pt2.y), 2) + powf((pt1.z - pt2.z), 2));
    }

    /**
     * @brief 把弧度转化为角度
     * @param rad 要转化的弧度
     * @return 转化出的角度
     */
    static float radian2Angle(float rad) {
        return rad * 180.0f / static_cast<float>(CV_PI);
    }

    /**
     * @brief 把角度转化为弧度
     * @param angle 要转化的角度
     * @return 转化出的弧度
     */
    static float angle2Radian(float angle) {
        return angle / 180.0f * static_cast<float>(CV_PI);
    }

    /**
     * @brief 在点处绘制十字标记
     * @param img       要绘制的原图
     * @param pt        十字原点
     * @param len_cross 十字一侧的线长
     * @param ...       其余与cv::line基本一致
     */
    static void drawCross(cv::Mat& img, cv::Point pt, int len_cross = 10,
                          const BGR color = BGR::all(255),
                          int thickness = 1, int type_line = cv::LINE_8, int shift = 0);

    /**
     * @brief 绘制旋转矩形
     * @param img   要绘制的原图
     * @param rrect 要绘制的旋转矩形
     * @param ..    其余与cv::line基本一致
     */
    static void drawRotatedRectangle(cv::Mat& img, cv::RotatedRect rrect,
                                     const BGR color = {0, 255, 0}, int thickness = 3,
                                     int type_line = 8, int shift = 0);

    /**
     * @brief 计算图像ROI区域强度
     * @param img      要计算的原图
     * @param rect_roi 要计算的矩形区域
     * @return 区域强度
     */
    static int getRoiAverageIntensity(const cv::Mat& img, cv::Rect rect_roi);

    /**
     * @brief 计算图像区域强度
     * @param img  要计算的图像
     * @return 区域强度
     */
    static int getRoiAverageIntensity(const cv::Mat& img);

    static float getMeanValue(std::vector<float> data);

    /**
     * @brief 限制角度
     * @param angle      要限制的角度
     * @param angle_max  最大角度
     * @return 角度值
     */
    static float limitValue(float angle, float angle_max);

    /**
     * @brief 角度插值，若超过了角度限幅，则使用上一次的角度
     * @param angle       要限制的角度
     * @param angle_max   最大角度
     * @param angle_last  上次角度
     * @return 角度值
     */
    static float limitValue(float angle, float angle_max, float angle_last);

    /**
     * @brief 平滑角度变化(防止角度数据跳动过大)
     * @param angle     要限制的角度
     * @param angle_max 最大角度
     * @return 角度值
     */
    static float smoothAngleChange(float cur_angle, float factor, float th_angle);

    /**
     * @brief 最小二乘法线性拟合
     * @param x     数据横坐标
     * @param y     数据纵坐标
     * @param size  数据个数
     * @return 直线斜率和截距
     */
    static cv::Point2f LineFitting(std::vector<int> x, std::vector<float> y, int size);

    /**
     * @brief 计算所输入图像的亮度比例
     * @param src 输入图像
     * @return 图像亮度比例
     */
    static double judge(cv::Mat &src);

    /**
     * @brief 调节输入角度使其范围在[0,360)
     * @param angle 待规范的角度
     * @return 规范后的角度
     */
    static float makeAngleRegular(float angle);

    /**
     * @brief 将旋转矩阵转化为欧拉角
     * @param R 旋转矩阵
     * @return 欧拉角
    */
    static Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);

    /**
     * @brief 限制 ROI 范围
     * @param img 图像
     * @param rect ROI 区域
     */
    static bool makeRectSafe(cv::Rect &rect, const cv::Mat &img);

    /**
     * @brief 限制 ROI 范围
     * @param img 图像
     * @param rect ROI 区域
     * @return cv::Rect 返回安全的 Rect 参数
     */
    static cv::Rect makeRectSafe(const cv::Mat& img, const cv::RotatedRect& rect);

    /**
     * @brief ROI 扩大倍数
     * @param rect    ROI 的旋转矩形
     * @param ratio_w 宽度增加的倍数
     * @param ratio_h 高度增加的倍数
     */
    static void expandRect(cv::RotatedRect &rect, float ratio_w, float ratio_h);

    /**
     * @brief 计算任意四边形的中心
     * @param pts  四边形角点
     */
    static cv::Point2f getCenter(cv::Point2f pts[4]);

    /**
     * @brief 计算任意四边形的面积
     * @param pts  四边形角点
     */
    static double getArea(cv::Point2f pts[4]);

    /**
     * @brief 将视频切成640×480的大小
     * @param src  图像
     */
    static void extractVideo(cv::Mat &src) ;

private:
    static Tool &instance() {
        static Tool tool;
        return tool;
    }
};

/**
 * @brief 求最值
 */
template<class F, class T, class ...Ts>
T reduce(F &&func, T x, Ts... xs) {
    if constexpr (sizeof...(Ts) > 0) {
        return func(x, reduce(std::forward<F>(func), xs...));
    } else {
        return x;
    }
}

template<class T, class ...Ts>
T _max(T x, Ts... xs) {
    return reduce([](auto &&a, auto &&b) {
        return std::max(a, b);
    }, x, xs...);
}

template<class T, class ...Ts>
T _min(T x, Ts... xs) {
    return reduce([](auto &&a, auto &&b) {
        return std::min(a, b);
    }, x, xs...);
}

#endif //  UTILS_H_INCLUDE
