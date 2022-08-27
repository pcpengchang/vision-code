#ifndef ARMOR_DETECTOR_INCLUDE
#define ARMOR_DETECTOR_INCLUDE

#include "common.h"
#include "utils.h"

#include <opencv2/dnn.hpp>

/**
 * @brief 灯条颜色
 *@Light_ +:
 *  @RED  0，红色
 *  @BLUE 1，蓝色
 */
enum LightColor {
    Light_RED  = 0,
    Light_BLUE = 1
};

/**
 * @brief 灯条类
 */
class Light : public cv::RotatedRect {
public:
    Light() = default;
    ~Light() = default;

    /**
     * @brief 用拟合出来的旋转矩阵构造灯条
     * @param lightRect 灯条轮廓拟合出来的旋转矩阵
     */
    explicit Light(const cv::RotatedRect& lightRect) ;

    /**
     * @brief 判断是否为合适的尺寸
     * @return true / false
     */
    static bool isValidLight(const Light& light);

public:
    cv::Point2f top;     // 灯条顶边中心
    cv::Point2f bottom;  // 灯条底边中心
    cv::Point2f center;  // 灯条中心
    double height;       // 灯条长度
    double width;        // 灯条宽度
    float angle;         // 灯条长度方向与竖直方向的夹角，左偏为0~90,右偏为0~-90

public:
    static bool coordinateComparator(const Light &a, const Light &b) {
        if (a.center.x == b.center.x) {
            return a.center.y < b.center.y;
        } else {
            return a.center.x < b.center.x;
        }
    }
};


/**
 * @brief 灯条对类
 * @details 由两个灯条构造，通过灯条之间的长度比，角度差，高度差,间距,内侧区域强度，几何关系等匹配
 */
class LightPair {

public:
    LightPair() = default;
    ~LightPair() = default;

    /**
     * @brief 使用两灯条来拟合装甲
     */
    LightPair(const Light& l1, const Light& l2);

    /**
     * @brief 计算装甲板内部区域平均强度，用于筛选含有内嵌灯条的装甲板
     * @param img 装甲板内部区域
     * @return 平均强度
     */
    int getAverageIntensity(const cv::Mat& img) const;

    /**
     * @brief 判断是否为合适的尺寸
     * @return true / false
     */
    bool isValidArmor() const;

public:
    cv::Rect rect;	            //灯条对构成的矩形区域
    cv::RotatedRect rrect;      //灯条对构成的矩形区域
    cv::Point2i pt_cen;		    //灯条对构成矩形的中心(以ROI为背景)
    std::array<Light, 2> light; //灯条对，顺序为左、右
};

/**
 * @brief 装甲板类
 * @details 灯条高大约6cm，装甲高大约12cm，小装甲宽大约12cm，大装甲宽大约24cm
 */
class Armor: public LightPair {
public:

    /**
     * @brief 默认空构造, 只经过默认构造函数操作的装甲板对象是无效的
     */
    Armor(): is_exist(false) { }

    /**
     * @brief 使用两灯条来拟合装甲板, 经过自定义构造函数操作的装甲板对象是有效的
     * @param pair 要拟合的灯条对
     * @param sz   图像的大小
     */
    Armor(const LightPair& pair);

    ~Armor() = default;

    /**
     * @brief 赋值运算符重载函数
     * @details 将传入的已有装甲板对象的各项参数赋给另一个装甲板对象，使两个装甲板对象完全相同
     * @param armor 将要赋值的对象
     * @return 赋值后的对象
     */
    Armor &operator=(const Armor &armor);

    //  自瞄优先级防抖
    //  上一次击打的装甲板附近区域 > 优先级 id > 图像最中心的装甲板
    //  上一次击打的装甲板作为本次击打的最高优先级，排除0号和2号，考虑图像位置的匹配：避免 id 抖动
    bool operator > (const Armor &armor) const {
        std::map <int, int> enemy_prior = {
            //HERO > INFANTRY > SENTRY > ENGINEER
            {1, 3}, {3, 2}, {4, 2}, {5, 2}, {6, 1}, {2, 0}, {0, 0},
        };
        if(id != armor.id)
            return enemy_prior[id] > enemy_prior[armor.id];
        else
            return cv::norm(pt_arm - cv::Point(640, 480)) <
                   cv::norm(armor.pt_arm - cv::Point(640, 480));
    }

public:
    cv::Size size_img;      //图像的尺寸
    cv::Point2f pts[4];     //装甲板四点
    cv::Point2i pt_arm;     //灯条对构成矩形的中心(以原图为背景)
    bool is_exist;          //是否为空
    int id;                 //装甲板id
    int time_stamp;         //装甲板时间戳

    int color;              //颜色
    int area;               //面积
    double conf;            //置信度
    std::string key;        //颜色+ID
};

/**
 * @brief 装甲板追踪器
 */
class ArmorTracker {
public:
    /**
     * @brief 构造新装甲板和对应的时间戳信息，并存入历史装甲板
     */
    ArmorTracker(Armor& src, const int src_timestamp);

    /**
     * @brief 更新同一块装甲板和对应的时间戳信息，并存入历史装甲板
     */
    void update(Armor& new_armor, const int new_timestamp);

    /**
     * @brief 计算当前装甲板击打分数，由装甲板旋转角度和面积大小决定
     */
    void calcTargetScore();

public:
    Armor armor;     // 装甲板
    int timestamp;   // 装甲板对应时间戳

    int last_selected_timestamp; // 上一目标装甲板对应时间戳
    double hit_score;            // 击打分数
};

/**
 * @brief 装甲板id和size分类器
 * @details 对于装甲板ID，有cv::ml::SVM，cv::dnn+MLP 和 cv::dnn+Lenet三种方法
 */
class ArmorClassifier {
public:
    ArmorClassifier() = default;
#if TYPE_ARMOR_CLASSIFIER == 0
    /**
     * @brief 初始化svm分类器
     * @param path_svm svm分类器路径(.xml)
     */
    ArmorClassifier(std::string path_svm) {
        hog.winSize = cv::Size(48, 32);
        hog.gammaCorrection = false;
        svm = cv::ml::SVM::load(path_svm);
    }

#elif TYPE_ARMOR_CLASSIFIER == 1 || TYPE_ARMOR_CLASSIFIER == 2

    /**
     * @brief 初始化
     * @param path_dnn_net   模型路径(.onnx)
     */
    ArmorClassifier(std::string path_dnn_net) {
        net = cv::dnn::readNetFromONNX(path_dnn_net);

        std::ifstream label_file("../configs/other/label.txt");
        std::string line;
        while (std::getline(label_file, line))
            class_names.emplace_back((int)line[0]);
    }
#endif
    ~ArmorClassifier() = default;

    /**
     * @brief Gamma矫正增强数字显示效果
     * @param src 源图像
     * @param gammaG 矫正变量, 大于1以后, 越大则对低灰度值的增强越明显
     * @param gammaC 常系数, 通常为1
     */
    void gammaCorrect(cv::Mat &src, const double gammaG = 2.2, const double gammaC = 1.0);

    /**
     * @brief 获取图像增强后的装甲板数字区域
     * @details Gamma矫正增强数字,
     * @param armor    装甲数据
     * @param img 装甲板ROI区
     */
    void getArmorROI(Armor& armor, cv::Mat& img_roi);

    /**
     * @brief 获取装甲板数字
     * @param armor    装甲数据
     * @param img_roi 装甲板ROI区
     */
    void getArmorID(Armor& armor, cv::Mat& img_roi);

    /**
     * @brief 获取装甲板大小类型
     * @param armor    装甲板数据
     */
    void getArmorType(const Armor& armor);

public:

    cv::Mat img_num = cv::Mat(cv::Size(48, 48), CV_8UC1);     //数字图像

    cv::HOGDescriptor hog;           //hog特征
    cv::Ptr<cv::ml::SVM> svm;	     //装甲SVM分类器
    std::vector<float> descriptors;	 //存放结果

    //  ImageNet的均值和标准差，用于归一化
    constexpr static float mean_value = 0.2105;
    constexpr static float std_value  = 0.2829;
    cv::dnn::Net net;
    std::vector<int> class_names;
};

/**
 * @brief 陀螺观测器
 */
class SpinObserver {
public:
    /**
     * @brief 陀螺识别
     * @param
     */
    void getSpinData(const Armor &armor, const int time_stamp, const std::multimap<int, Armor> &armor_history);

    /**
     * @brief 更新陀螺Score
     */
    void updateSpinScore();

private:
    //CircleQueue<double, 1000> top_queue;

    enum SpinStatus {
        UNKNOWN = 0,
        CLOCKWISE = 1,      // 顺时针旋转
        ANTI_CLOCKWISE = 2  // 逆时针旋转
    };

    int last_time;
    float last_angle;
    float last_center_x;
    std::vector<float> angles;
    int left_spin_times;
    int right_spin_times;

    std::map<int, double> spin_score;          //记录各装甲板小陀螺可能性分数，大于0为逆时针旋转，小于0为顺时针旋转
    std::map<int, SpinStatus> spin_status_map; //反小陀螺，记录该车小陀螺状态

};

/**
 * @brief 装甲板检测类
 */
class ArmorDetector {

public:

    ArmorDetector() = default;
#if TYPE_ARMOR_CLASSIFIER == 0
    /**
     * @brief 初始化
     * @param img_src   相机画面
     * @param setting   固定参数
     */
    ArmorDetector(Setting& setting):
        armor_classifier(setting.path_classifier_svm) { }
#elif TYPE_ARMOR_CLASSIFIER == 1
    /**
     * @brief 初始化
     * @param img_src   相机画面
     * @param setting   固定参数
     */
    ArmorDetector(Setting& setting) :
        armor_classifier(setting.path_classifier_mlp) { }

#elif TYPE_ARMOR_CLASSIFIER == 2
    /**
     * @brief 初始化
     * @param img_src   相机画面
     * @param setting   固定参数
     */
    ArmorDetector(Setting& setting) :
        armor_classifier(setting.path_classifier_lenet) { }
#endif
    ~ArmorDetector() = default;

    /**
     * @brief  装甲识别的主要函数
     * @return 是否识别到装甲
     */
    bool detect(cv::Mat& src, const int time_stamp);

    /**
     * @brief 根据上次装甲设置ROI
     * @details 上一次识别到装甲板，则处理ROI，否则处理全图
     */
    void setROI();

    /**
     * @brief 原图转灰度图转二值图(固定阈值)
     */
    void imgProcess();

    /**
     * @brief 图像转(1. 灰度图 2. 颜色通道相减图 3. 单颜色通道图 & 灰度图)转二值图
     */
    void imgProcess2();

    /**
     * @brief 原图转灰度图转二值图(自适应阈值)
     */
    void imgProcessAdaptive();

    /**
     * @brief 找出所有灯条
     */
    void fitLights();

    /**
     * @brief 对灯条进行两两匹配, 根据一系列标准选出候选装甲板
     */
    void fitArmors();

    /**
     * @brief 从候选装甲板中选出最合适的目标装甲板, 最终按照优先级从高到低排列
     * @param armor_target 目标装甲
     */
    void chooseTarget(Armor& armor_target, const int time_stamp);

    /**
     * @brief 从目标中获取点，用于角度解算
     * @param armor_target 目标装甲
     */
    void getTargetPoints(const Armor& armor_target);

    /**
     * @brief 选择目标车辆ID
     * @param armors 候选装甲板
     * @param timestamp 本次时间戳
     */
    std::string chooseTargetID(const std::vector<Armor> &armors, const int timestamp);

    /**
     * @brief 选择目标车辆的具体装甲板
     * @param trackers 候选装甲板
     * @param timestamp 本次时间戳
     */
    ArmorTracker* chooseTargetTracker(const std::vector<ArmorTracker*> &trackers, const int timestamp);

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
    cv::Mat           img;            //图像
    cv::Mat           img_roi;        //图像roi区域
    cv::Mat           img_gray;	      //灰度图像
    cv::Mat           img_bin;	      //二值化图像

    // 膨胀核
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat element2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::RotatedRect rect_roi_last;

    cv::Point         pt_roi;         //roi图像偏移，即roi左上角点在源图的位置
    cv::Point2f       pts[4];         //装甲板四点

    std::vector<Light> light_list;           //灯条  候选区
    std::vector<Armor> armor_list;           //装甲板候选区
    std::multimap<int, Armor> armor_history; //历史装甲板

    ArmorClassifier   armor_classifier;	  //装甲板分类器
    SpinObserver      spin_observer;      //陀螺观测器
    DestoryableWindow win_roi;            //ROI对应的窗口

    Armor last_target;                    //上一次的目标装甲板

    bool num_classifier = true;           //是否使用分类器
    int cnt_detect;                       //目标丢失次数
    int cnt_lost;                         //目标丢失次数
    int last_time;                  //上一次的时间戳
};

#endif	//ARMOR_DETECT_H_INCLUDE
