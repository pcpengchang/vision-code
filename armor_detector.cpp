#include "armor_detector.h"

Light::Light(const cv::RotatedRect& lightRect) {

    cv::Point2f p[4];
    lightRect.points(p);
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) {
        return a.y < b.y;
    });

    center = Tool::getCenter(p);
    top    = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    height = cv::norm(top - bottom);
    width  = cv::norm(p[0] - p[1]);

    angle  = Tool::radian2Angle(std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)));
}

bool Light::isValidLight(const Light& light) {

    float ratio = light.width / light.height;
    bool ratio_ok = 0.1f < ratio && ratio < 0.4f; // 0.55
    // std::cout << "ratio " << ratio << std::endl;

    bool angle_ok = light.angle < 35.f;  // 筛选倾斜过度的灯条

    return ratio_ok && angle_ok;
};

LightPair::LightPair(const Light& l1, const Light& l2) {
    light[0] = l1;
    light[1] = l2;
    pt_cen   = (l1.center + l2.center) / 2;

    rect.width  = static_cast<int>(cv::norm(light[1].center - light[0].center));
    rect.height = static_cast<int>(light[0].height + light[1].height) ;
    rect.x = pt_cen.x - rect.width  / 2;
    rect.y = pt_cen.y - rect.height / 2;
}

int LightPair::getAverageIntensity(const cv::Mat& img) const {
    return Tool::getRoiAverageIntensity(img, rect);
}

bool LightPair::isValidArmor() const {
    // TODO：阈值是否为最优解？
    // 灯条长度比
    // 灯条角度差
    // 中心 Y 方向差距比值  错位度 => 两灯条连接端点后应接近矩形而非平行四边形
    // 中心间距与灯条的长度比 common:[2,4]

    float light_height_ratio   = _min(light[1].height / light[0].height,
                                      light[0].height / light[1].height);
    bool light_height_ratio_ok = light_height_ratio > 0.7f;


    float light_angle_err    = fabs(light[1].angle - light[0].angle);
    bool light_angle_err_ok  = light_angle_err < 5.5f;


    cv::Point2f light_center_diff = light[1].center - light[0].center;
    bool light_angle_ok = Tool::radian2Angle(std::abs(
                              std::atan(light_center_diff.y / light_center_diff.x))) < 35.f;


    float light_height_avg   = (light[0].height + light[1].height) / 2;
    float center_distance = cv::norm(light[0].center - light[1].center) / light_height_avg;
    bool center_distance_ok  = (0.8f < center_distance && center_distance < 2.8f) ||
                               (3.2f < center_distance && center_distance < 4.3f); // 0.8 3.2 3.2 5.0

    // HINT;
    return light_height_ratio_ok && light_angle_err_ok && light_angle_ok && center_distance_ok;
}

Armor &Armor::operator=(const Armor &armor) {
    this->pt_cen = armor.pt_cen;
    this->pt_arm = armor.pt_arm;
    this->rect = armor.rect;
    this->id = armor.id;
    this->is_exist = armor.is_exist;
    //this->is_exist = false;
    this->light = armor.light;
    memcpy(this->pts, pts, 4);
    return *this;
}

Armor::Armor(const LightPair& pair): LightPair(pair), is_exist(true) {

    // 装甲板矩形的四个点
    pts[0] = cv::Point2f(rect.x,              rect.y + rect.height);
    pts[1] = cv::Point2f(rect.x + rect.width, rect.y + rect.height);
    pts[2] = cv::Point2f(rect.x + rect.width, rect.y);
    pts[3] = cv::Point2f(rect.x,              rect.y);
}

ArmorTracker::ArmorTracker(Armor& src, int src_timestamp) {
    armor = src;
    timestamp = src_timestamp;
    hit_score = 0;
    calcTargetScore();
}

void ArmorTracker::calcTargetScore() {

    cv::Point2f p[4];
    armor.rrect.points(p);
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) {
        return a.y < b.y;
    });
    cv::Point2f top = (p[0] + p[1]) / 2;
    cv::Point2f bottom = (p[2] + p[3]) / 2;

    float rotate_angle = fabs(std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)));
    rotate_angle = Tool::radian2Angle(rotate_angle);

    hit_score = log(0.15 * (90 - rotate_angle) + 10) * (armor.area);
    std::cout << "hit_socre: " << hit_score << "  : " <<
              rotate_angle << " " << armor.area << " " <<
              log(0.15 * (90 - rotate_angle) + 10) << std::endl;

}

void ArmorTracker::update(Armor& new_armor, const int new_timestamp) {
    armor = new_armor;
    timestamp = new_timestamp;
    calcTargetScore();
}

void ArmorClassifier::gammaCorrect(cv::Mat &src, const double gammaG, const double gammaC) {
    uchar *p_src;
    double val;

    //  openmp并行化
    #pragma omp parallel for num_threads(4) default(none) shared(src, rows, cols, gammaG) private(val, p_src, i, j) \
    schedule(static, 1)
    for (int i = 0; i < src.rows; i++) {
        // 获取图像每一行的首地址
        p_src = src.ptr<uchar>(i);
        for (int j = 0; j < src.cols; j++) {
            // 每次迭代获取图像列的地址
            uchar &pix_src = *p_src++;
            // 归一化处理
            val = static_cast<double>(pix_src) / 255;
            // 按公式计算并逆归一化
            pix_src = static_cast<uchar>((pow(val / gammaC, 1 / gammaG) * 255));
        }
    }
}

void ArmorClassifier::getArmorROI(Armor& armor, cv::Mat& img_roi) {

    //透视变换 四个点一一对应
    std::vector<cv::Point2f> pts_points(armor.pts, armor.pts + 4);
    static const std::vector<cv::Point2f> points = {
        cv::Point2f(0, 48), cv::Point2f(48, 48),
        cv::Point2f(48, 0), cv::Point2f(0, 0)
    };
    cv::Mat mat_trans = cv::getPerspectiveTransform(pts_points, points);

    cv::warpPerspective(img_roi, img_num, mat_trans, cv::Size(48, 48));

    //涂黑灯条，减少涣散的灯光对成像质量的影响
    cv::rectangle(img_num, cv::Point(0, 0),  cv::Point(10, 47), BGR::all(0), -1);
    cv::rectangle(img_num, cv::Point(40, 0), cv::Point(47, 47), BGR::all(0), -1);

    static int _gamma = 8;
    //cv::createTrackbar("_gamma", "Parameter", &_gamma, 50, nullptr);

    //图像增强1
    //cv::Mat lookUpTable(1,256,CV_8UC3);//0~255分别非线性矫正，放在lookUpTable,若为多通道则改为CV_8UC3
    //uchar *p = lookUpTable.ptr();
    //double gammaa = _gamma * 0.01;//gamma小于1时，亮度提升
    //for(int i = 0; i < 256; i++)
    //    p[i] = cv::saturate_cast<uchar>(pow((float)i/255.0,gammaa) * 255.0f);//像素点的幂函数转换
    //cv::LUT(img_num, lookUpTable, img_num);

    //图像增强2
    //double max = 0;
    //minMaxLoc(img_num, nullptr, &max);
    //img_num *= (255.0 / max);

    //图像增强3

    convertScaleAbs(img_num, img_num, _gamma);

    cv::cvtColor(img_num, img_num, cv::COLOR_BGR2GRAY);

    //gammaCorrect(img_num, 2, 1);

    //直方图均衡化,滤波去噪
    //cv::equalizeHist(img_num, img_num);
    //cv::medianBlur(img_num,img_num,7);

    cv::imshow("num", img_num);

    //保存数字训练样本
    //static uint32_t number = 0;
    //FileOperation::saveImg("../pic/", number, img_num, ".png");
}

#if TYPE_ARMOR_CLASSIFIER == 0

void ArmorClassifier::getArmorID(Armor& armor, cv::Mat& img_roi) {

    getArmorROI(armor, img_roi);

    //Timer time;
    //time.start();
    descriptors.clear();

    //std::cout << "  confidence: "       << confidence<< std::endl;

    cv::resize(img_num, img_num, cv::Size(48, 32));
    hog.compute(img_num, descriptors, cv::Size(8, 8));    //Hog特征计算
    armor.id = svm -> predict(descriptors);

    //std::cout << "  num: "       << armor.id
    //          << "  spend "      << time.end() << " ms" << std::endl;

}

#elif TYPE_ARMOR_CLASSIFIER == 1

void ArmorClassifier::getArmorID(Armor& armor, cv::Mat& img_roi) {

    getArmorROI(armor, img_roi);

    //Timer time;
    //time.start();
    cv::resize(img_num, img_num, cv::Size(28, 20));
    cv::threshold(img_num, img_num, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    img_num /= 255.0;  //归一化
    cv::Mat blob;
    cv::dnn::blobFromImage(img_num, blob, 1., cv::Size(28, 20));
    net.setInput(blob);

    cv::Mat pred = net.forward();   //  pred为1×n的vector(n为要分类的装甲板数字个数)
    double confidence;
    cv::Point maxLoc;

    //  Do softmax
    float max_prob = *std::max_element(pred.begin<float>(), pred.end<float>());
    cv::Mat softmax_prob;
    cv::exp(pred - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    cv::minMaxLoc(pred.reshape(1, 1), nullptr, &confidence, nullptr, &maxLoc);//  找到pred中置信度最高的项
    armor.id = maxLoc.x;               //  置信度最高项的坐标x值即为1×n vector的索引值，即装甲板id

    //std::cout << "  num: "       << armor.id
    //          << "  threshold: " << confidence
    //          << "  spend "      << time.end() << " ms" << std::endl;
}

#elif TYPE_ARMOR_CLASSIFIER == 2

void ArmorClassifier::getArmorID(Armor& armor,cv::Mat& img_roi) {
    getArmorROI(armor, img_roi);

    Timer time;
    time.start();
    img_num.convertTo(img_num, CV_32FC1, 1.0 / 255, -mean_value); //归一化
    img_num.convertTo(img_num, CV_32FC1, 1.0 /std_value, 0.0);
    cv::Mat blob = cv::dnn::blobFromImage(img_num);
    net.setInput(blob);
    cv::Mat pred = net.forward();                        //  pred为1×n的vector(n为要分类的装甲板数字个数)
    double confidence;
    cv::Point maxLoc;

    cv::minMaxLoc(pred, nullptr, &confidence, nullptr, &maxLoc);     //  找到pred中置信度最高的项
    armor.id = maxLoc.x + 1;                                  //  置信度最高项的坐标x值即为1×n vector的索引值，即装甲板id
    std::cout << "  num: "       << armor.id
              << "  threshold: " << confidence/10
              << "  spend "      << time.end() << " ms" << std::endl;
}

#endif

void ArmorClassifier::getArmorType(const Armor& armor) {
    Data& data = Data::getData();

    //灯条长和装甲宽度之比
    float light_height_avg = (armor.light[0].height + armor.light[1].height) / 2;
    float armor_width = cv::norm(armor.light[0].center - armor.light[1].center);
    float height_scale = _max(armor.light[0].height / armor.light[1].height,
                              armor.light[1].height / armor.light[0].height);

    data.armor.type_armor = ArmorType::SMALL;

    if (armor_width / light_height_avg > 3.3f) {
        data.armor.type_armor = ArmorType::BIG;
    }
    else if (armor_width / light_height_avg > 2.6f && height_scale > 1.3f) {
        data.armor.type_armor = ArmorType::BIG;
    }
}

void SpinObserver::getSpinData(const Armor &armor, const int time_stamp, const std::multimap<int, Armor> &armor_history) {
    Data& data = Data::getData();
    last_time = time_stamp;

    float angle = data.yaw;
    angles.emplace_back(angle);
    float angle_offset = fabs(angle - last_angle);
    last_angle = angle;

    auto candiadates = armor_history.equal_range(armor.id);
    int same_armors_cnt = armor_history.count(armor.id);

    //std::cout << angle_offset << ' ' << same_armors_cnt << std::endl;
    int spinning_coeffient = 1;
    float center_x = candiadates.first->second.pt_arm.x;
    for (auto i = candiadates.first; i != candiadates.second; ++i) {
        if((*i).second.pt_arm.x - center_x > 10)
            spinning_coeffient *= 2;
        center_x = spinning_coeffient;
    }

    //同一目标两次偏转角度差过大过小，则认为未处于陀螺状态
    if(angle_offset < 0.4f || angle_offset > 3.5f
            || spinning_coeffient < 2 || same_armors_cnt < 1) {
        left_spin_times = 0;
        right_spin_times = 0;
        angles.clear();
        //spin_score[armor.id] *= 0.85;
        last_center_x = armor.pt_arm.x;
        updateSpinScore();
        return;
    }

    float x_offset = armor.pt_arm.x - last_center_x;
    if(x_offset > 40)
        right_spin_times++;
    else if(x_offset < -40)
        left_spin_times++;

    //std::cout << left_spin_times << ' ' << right_spin_times << std::endl;
    //std::cout << spin_score.count(armor.id) << ' ' << angles.size() << std::endl;

    if (angles.size() > 10) {
        //若无该元素则插入该元素
        if (spin_score.count(armor.id) == 0) {
            //HINT;
            spin_score[armor.id] = 1000 * x_offset / fabs(x_offset);
        }
        //若已有该元素且目前旋转方向与记录不同, 左右扭腰状态
        else if (x_offset * spin_score[armor.id] < 0
                 || std::fabs(right_spin_times - left_spin_times < 3)) {
            //HINT;
            spin_score[armor.id] *= 0.5;
        }
        //若已有该元素则更新元素
        else if(right_spin_times > 7 || left_spin_times > 7) {
            //HINT;
            spin_score[armor.id] *= 4.5;
        }
    }

    last_center_x = armor.pt_arm.x;
    //std::cout << "spin_score[armor.id] " << spin_score[armor.id] << std::endl;

    updateSpinScore();
}

void SpinObserver::updateSpinScore() {

    Data& data = Data::getData();

    for (auto score = spin_score.begin(); score != spin_score.end();) {

        SpinStatus spin_status;
        if (spin_score.count((*score).first) == 0)
            spin_status = SpinStatus::UNKNOWN;
        else
            spin_status = spin_status_map[(*score).first];

        //std::cout << "[SpinObserver] Current Spin score : " << (*score).second << std::endl;

        //  若分数过低移除此元素
        if (abs((*score).second) <= 2e3 && spin_status != SpinStatus::UNKNOWN) {
            data.armor.is_anti_spin = false;
            spin_status_map.erase((*score).first);
            score = spin_score.erase(score);
            left_spin_times = 0;
            right_spin_times = 0;
            angles.clear();

            fmt::print(fmt::fg(fmt::color::red), "[SpinObserver] Removing {}.\n", (*score).first);
            continue;
        }

        if (spin_status != UNKNOWN)
            (*score).second = 0.978 * (*score).second - 1 * abs((*score).second) / (*score).second;
        else
            (*score).second = 0.997 * (*score).second - 1 * abs((*score).second) / (*score).second;

        if (abs((*score).second) >= 2e4) {
            data.armor.is_anti_spin = true;
            (*score).second = 2e4 * abs((*score).second) / (*score).second;
            if ((*score).second > 0)
                spin_status_map[(*score).first] = SpinStatus::CLOCKWISE;
            else if((*score).second < 0)
                spin_status_map[(*score).first] = SpinStatus::ANTI_CLOCKWISE;

            fmt::print(fmt::fg(fmt::color::pale_violet_red), "[SpinObserver]Fitting Succeed!{} \n", spin_status);
        }
        else if(isnan((*score).second) || (*score).second < 10) {
            spin_status_map.erase((*score).first);
            score = spin_score.erase(score);
            continue;
            //spin_score[(*score).first] *= 0.5;
        }
        ++score;
    }
}


void ArmorDetector::setROI() {

    //上一次识别到装甲板，则处理ROI，否则处理全图
    Data& data = Data::getData();

    cv::Rect rect_roi;  //ROI 矩形搜索框

    static float ratio_w = 2.5;
    static float ratio_h = 1.5;

    //  目标彻底丢失, 搜索整个图像
    if(rect_roi_last.center.x == 0 || rect_roi_last.center.y == 0) {
        rect_roi = cv::Rect(0, 0, img.cols, img.rows);
    }
    //  继承上一图像，并据此得到ROI框
    else {
        Tool::expandRect(rect_roi_last, ratio_w, ratio_h);
        rect_roi = Tool::makeRectSafe(img, rect_roi_last);
    }

    pt_roi  = cv::Point2f(rect_roi.x, rect_roi.y);
    img_roi = img(rect_roi);
    cv::rectangle(data.img_show, rect_roi, BGR::all(255), 3);

#ifdef SHOW_ROI
    if (rect_roi.width != img.cols && rect_roi.height != img.rows)
        win_roi.showImage(img_roi);
    else
        win_roi.destory();
#endif
}

void ArmorDetector::imgProcess() {
    Parameter& param = Parameter::getParameter();
    cv::cvtColor(img_roi, img_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(img_gray, img_bin, param.armor.th_gray, 255, cv::THRESH_BINARY);
}

void ArmorDetector::imgProcess2() {

    Data& data = Data::getData();
    Parameter& param = Parameter::getParameter();

    cv::Mat img_color, img_white, img_green;

    cv::cvtColor(img_roi, img_gray, cv::COLOR_BGR2GRAY);

    cv::threshold(img_gray, img_bin, param.armor.th_gray_blue, 255, cv::THRESH_BINARY);

    std::vector<cv::Mat> channels(3);	//bgr通道
    cv::split(img_roi, channels);

    // 红蓝不同阈值
    if (data.mcu.color_enemy == EnemyColor::RED) {
        cv::subtract(channels[2], channels[0], img_color);
        cv::subtract(channels[2], channels[1], img_green);
        // 使用不同的二值化阈值进行灯条提取，减少环境光照影响
        // 亮度部分
        cv::threshold(img_gray,  img_gray,  param.armor.th_gray_red,  255, cv::THRESH_BINARY);
        // 颜色部分
        cv::threshold(img_color, img_color, param.armor.th_color_red, 255, cv::THRESH_BINARY);
        cv::dilate(img_color, img_color, element);

        // 颜色部分
        //cv::threshold(img_green, img_green, param.armor.th_green, 255, cv::THRESH_BINARY);
        //cv::dilate(img_green, img_green, element);
    }
    else if(data.mcu.color_enemy == EnemyColor::BLUE) {
        cv::subtract(channels[0], channels[2], img_color);
        cv::subtract(channels[0], channels[1], img_green);
        //  过滤紫色灯条
        //  cv::threshold(channels[2], img_white, param.armor.th_purple, 255, cv::THRESH_BINARY);
        //  cv::bitwise_not(img_white, img_white);
        //  使用不同的二值化阈值进行灯条提取，减少环境光照影响
        //  亮度部分
        cv::threshold(img_gray,  img_gray,  param.armor.th_gray_blue,  255, cv::THRESH_BINARY);
        //  颜色部分
        cv::threshold(img_color, img_color, param.armor.th_color_blue, 255, cv::THRESH_BINARY);
        cv::dilate(img_color, img_color, element);

        //  颜色部分
        //cv::threshold(img_green, img_green, param.armor.th_green, 255, cv::THRESH_BINARY);
        //cv::dilate(img_green, img_green, element);
    }

    //  逻辑与
    img_bin = img_gray & img_color;
    //cv::dilate(img_bin, img_bin, element);

#ifdef SHOW_GRAY
    cv::imshow("gray", img_gray);
#endif

#ifdef SHOW_COLOR
    cv::imshow("color", img_color);
#endif

    //cv::imshow("green", img_green);

#ifdef SHOW_BINARY
    //cv::imshow("binary", img_bin);
#endif

}

void ArmorDetector::imgProcessAdaptive() {

    Data& data = Data::getData();

    cv::Mat src     = img_roi.clone();
    cv::Mat gray_br = cv::Mat::zeros(src.size(), CV_8UC1);
    cv::Mat gray_g  = cv::Mat::zeros(src.size(), CV_8UC1);

    // 因为常量会比变量快，所以当图像通道确定时，用常量
    // bgr 三通道的权重转化为 0.1、0.2、0.7
    int src_rows = src.cols * 3;
    for(int i = 0; i < src.rows; i++) {
        uchar *p_src     = src.data + src_rows * i;
        uchar *p_gray_br = gray_br.data + gray_br.cols * i;
        uchar *p_gray_g  = gray_g.data + gray_g.cols * i;

        int jj = 0;
        for(int j = 0; j < src_rows; j += 3) {
            if (data.mcu.color_enemy == EnemyColor::BLUE)
                *(p_gray_br + jj) = *(p_src + j) * 0.7 + *(p_src + j + 1) * 0.2 + *(p_src + j + 2) * 0.1;
            else if (data.mcu.color_enemy == EnemyColor::RED)
                *(p_gray_br + jj) = *(p_src + j) * 0.1 + *(p_src + j + 1) * 0.2 + *(p_src + j + 2) * 0.7;
            *(p_gray_g + jj) = *(p_src + j + 1);
            jj++;
        }
    }
    // 最大灰度
    uchar gray_max = gray_br.data[0];
    long gray_sum = 0;
    for(int i = 0; i < gray_br.rows; i++) {
        uchar *data = src.ptr<uchar>(i);
        uchar *p = gray_br.data + i * gray_br.cols;
        for(int j = 0; j < gray_br.cols; j++) {
            gray_sum += data[i];
            gray_max = _max(*(p + j), gray_max);
        }
    }
    // 平均灰度
    int gray_avg = gray_sum * 1.0 / (src.cols * src.rows);
    // 最大灰度 x 0.6+平均灰度 x 0.4
    int thre = gray_max * 0.6 + gray_avg * 0.4;

    img_gray = gray_br;

    //std::cout << "thre " << thre << std::endl;
    cv::threshold(img_gray, img_bin, thre, 255, 0); // 二值化

    cv::dilate(img_bin, img_bin, element);

    //cv::imshow("img_bin", img_bin);
}

void ArmorDetector::fitLights() {

    Data& data = Data::getData();

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_bin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& light_contour : contours) {
        double perimeter = cv::arcLength(light_contour, true);
        double area = cv::contourArea(light_contour);
        if (area < 25 || area > 6000) {
            continue;
        }

        // 轮廓点数小于 5，不可拟合
        if (perimeter < 15 || perimeter > 400 || light_contour.size() < 5) {
            continue;
        }

        cv::RotatedRect light_bar = cv::minAreaRect(light_contour);
        auto light = Light(light_bar);

        if(Light::isValidLight(light)) {
            cv::Rect rect = light_bar.boundingRect();
            //  Timer t;
            if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= img_roi.cols &&
                    0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= img_roi.rows) {

                cv::Mat roi = img_roi(rect);
                int sum_r = 0, sum_b = 0;
                for (int i = 0; i < roi.rows; i++) {
                    for (int j = 0; j < roi.cols; j++) {
                        if (cv::pointPolygonTest(light_contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
                            sum_b += roi.at<cv::Vec3b>(i, j)[0];
                            sum_r += roi.at<cv::Vec3b>(i, j)[2];
                        }
                    }
                }
                //std::cout << "rb_diff " << abs(sum_b - sum_r) / rect.area() << std::endl;
                //std::cout << sum_r << ' ' << sum_b << std::endl;
                int color = sum_r > sum_b ? LightColor::Light_RED : LightColor::Light_BLUE;

                //cv::imshow("light_roi", roi);
                //  t._end("sum_br");

                if(color == data.mcu.color_enemy) {
                    light_list.emplace_back(light);
                    Tool::drawRotatedRectangle(data.img_show, convertRrect(light_bar));
                }
            }

        }
    }

    //候选灯条排序
    sort(light_list.begin(), light_list.end(), Light::coordinateComparator);

}

void ArmorDetector::fitArmors() {

    if(light_list.size() < 2)
        return;

    std::vector <bool> vis(light_list.size(), false);

    for (size_t i = 0; i != light_list.size(); i++) {
        for (size_t j = i; j != light_list.size(); j++) {
            LightPair pair_light(light_list[i], light_list[j]);	//临时灯条对

            if (!pair_light.isValidArmor()
                    //去除含有内嵌灯条的装甲板
                    || pair_light.getAverageIntensity(img_gray) > 50
                    || (vis[i] || vis[j])) {
                continue;
            }

            //  防止共用同一灯条
            vis[i] = true;
            vis[j] = true;

            //  对灯条进行匹配得出装甲板候选区
            Armor armor(pair_light);
            armor.pt_arm = convertPt(armor.pt_cen);
            armor_list.emplace_back(armor);
        }
    }

    //  按照优先级对装甲板进行排序
    sort(armor_list.begin(), armor_list.end(), [&](const Armor &a, const Armor &b) {
        if (last_target.is_exist) {
            // 该目标运动较小
            return cv::norm(a.pt_arm - last_target.pt_arm) <
                   cv::norm(b.pt_arm - last_target.pt_arm);
        } else {
            return a > b;
        }
    });

}

void ArmorDetector::chooseTarget(Armor& armor_target, const int time_stamp) {

    Data& data = Data::getData();

    for (auto &armor: armor_list) {

        armor.id = 3;

        //  如果分类器可用，则使用分类器对装甲板候选区进行筛选
        num_classifier = true;
        if (num_classifier) {
            armor_classifier.getArmorID(armor, img_roi);

            cv::putText(data.img_show, std::to_string(armor.id), armor.pt_arm,
                        cv::FONT_HERSHEY_SIMPLEX, 2, BGR::all(255), 2);
        }

        // 排除2号工程和误识别的装甲板
        if(armor.id == 2)
            continue;

        armor_target = armor;
        armor_target.time_stamp = time_stamp;
        //若存在上次击打目标,时间较短,且该目标运动较小则将其选为候选目标
        //if (armor.id == last_target.id && time_stamp - last_time < 100) {
        //    armor_target = armor;
        // }

        break;
    }

    //删除过久之前的装甲板
    if (armor_history.size() != 0) {
        for (auto iter = armor_history.begin(); iter != armor_history.end();) {
            auto next = iter;
            if ((time_stamp - (*iter).second.time_stamp) > 1000)
                next = armor_history.erase(iter);
            else
                ++next;
            iter = next;
        }
    }

    if(armor_target.is_exist) {
        cnt_detect++;
        cnt_lost = 0;

        last_target = armor_target;
        last_target.rect = convertRect(last_target.rect);
        rect_roi_last = cv::RotatedRect(last_target.rect.tl(), last_target.rect.br(),
                                        cv::Point2f(last_target.rect.x + last_target.rect.width,
                                                last_target.rect.y + last_target.rect.height));

        armor_classifier.getArmorType(armor_target);
        cv::putText(data.img_show, " type_armor: " + std::to_string(data.armor.type_armor),
                    cv::Point2f(0, 50), cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 255});

        last_time = time_stamp;
        armor_history.insert(std::make_pair(armor_target.id, armor_target));
        cnt_detect > 5 ? data.tracker_state = TrackerState::TRACKING : data.tracker_state = TrackerState::DETECTING;
        spin_observer.getSpinData(armor_target, time_stamp, armor_history);
    }
    else {
        cnt_detect = 0;
        cnt_lost++;
        data.tracker_state = TrackerState::TEMP_LOST;
        //  逐次加大搜索范围（根据相机帧率调整参数）
        if (cnt_lost == 3)
            Tool::expandRect(rect_roi_last, 1.5, 1.5);
        else if(cnt_lost == 5)
            Tool::expandRect(rect_roi_last, 1.5, 1.5);
        //else if(cnt_lost > 5)
        else if(cnt_lost > 5) {
            last_target.is_exist = false;
            data.tracker_state = TrackerState::LOST;
            armor_history.clear();
            rect_roi_last = cv::RotatedRect();
        }
    }

    //spin_observer.getSpinData(armor_target, time_stamp, armor_history);

}

void ArmorDetector::getTargetPoints(const Armor& armor_target) {

    Data& data = Data::getData();

    // 左右灯条分类，顺时针提取装甲板四个角点
    pts[0] = convertPt(static_cast<cv::Point2f>(armor_target.light[0].top));
    pts[3] = convertPt(static_cast<cv::Point2f>(armor_target.light[0].bottom));

    pts[1] = convertPt(static_cast<cv::Point2f>(armor_target.light[1].top));
    pts[2] = convertPt(static_cast<cv::Point2f>(armor_target.light[1].bottom));

    // 瞄准点
    cv::Point2f pt_aim = Tool::getCenter(pts);
    cv::circle(data.img_show, pt_aim, 5, BGR::all(255), 2);

    // 将装甲板中心作为空间坐标系的原点，其四个顶点作为2D-3D的四对点
    // points3d中的点需和points2d中的点按顺序一一对应
    for(int i = 0; i < 4; i++) {
        cv::line(data.img_show, pts[i % 4], pts[(i + 1) % 4], {128, 0, 128}, 6);
        data.armor.pts_2d.emplace_back(pts[i]);
    }

    //cv::rectangle(data.img_show, convertRect(armor_target.rect), {128, 0, 128}, 6);

}

std::string ArmorDetector::chooseTargetID(const std::vector<Armor> &armors, const int timestamp) {

    bool is_last_id_exists = false;
    std::string target_key;

    for (auto armor : armors) {
        // 若存在英雄且距离较近，直接选为目标
        if (armor.id == 1 && armor.area > 100) {
            return armor.key;
        }
        // 若存在上次击打目标，且该目标运动较小则将其选为候选目标
        else if (armor.id == last_target.id && abs(timestamp - last_time) < 30) {
            is_last_id_exists = true;
            target_key = armor.key;
        }
    }

    // 以上两者都不存在，选择面积最大的装甲板
    if (is_last_id_exists)
        return target_key;
    else
        return (*armors.begin()).key;
}

ArmorTracker* ArmorDetector::chooseTargetTracker(const std::vector<ArmorTracker*> &trackers, const int timestamp) {

    int target_idx = 0;
    int last_target_idx = -1;
    float max_score = 0;

    for (unsigned int i = 0; i < trackers.size(); i++) {
        if (trackers[i]->timestamp == timestamp) {
            if (trackers[i]->last_selected_timestamp == last_time && abs(last_time - timestamp) < 100)
                last_target_idx = i;
            if (trackers[i]->hit_score > max_score) {
                max_score = trackers[i]->hit_score;
                target_idx = i;
            }
        }
    }

    // 若上次存在目标且分数与当前装甲板相差不大，选择当前装甲板
    if (last_target_idx != -1 && abs(trackers[last_target_idx]->hit_score - max_score) / max_score < 0.1)
        target_idx = last_target_idx;
    return trackers[target_idx];
}


bool ArmorDetector::detect(cv::Mat& src, const int time_stamp) {

    Timer t;

    Data& data = Data::getData();
    img = src.clone();
    data.armor.pts_2d.clear();

    Armor armor_target;

    //清空历史灯条和装甲板信息
    light_list.clear();
    armor_list.clear();

    //划分ROI
    setROI();

    //预处理ROI图像
    imgProcessAdaptive();

    //寻找所有符合灯条条件的轮廓
    fitLights();

    //对灯条进行匹配得出装甲板候选区
    fitArmors();

    //选择最终击打的目标
    chooseTarget(armor_target, time_stamp);

    if (!armor_target.is_exist) {
        data.cnt_lost++;
        return false;
    }
    else {
        data.cnt_lost = 0;
    }

    getTargetPoints(armor_target);

    t._end("Armor");
    return true;
}
