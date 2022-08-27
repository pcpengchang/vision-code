#include "buff_detector.h"
#include "coord_solver.h"

void BuffDetector::setROI() {

    //上一次识别到装甲板，则处理ROI，否则处理全图
    Data& data = Data::getData();

    cv::Rect rect_roi; //图像roi区域

    if(!is_detected) {
        rect_roi = cv::Rect(cv::Point(0, 0), img.size());
        img_roi = img(rect_roi);
        pt_roi = cv::Point(0, 0);
        return;
    }

    cv::RotatedRect rect_roi_tmp = convertRrect(target.rect_big);
    float k = 2.2;
    cv::Point2f center = (k + 1) * target.rect_big.center - k * target.rect_small.center;
    rect_roi_tmp.center = convertPt(center);
    rect_roi_tmp.size.width = 3 * _max(target.rect_big.size.width, target.rect_big.size.height);
    rect_roi_tmp.size.height = 3 * _max(target.rect_big.size.width, target.rect_big.size.height);

    cv::Point2f points_tmp[4];
    rect_roi_tmp.points(points_tmp);
    std::vector<cv::Point2f> points;
    for (int i = 0; i < 4; i++)
        points.emplace_back(points_tmp[i]);

    rect_roi = cv::boundingRect(points);

    Tool::makeRectSafe(rect_roi, img);
    pt_roi = cv::Point(rect_roi.x, rect_roi.y);
    img_roi = img(rect_roi);

    cv::rectangle(data.img_show, rect_roi, BGR::all(255), 3);
}

void BuffDetector::imgProcess() {

    Data& data = Data::getData();
    Parameter& param = Parameter::getParameter();

    cv::Mat img_gray;   //灰度图像
    cv::Mat img_color;  //颜色通道相减图像

    cv::cvtColor(img_roi, img_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Mat> channels(3);  //bgr通道
    cv::split(img_roi, channels);

    if (data.mcu.color_enemy == EnemyColor::BLUE) {
        float conf = 0.7;
        cv::subtract(channels[2], conf * channels[0], img_color);
        //  亮度部分
        cv::threshold(img_gray,  img_gray,  param.buff.th_gray_red,  255, cv::THRESH_BINARY);
        //  颜色部分
        cv::threshold(img_color, img_color, param.buff.th_color_red, 255, cv::THRESH_BINARY);
    }
    else if(data.mcu.color_enemy == EnemyColor::RED) {
        float conf = 1;
        cv::subtract(channels[0], conf * channels[2], img_color);
        //  亮度部分
        cv::threshold(img_gray,  img_gray,  param.buff.th_gray_blue,  255, cv::THRESH_BINARY);
        //  颜色部分
        cv::threshold(img_color, img_color, param.buff.th_color_blue, 255, cv::THRESH_BINARY);
    }

    //  逻辑与
    cv::bitwise_and(img_color, img_gray, img_bin);

    //  膨胀处理, 让箭头连在一起, 断开的灯条连在一起
    cv::dilate(img_bin, img_bin, element2);

#ifdef SHOW_GRAY
    cv::imshow("gray", img_gray);
#endif

#ifdef SHOW_COLOR
    cv::imshow("color", img_color);
#endif

#ifdef SHOW_BINARY
    cv::imshow("binary", img_bin);
#endif
}

void BuffDetector::imgProcessAdaptive() {

    Data& data = Data::getData();
    cv::Mat img_gray;   //灰度图像

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
            if (data.mcu.color_enemy == EnemyColor::RED)
                *(p_gray_br + jj) = *(p_src + j) * 0.7 + *(p_src + j + 1) * 0.2 + *(p_src + j + 2) * 0.1;
            else if (data.mcu.color_enemy == EnemyColor::BLUE)
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

    // std::cout << "thre " << thre << std::endl;
    cv::threshold(img_gray, img_bin, thre, 255, 0);
    cv::imshow("binary", img_bin);

    //  膨胀处理, 让箭头连在一起, 断开的灯条连在一起
    cv::dilate(img_bin, img_bin, element2);

#ifdef SHOW_GRAY
    cv::imshow("gray", img_gray);
#endif

#ifdef SHOW_BINARY
    cv::imshow("binary", img_bin);
#endif
}

void BuffDetector::findCenter() {

    Data& data = Data::getData();
    //根据未激活扇叶和装甲板的位置关系进行目标区域选择
    cv::RotatedRect center_rect = target.rect_big;
    float k = 2.2;
    cv::Point2f center_tmp = (k + 1) * target.rect_big.center - k * target.rect_small.center;
    center_rect.center = center_tmp;
    center_rect.size.width = std::max(target.rect_big.size.width, target.rect_big.size.height) / 3 * 2.5;
    center_rect.size.height = std::max(target.rect_big.size.width, target.rect_big.size.height) / 3 * 2.5;

    cv::Point2f points_tmp[4];
    center_rect.points(points_tmp);
    for (int k = 0; k < 4; k++)
        line(data.img_show, convertPt(points_tmp[k]), convertPt(points_tmp[(k + 1) % 4]), {0, 255, 255}, 2);

    //cv::Mat img_center;
    //img_bin(center_rect.boundingRect()).copyTo(img_center);
    //cv::imshow("img_center", img_center);

    std::vector<cv::Point2f> center_boundary;
    std::vector<cv::Point2f> centers;
    for (size_t i = 0; i < 4; i++)
        center_boundary.emplace_back(points_tmp[i]);

    for (size_t i = 0; i < contours.size(); i++) {
        //if(contourArea(contours[i]) > 500 && contourArea(contours[i]) < 700)
        //std::cout << "findCenter " << contourArea(contours[i]) << std::endl;
        if (contourArea(contours[i]) > 1000 || contourArea(contours[i]) < 150)
            continue;
        cv::Point2f center_tmp;
        float radius_tmp;
        minEnclosingCircle(contours[i], center_tmp, radius_tmp);
        //  检测点是否在轮廓内
        if (pointPolygonTest(center_boundary, center_tmp, false) != 1)
            continue;
        centers.emplace_back(center_tmp);
    }
    if (centers.size() != 1) {
        center_is_detected = false;
        pt_r = center_rect.center;   //  假定圆心
        //pt_r = cv::Point2f(-1, -1);
        //return;
    }
    else if(centers.size())
        pt_r = centers.at(0);

    circle(data.img_show, convertPt(pt_r), 10, {0, 0, 252}, -1);
    circle(data.img_show, convertPt(pt_r), 15, {0, 200, 0}, 2);

    center_is_detected = true;
}

void BuffDetector::getRotatedDirection(float angle) {

    Data& data = Data::getData();
    //采集30个角度数据
    if (angles.empty())
        angles.emplace_back(angle);
    else if (angles.size() < 30) {
        if (fabs(angle - angles[angles.size() - 1]) < 20)
            angles.emplace_back(angle);
        else
            angles.clear();
    }

    if (angles.size() == 30) {
        int stop = 0, clockwise = 0, counter_clockwise = 0;
        for (size_t i = 0; i < 15; i++) {
            if (fabs(angles[i + 15] - angles[i]) < 3.5)
                stop++;
            else if ((angles[i + 15] - angles[i]) > 0)
                clockwise++;
            else
                counter_clockwise++;
        }

        angles.clear();

        if (stop > counter_clockwise && stop > clockwise) {
            s_rot_direction = "ERROR";
            rot_direction = 0;
        }
        else if (clockwise > counter_clockwise) {
            s_rot_direction = "clockwise";
            rot_direction = -1;
        }
        else {
            s_rot_direction = "anticlockwise";
            rot_direction = 1;
        }
    }

    //std::cout << "s_rot_direction : "<< rot_direction << std::endl;
    cv::putText(data.img_show, " Direction:" + s_rot_direction, cv::Point2f(0, 60),
                cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 255});
}

void BuffDetector::calcAngle(Fan &fan) {
    Data& data = Data::getData();

    if (center_is_detected) {
        double angle = atan2(pt_r.y - fan.rect_small.center.y, fan.rect_small.center.x - pt_r.x) / CV_PI * 180;
        fan.angle = Tool::makeAngleRegular(angle); //  过零处理
    }
    else {
        if (fan.rect_small.size.width > fan.rect_small.size.height) {
            if ((fan.rect_small.center.y + fan.rect_small.center.x) < (fan.rect_big.center.y + fan.rect_big.center.x))
                fan.angle = 90 - fan.rect_small.angle;
            if ((fan.rect_small.center.y + fan.rect_small.center.x) > (fan.rect_big.center.y + fan.rect_big.center.x))
                fan.angle = 270 - fan.rect_small.angle;
        }
        if (fan.rect_small.size.width < fan.rect_small.size.height) {
            if ((fan.rect_small.center.y - fan.rect_small.center.x) < (fan.rect_big.center.y - fan.rect_big.center.x))
                fan.angle = -fan.rect_small.angle;
            if ((fan.rect_small.center.y - fan.rect_small.center.x) > (fan.rect_big.center.y - fan.rect_big.center.x))
                fan.angle = 180 - fan.rect_small.angle;
        }
    }

    fan.angle = 360 - fan.angle;
    data.buff.angle = fan.angle;
}

bool BuffDetector::isResectable(cv::RotatedRect &rect) {
    if (rect.center.x - rect.size.width * 0.5 > 0 && rect.center.y - rect.size.height * 0.5 > 0
            && rect.center.x + rect.size.width * 0.5 < img_bin.cols && rect.center.y + rect.size.height * 0.5 < img_bin.rows)
        return true;
    else
        return false;
}

cv::Mat BuffDetector::armorCut(cv::RotatedRect &rect) {
    //将图像旋转后切割
    cv::Mat rot_mat = getRotationMatrix2D(rect.center, rect.angle, 1.0);
    cv::Mat rot_image;
    cv::Size dst_size(img_bin.size());
    cv::warpAffine(img_bin, rot_image, rot_mat, dst_size);
    //imshow("armorCut",rot_image(Rect(rect.center.x - rect.size.width * 0.5, rect.center.y - rect.size.height * 0.5, rect.size.width, rect.size.height)));
    return rot_image(cv::Rect(rect.center.x - rect.size.width * 0.5, rect.center.y - rect.size.height * 0.5, rect.size.width, rect.size.height));
}


void BuffDetector::getRotatedSpeed(int cur_time) {

    Data& data = Data::getData();

    angle_diff = target.angle - angle_last;

    //  过零处理
    if (angle_diff > 180) {
        angle_diff -= 360;
    } else if (angle_diff < -180) {
        angle_diff += 360;
    }

    //  当变化量大于 30°时，则是叶片跳变，则重置 diff 为 0, 速度置零
    if(fabs(angle_diff) > 30.f) {
        same_target = false;
        angle_diff = 0;
    }
    else {
        same_target = true;
    }

    angle_last = target.angle;

    //std::cout << "target.angle : "<< target.angle << std::endl;

    cur_speed = fabs(Tool::angle2Radian(angle_diff) * (1000.0f / (cur_time - last_time))); //角速度(rad/s)

    //std::cout << "cur_speed " << cur_speed << std::endl;
    last_time = cur_time;

    //防止异常数据
    if (fabs(cur_speed) > 3.5 || cur_speed == 0)
        return;

    predict_angle = 0;
    //  小符模式不需要额外计算, 给定恒定转速进行击打
    if(data.mcu.mode_detect == DetectMode::SMALL_BUFF) {
        predictor.predict(cur_speed, predict_angle);
    }
    else {
        predictor.predict(cur_speed, cur_time, predict_angle);
    }
}

void BuffDetector::getTargetPoint() {

    Data& data = Data::getData();

    predict_angle = Tool::limitValue(predict_angle, 60, predict_angle_last); //丢帧插值

    if(rot_direction == 1)
        predict_angle = -predict_angle;

    //  最终角度
    float cur_angle = Tool::makeAngleRegular(target.angle + predict_angle);
    data.buff.angle = Tool::angle2Radian(cur_angle);

    //std::cout << "data.buff.angle "  << data.buff.angle << std::endl;
    //std::cout << "predict_angle "  << predict_angle << std::endl;

    //  最终坐标点
    double ratio = cv::norm(pt_r - target.rect_small.center);
    pt_arm.x = pt_r.x + ratio * cos(data.buff.angle);
    pt_arm.y = pt_r.y + ratio * sin(data.buff.angle);
    cv::circle(data.img_show, convertPt(pt_arm), 10, {255, 255, 125}, 3, 8, 0);
    cv::line(data.img_show, convertPt(pt_arm), convertPt(pt_r), {0, 255, 255}, 2);

    //  最终目标的旋转矩形
    cv::RotatedRect rect_target = cv::RotatedRect(pt_arm, target.rect_small.size, cur_angle);
    cv::Point2f vertex_target[4];
    rect_target.points(vertex_target);

    //  0 1
    //  2 3
    for (int k = 0; k < 4; k++) {
        data.armor.pts_2d.emplace_back(convertPt(vertex_target[(k + 2) % 4]));
        putText(data.img_show, std::to_string(k), data.armor.pts_2d[k],
                cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0});
    }

    for (int k = 0; k < 4; k++) {
        cv::line(data.img_show, convertPt(target.pts_2d[k]), convertPt(target.pts_2d[(k + 1) % 4]), {0, 200, 0}, 4);
        cv::line(data.img_show, convertPt(target.pts_2d[2]), convertPt(pt_r), {0, 200, 0}, 4);
        cv::line(data.img_show, convertPt(target.pts_2d[3]), convertPt(pt_r), {0, 200, 0}, 4);
        cv::line(data.img_show, data.armor.pts_2d.at(k), data.armor.pts_2d.at((k + 1) % 4), {0, 128, 0}, 4);
    }

    cv::putText(data.img_show, " Angle:" + std::to_string(cur_angle),
                cv::Point2f(0, 100), cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 255});

    predict_angle_last = predict_angle;
}

bool BuffDetector::findTarget() {

    Data& data = Data::getData();
    targets.clear();
    contours.clear();
    hierarchy.clear();

    cv::findContours(img_bin, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

    for (size_t i = 0; i != contours.size(); i++) {

        //  用于寻找小轮廓，没有父轮廓的跳过，以及不满足6点拟合椭圆
        if (hierarchy[i][3] < 0 || contours[i].size() < 6
                || contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
            continue;

        //  小轮廓面积
        double small_contour_area = cv::contourArea(contours[i]);
        //if (small_contour_area < 20)
        //    continue;

        cv::RotatedRect rect_small = cv::minAreaRect(contours[i]);

        //  小轮廓长宽比
        float rect_small_size_ratio = _max(rect_small.size.width / rect_small.size.height,
                                           rect_small.size.height / rect_small.size.width);

        if (rect_small_size_ratio > 2.5f)
            continue;

        //
        float small_contour_rect_ratio = small_contour_area / rect_small.size.area();

        if (small_contour_rect_ratio < 0.6f)
            continue;

        //  切割小轮廓并计算亮度比例
        if (isResectable(rect_small)) {

            cv::Mat armor_img = armorCut(rect_small);
            double armor_rate = Tool::judge(armor_img);

            //std::cout << "armor_rate: " << armor_rate << std::endl;
            if (armor_rate > 0.35)
                continue;
        }

        //  大轮廓面积
        double big_contour_area = cv::contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
        //if (big_contour_area < 10000 || big_contour_area > 15000)
        //    continue;

        //  大轮廓长宽比
        cv::RotatedRect rect_big = minAreaRect(contours[static_cast<uint>(hierarchy[i][3])]);
        float rect_big_size_ratio = _max(rect_big.size.width / rect_big.size.height,
                                         rect_big.size.height / rect_big.size.width);

        if (rect_big_size_ratio > 3.0f || rect_big_size_ratio < 1.5f)
            continue;

        //  切割大轮廓并计算亮度比例
        if (isResectable(rect_big)) {
            cv::Mat fan_img = armorCut(rect_big);
            double fan_rate = Tool::judge(fan_img);
            //std::cout << "fan_rate: " << fan_rate << std::endl;
            if (fan_rate < 0.1 || fan_rate > 0.7)
                continue;
        }

        //  内外轮廓的面积比
        //if (rect_big.size.area() / rect_small.size.area() > 10
        //        || rect_big.size.area() / rect_small.size.area() < 4)
        //    continue;

        //  内外轮廓的距离比
        if (cv::norm(rect_big.center - rect_small.center)
                < (_max(rect_big.size.width, rect_big.size.height) / 4))
            continue;

        //  内外轮廓的角度差
        cv::RotatedRect ellipse_rect_small = cv::fitEllipse(contours[i]);
        cv::RotatedRect ellipse_rect_big = cv::fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);
        double angle_diff = fabsf(ellipse_rect_big.angle - ellipse_rect_small.angle);
        if (angle_diff > 100 || angle_diff < 80)
            continue;
        Fan fan;
        fan.rect_small = cv::fitEllipse(contours[i]);
        fan.rect_big = rect_big;

        //std::cout << "big/samll(contour_area): " << big_contour_area / small_contour_area << std::endl;

        update2DPoints(fan, rect_big);

        //判断是否激活
        /*
        if (small_contour_area * 12 > big_contour_area
                && small_contour_area * 5 <= big_contour_are && fan.type == FanType::ACTION) {
            fan.type = FanType::ACTION;
        }
        else if (small_contour_area * 5 > big_contour_area
                 && small_contour_area * 2 < big_contour_area) {
            //std::cout << "big/samll(contour_area): " << big_contour_area / small_contour_area << std::endl;
            fan.type = FanType::INACTION;
            targets.emplace_back(fan);
        }
        else {
            fan.type = FanType::UNKOWN;
        }
        */

        if(fan.type == FanType::INACTION)
            targets.emplace_back(fan);

        putText(data.img_show, std::to_string(fan.type), convertPt(fan.rect_small.center),
                cv::FONT_HERSHEY_COMPLEX, 3, {0, 255, 0});

    }

    //  如果在激活后的一瞬间存在不同步 也会出现这个问题
    if (targets.size() != 1) {
        is_detected = false;
        fmt::print(fmt::fg(fmt::color::red), "[BuffDetector] targets.size() = {} \n\n", targets.size());
        return false;
    }

    is_detected = true;
    target = targets.at(0);

    circle(data.img_show, convertPt(target.rect_small.center), 7, {154, 157, 252}, -1);

    return true;
}

void BuffDetector::update2DPoints(Fan &fan, cv::RotatedRect &rect_big) {

    Data& data = Data::getData();

    cv::Point2f points[4];
    fan.rect_small.points(points);

    // 上层中心点 和 下层中心点
    cv::Point2f pt_cen_up = (points[0] + points[1]) / 2;
    cv::Point2f pt_cen_down = (points[2] + points[3]) / 2;

    // 上下层离外轮廓中点距离
    double dist_up   = cv::norm(pt_cen_up - rect_big.center);
    double dist_down = cv::norm(pt_cen_down - rect_big.center);

    if (dist_up > dist_down) {
        //  0 1
        //  3 2
        fan.pts_2d.emplace_back(points[0]);
        fan.pts_2d.emplace_back(points[1]);
        fan.pts_2d.emplace_back(points[2]);
        fan.pts_2d.emplace_back(points[3]);

    } else {
        //  2 3
        //  1 0
        fan.pts_2d.emplace_back(points[2]);
        fan.pts_2d.emplace_back(points[3]);
        fan.pts_2d.emplace_back(points[0]);
        fan.pts_2d.emplace_back(points[1]);
    }

    // 装甲板的高度差，方向朝向圆心
    cv::Point2f vector_height = fan.pts_2d[0] - fan.pts_2d[3];
    // 两个小roi的中心点
    cv::Point center_left  = fan.pts_2d[3] - vector_height;
    cv::Point center_right = fan.pts_2d[2] - vector_height;

    // 创建roi并绘制
    int width  = 10;
    int height = 5;

    cv::Rect rect_Left(cv::Point(center_left.x - width, center_left.y - height),
                       cv::Point(center_left.x + width, center_left.y + height));// 左边小roi
    cv::Rect rect_right(cv::Point(center_right.x - width, center_right.y - height),
                        cv::Point(center_right.x + width, center_right.y + height));  // 右边小roi

    // 计算光线强度
    int intensity_left  = Tool::getRoiAverageIntensity(img_bin, rect_Left);
    int intensity_right = Tool::getRoiAverageIntensity(img_bin, rect_right);

    if (intensity_left <= 15 && intensity_right <= 15) {
        fan.type = FanType::INACTION;
    } else {
        fan.type = FanType::ACTION;
    }

    cv::rectangle(data.img_show, convertRect(rect_Left),  BGR::all(255), 1);
    cv::rectangle(data.img_show, convertRect(rect_right), BGR::all(255), 1);

    // 画出各个顶点
    for (int k = 0; k < 4; k++)
        putText(data.img_show, std::to_string(k), convertPt(fan.pts_2d.at(k)),
                cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0});
}

BuffPredictor::BuffPredictor() {
    is_params_confirmed = false;

    // 配置求解器
    options.linear_solver_type = ceres::DENSE_QR;   // 增量方程如何求解
    options.minimizer_progress_to_stdout = false;   // 不输出到控制台
    options.num_threads = 3;

    params[0] = 0;
    params[1] = 0;
    params[2] = 0;
    params[3] = 0;

    cv::FileStorage fs("../configs/filter_param.yml", cv::FileStorage::READ);

    // 粒子滤波过程噪声 观测噪声设置
    fs["num_particle"] >> pf.num_particle;
    fs["process_noise"] >> pf.process_noise;
    fs["observe_noise"] >> pf.observe_noise;

    // 卡尔曼滤波过程噪声 观测噪声设置
    fs["Q"] >> kf.Q_;
    fs["R"] >> kf.R_;

    fs.release();

    fmt::print(fmt::fg(fmt::color::green), "[buffPredictor] Set param finished\n");

    pf.initFilter();
}

bool BuffPredictor::predict(double speed, float &result) {

    Data& data = Data::getData();

    int speed_bullet = data.mcu.speed_bullet;
    speed_bullet = 28;

    Eigen::VectorXd measure(1);
    measure << speed;
    pf.update(measure);

    if (pf.is_ready) {
        auto predict = pf.predict();
        speed = predict[0];
    }

    //speed = kf.run(speed);

    //延时补偿
    float delta_time_estimate = (7.0 / std::cos(0.21) / speed_bullet) + delay_small; //s

    result = Tool::radian2Angle(speed * delta_time_estimate);

    std::cout << result << std::endl;

    return true;
}

bool BuffPredictor::predict(double speed, int timestamp, float &result) {

    Data& data = Data::getData();

    int speed_bullet = data.mcu.speed_bullet;
    speed_bullet = 28;

    TargetInfo target = {speed, timestamp};

    //当时间跨度过长视作目标已更新，需清空历史信息队列
    //if (history_info.size() == 0 || target.timestamp - history_info.front().timestamp >= max_timespan) {
    if (history_info.size() == 0 ||
            target.timestamp - history_info.front().timestamp >= 10000) {
        history_info.clear();
        history_info.emplace_back(target);
        params[0] = 0;
        params[1] = 0;
        params[2] = 0;
        params[3] = 0;
        pf.initFilter();
        last_target = target;
        is_params_confirmed = false;

        std::cout << "is_params_confirmed = false " << std::endl;
        return false;
    }
    //std::cout << history_info.size() << std::endl;

    //speed滤波
    Eigen::VectorXd measure(1);
    measure << speed;
    pf.update(measure);

    if (pf.is_ready) {
        auto predict = pf.predict();
        target.speed = predict[0];
    }

    data.data_1 = target.speed;

    //std::cout << target.speed << std::endl;

    int deque_len = 0;

    if (!is_params_confirmed) {
        deque_len = history_deque_len_cos;   //250
        //deque_len = 100;
    }
    else {
        deque_len = history_deque_len_phase; //100
        //deque_len = 50;
    }
    //deque_len = 100;

    //std::cout << deque_len << std::endl;
    if ((int)history_info.size() < deque_len) {
        rotate_speed_sum += target.speed;
        history_info.emplace_back(target);
        last_target = target;
        return false;
    }
    else if ((int)history_info.size() == deque_len) {
        rotate_speed_sum -= history_info.front().speed;
        history_info.pop_front();
        rotate_speed_sum += target.speed;
        history_info.emplace_back(target);
    }
    else if ((int)history_info.size() > deque_len) {
        while((int)history_info.size() >= deque_len) {
            rotate_speed_sum -= history_info.front().speed;
            history_info.pop_front();
        }
        rotate_speed_sum += target.speed;
        history_info.emplace_back(target);
    }

    double mean_speed = rotate_speed_sum / history_info.size();
    //std::cout << rotate_speed_sum << std::endl;

    ceres::Problem problem;                    // 待求解问题
    ceres::Solver::Summary summary;            // 优化信息

    // 拟合函数: f(x) = a * sin(ω * t + θ) + b
    // 拟合a，b，ω，θ
    if (!is_params_confirmed) {

        //std::cout << "!is_params_confirmed" << std::endl;
        double params_fitting[4] = {1, 1, 1, mean_speed};

        // 旋转方向，逆时针为正
        if (rotate_speed_sum / fabs(rotate_speed_sum) >= 0)
            rotate_sign = 1;
        else
            rotate_sign = -1;
        //std::cout << "rotate_sign " << rotate_sign << std::endl;

        for (auto target_info : history_info) {
            problem.AddResidualBlock (
                // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4> (
                    new CURVE_FITTING_COST (static_cast<float>(target_info.timestamp) / 1e3,
                                            target_info.speed  * rotate_sign)
                ),
                //以开始拟合的一帧作为时间轴原点，其余帧与其计算相对时间
                new ceres::CauchyLoss(0.5),    //  核函数
                params_fitting                 //  待估计参数
            );
        }

        //for(int i = 0; i < 4; i++)
        //std::cout << "params_fitting[3] " << params_fitting[3] << ' ';
        //std::cout << std::endl;
        //return false;

        //设置上下限 需根据场上大符实际调整
        problem.SetParameterLowerBound(params_fitting, 0, 0.7);
        problem.SetParameterUpperBound(params_fitting, 0, 1.2);
        problem.SetParameterLowerBound(params_fitting, 1, 1.6);
        problem.SetParameterUpperBound(params_fitting, 1, 2.2);
        problem.SetParameterLowerBound(params_fitting, 2, -CV_PI);
        problem.SetParameterUpperBound(params_fitting, 2, CV_PI);
        problem.SetParameterLowerBound(params_fitting, 3, 0.4);
        problem.SetParameterUpperBound(params_fitting, 3, 2.5);

        //  开始优化
        ceres::Solve(options, &problem, &summary);
        double params_tmp[4] = {params_fitting[0] * rotate_sign, params_fitting[1], params_fitting[2], params_fitting[3] * rotate_sign};
        double rmse = evalRMSE(params_tmp);

        if (rmse > max_rmse) {
            std::cout << summary.BriefReport() << std::endl;
            fmt::print(fmt::fg(fmt::color::red), "[BUFF_PREDICT]RMSE is too high, Fitting failed! RMSE: {}\n", rmse);
            return false;
        }
        else {
            fmt::print(fmt::fg(fmt::color::pale_violet_red), "[BUFF_PREDICT]Fitting Succeed! RMSE: {}\n", rmse);
            params[0] = params_fitting[0] * rotate_sign;
            params[1] = params_fitting[1];
            params[2] = params_fitting[2];
            params[3] = params_fitting[3] * rotate_sign;
            is_params_confirmed = true;
        }
    }
    //参数确定时拟合相位 θ
    else {
        //std::cout << "is_params_confirmed" << std::endl;
        double phase;

        for (auto target_info : history_info) {
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_PHASE, 1, 1> (
                    new CURVE_FITTING_COST_PHASE (static_cast<float>(target_info.timestamp) / 1e3,
                                                  (target_info.speed - params[3]) * rotate_sign,
                                                  params[0], params[1], params[3])
                ),
                new ceres::CauchyLoss(1e1),
                &phase
            );
        }

        problem.SetParameterLowerBound(&phase, 0, -CV_PI);
        problem.SetParameterUpperBound(&phase, 0, CV_PI);

        ceres::Solve(options, &problem, &summary);
        double params_new[4] = {params[0], params[1], phase, params[3]};
        double old_rmse = evalRMSE(params);
        double new_rmse = evalRMSE(params_new);
        if (new_rmse < old_rmse) {
            fmt::print(fmt::fg(fmt::color::pale_violet_red), "[BUFF_PREDICT]Params Update Succeed! RMSE: {}\n", new_rmse);
            params[2] = phase;
        }
    }

    //延时补偿
    float delta_time_estimate = (7.0 / std::cos(0.21) / speed_bullet) * 1e3 + delay_big; //ms
    //std::cout << "delta_time_estimate: " << delta_time_estimate << std::endl;

    float timespan = history_info.back().timestamp;
    float time_estimate = delta_time_estimate + timespan;

    result = calcAimingAngleOffset(params, timespan / 1e3, time_estimate / 1e3);
    last_target = target;

    return true;
}

double BuffPredictor::calcAimingAngleOffset(double params[4], double t0, double t1) {

    Data& data = Data::getData();

    double a = params[0];
    double omega = params[1];
    double theta = params[2];
    double b = params[3];

    //f(x) = a * sin(ω * t + θ) + b  对目标函数进行积分
    fmt::print(fmt::fg(fmt::color::golden_rod), "f(x) = {} sin({} t + {}) \n", a, omega, b);

    double theta0 = (b * t0 - (a / omega) * cos(omega * t0 + theta));
    double theta1 = (b * t1 - (a / omega) * cos(omega * t1 + theta));

    data.data_2 = params[0] * sin (params[1] * t1 + params[2]) + params[3];
    //std::cout << "angle_diff " << (theta1 - theta0) * 180 / CV_PI << std::endl;
    return Tool::radian2Angle(theta1 - theta0);
}

double BuffPredictor::evalRMSE(double params[4]) {
    double rmse_sum = 0;
    double rmse = 0;
    for (auto target_info : history_info) {
        float t = static_cast<float>(target_info.timestamp) / 1e3;
        double pred = params[0] * sin (params[1] * t + params[2]) + params[3];
        double measure = target_info.speed;
        rmse_sum += pow((pred - measure), 2);
    }
    rmse = sqrt(rmse_sum / history_info.size());
    return rmse;
}

bool BuffDetector::detect(cv::Mat& src, const int time_stamp) {

    Timer timer;
    timer.start();

    Data& data = Data::getData();
    img = src.clone();
    data.armor.pts_2d.clear();

    //划分ROI
    setROI();

    //  预处理
    imgProcessAdaptive();

    //  寻找目标
    if(!findTarget()) {
        data.cnt_lost++;
        return false;
    } else {
        data.cnt_lost = 0;
    }

    //  寻找圆心
    findCenter();

    //  计算角度
    calcAngle(target);

    //  计算方向
    getRotatedDirection(target.angle);

    //  计算速度 预测量
    getRotatedSpeed(time_stamp);

    //  选择最终击打的目标
    getTargetPoint();

    timer._end("BUFF");

    return true;
}
