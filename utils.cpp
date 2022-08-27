#include "utils.h"

bool DestoryableWindow::create() {
    if(name == "" || flag)
        return false;
    cv::namedWindow(name);
    flag = true;
    return true;

}

bool DestoryableWindow::destory() {
    if (!flag)
        return false;
    cv::destroyWindow(name);
    flag = false;
    return true;
}

void DestoryableWindow::showImage(const cv::Mat& img) {
    cv::imshow(name, img);
    flag = true;
}

void Tool::drawCross(cv::Mat& img, cv::Point pt, int len_cross, const BGR color,
                     int thickness, int type_line, int shift) {
    cv::line(img, cv::Point2f(pt.x, pt.y - len_cross),
             cv::Point2f(pt.x, pt.y + len_cross),
             color, thickness, type_line, shift);
    cv::line(img, cv::Point2f(pt.x - len_cross, pt.y),
             cv::Point2f(pt.x + len_cross, pt.y),
             color, thickness, type_line, shift);
}

void Tool::drawRotatedRectangle(cv::Mat& img, cv::RotatedRect rrect,
                                const BGR color, int thickness, int type_line, int shift) {
    cv::Point2f pts[4];
    rrect.points(pts);
    for (int i = 0; i < 4; i++)
        cv::line(img, pts[i], pts[(i+1)%4], color, thickness, type_line, shift);
}

int Tool::getRoiAverageIntensity(const cv::Mat& img, cv::Rect rect) {
    if (rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    cv::Mat roi = img(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));
    // imshow("img", roi);
    int average_intensity = static_cast<int>(cv::mean(roi).val[0]);
    // std::cout << "average_intensity " << average_intensity << std::endl;
    return average_intensity;
}

int Tool::getRoiAverageIntensity(const cv::Mat& img) {
    int average_intensity = static_cast<int>(cv::mean(img).val[0]);
    return average_intensity;
}

float Tool::getMeanValue(std::vector<float> data) {
    float sum = 0.f;
    for(auto & value : data)
        sum += value;
    return (sum / data.size());
}

float Tool::limitValue(float value, float value_max) {
    if (value > 0)
        return value > value_max ? value_max : value;
    else
        return value < -value_max ? -value_max : value;
}

float Tool::limitValue(float value, float value_max, float value_last) {
    if (value > 0)
        return value > value_max ? value_last : value;
    else
        return value < -value_max ? -value_last : value;
}

float Tool::smoothAngleChange(float cur_angle, float factor, float th_angle) {
    //next = k * cur
    if(cur_angle >= 0)
        return cur_angle > th_angle ? cur_angle * factor : 0;
    else
        return cur_angle < -th_angle ? cur_angle * factor : 0;
}

cv::Point2f Tool::LineFitting(std::vector<int> x, std::vector<float> y, int size) {
    float x_mean = 0.0f;
    float y_mean = 0.0f;
    for(int i = 0; i < size; i++) {
        x_mean += x[i];
        y_mean += y[i];
    }
    x_mean /= size;
    y_mean /= size;

    float sumx2 = 0.0f;
    float sumxy = 0.0f;
    for(int i = 0; i < size; i++) {
        sumx2 += (x[i] - x_mean) * (x[i] - x_mean);
        sumxy += (y[i] - y_mean) * (x[i] - x_mean);
    }

    float k = sumxy / sumx2;
    float b = y_mean - k*x_mean;

    cv::Point2f point_coefficient;
    point_coefficient.x = k;
    point_coefficient.y = b;

    return point_coefficient;
}

double Tool::judge(cv::Mat &src) {
    cv::Mat image = src;
    int colornum = 0;

    int rows = image.rows;
    int cols = image.cols;

    if (image.isContinuous()) {
        rows = 1;
        cols = image.cols * image.rows;
    }
    for (int i = 0; i < rows; i++) {
        const uchar *inData = image.ptr<uchar>(i);
        for (int j = 0; j < cols; j++) {
            if (*inData > 200)
                colornum++;
            inData++;
        }
    }
    return (double)colornum / ((double)image.cols * (double)image.rows);
}

float Tool::makeAngleRegular(float angle) {
    float angle_tmp;
    angle_tmp = fmod(angle, 360);

    if (angle_tmp < 0)
        angle_tmp += 360;

    return angle_tmp;
}

Eigen::Vector3d Tool::rotationMatrixToEulerAngles(Eigen::Matrix3d &R) {
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular) {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {z, y, x};
}

bool Tool::makeRectSafe(cv::Rect &rect, const cv::Mat &img) {
    if (rect.x < 0)
        rect.x = 0;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.x + rect.width  > img.cols)
        rect.width  = img.cols - rect.x;
    if (rect.y + rect.height >= img.rows)
        rect.height = img.rows - rect.y;
    return !(rect.width <= 0 || rect.height <= 0);
}

cv::Rect Tool::makeRectSafe(const cv::Mat& img, const cv::RotatedRect& rect) {

    int width = rect.boundingRect().width;
    int height = rect.boundingRect().height;

    cv::Point tl = cv::Point(rect.center.x - (width * 0.5f),
                             rect.center.y - (height * 0.5f));
    if (tl.x < 0)
        tl.x = 0;
    if (tl.y < 0)
        tl.y = 0;
    if (tl.x + width > img.cols)
        width = img.cols - tl.x;
    if (tl.y + height > img.rows)
        height = img.rows - tl.y;

    return cv::Rect(tl.x, tl.y, width, height);
}

void Tool::expandRect(cv::RotatedRect &rect, float ratio_w, float ratio_h) {
    rect.size.width *= ratio_w;
    rect.size.height *= ratio_h;
}

cv::Point2f Tool::getCenter(cv::Point2f pts[4]) {
    cv::Point2f center;
    center.x = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4;
    center.y = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4;
    return center;
}

double Tool::getArea(cv::Point2f pts[4]) {
    double bx_a = sqrt(pow(pts[0].x - pts[1].x, 2) + pow(pts[0].y - pts[1].y, 2));
    double bx_b = sqrt(pow(pts[1].x - pts[2].x, 2) + pow(pts[1].y - pts[2].y, 2));
    double bx_c = sqrt(pow(pts[2].x - pts[3].x, 2) + pow(pts[2].y - pts[3].y, 2));
    double bx_d = sqrt(pow(pts[3].x - pts[0].x, 2) + pow(pts[3].y - pts[0].y, 2));
    double bx_z = (bx_a + bx_b + bx_c + bx_d)/2;
    return 2 * sqrt((bx_z - bx_a) * (bx_z - bx_b)*(bx_z - bx_c)*(bx_z - bx_d));
}

void Tool::extractVideo(cv::Mat &src) {
    if (src.empty())
        return;
    float length = static_cast<float>(src.cols);
    float width = static_cast<float>(src.rows);
    if (length / width > 640.0 / 480.0) {
        length *= 480.0 / width;
        resize(src, src, cv::Size(length, 480));
        src = src(cv::Rect((length - 640) / 2, 0, 640, 480));
    } else {
        width *= 640.0 / length;
        resize(src, src, cv::Size(640, width));
        src = src(cv::Rect(0, (width - 480) / 2, 640, 480));
    }
}
