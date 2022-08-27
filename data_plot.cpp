#include "data_plot.h"

int is_changeOffset;
int offset_v_tmp;
int offset_t_tmp;
int xold;
int yold;

DataPlot::DataPlot(int canvas_w, int canvas_h) {
    data_q_h = 0;
    this->canvas_w = canvas_w;
    this->canvas_h = canvas_h;
    row_size_max = 5;
    col_size_max = 1200;

    pro_offset.proportion_v = 1;  // 值比例
    pro_offset.proportion_t = 1;  // 时间比例
    pro_offset.offset_v = 0;      // 值偏移
    pro_offset.offset_t = 0;      // 时间偏移
}

void DataPlot::dataCollectProc(std::vector<float> data, std::vector<std::string> data_name) {
    cv::namedWindow("波形图");
    //cv::moveWindow("波形图", 1500, 200);

    //黑色背景
    coll = cv::Mat::zeros(canvas_h, canvas_w, CV_8UC3);
    std::vector<BGR> color = {{0, 165, 255},  {0, 200, 0}};
    //cv::RNG rng(time(0));
    //std::vector<BGR> color = {BGR(
    //                              rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)),  {0, 150, 0}
    //                         };

    cv::setMouseCallback("波形图", on_MouseHandle, (void*)&pro_offset);

    maxData = 1;
    for (uint i = 0; i != data.size(); i++) {
        for(int j = 0; j <= data_q_h && j < canvas_w; j++) {
            maxData = _max(fabs(data_q[i][j]), maxData);
        }
    }

    //缩放到合适的范围
    factor = 10;
    if(10 * maxData > canvas_h / 2) {
        factor = (canvas_h / 2 - 10) / maxData;
    }
    if(10 * maxData < 150 && maxData != 0) {
        factor = 150.0 / maxData;
    }

    setCanvas();

    for (uint i = 0,rect_y = 25, point_y = 45; i != data.size(); i++, rect_y += 30, point_y += 30) {
        dataCollect(data[i], color[i], i);
        rectangle(coll, cv::Rect(30, rect_y, 20, 20), color[i], -1, 4);
        cv::putText(coll, data_name[i], cv::Point(60, point_y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar::all(255), 1, 2);
    }

    data_q_h++;

    cv::imshow("波形图", coll);
}

void DataPlot::setCanvas() {
    int color_gray = 100;

    for(int y = 0; y < canvas_h; y += 50) {
        //横线
        cv::line(coll, cv::Point(pro_offset.offset_t, y + pro_offset.offset_v),
                 cv::Point(canvas_w + pro_offset.offset_t, y + pro_offset.offset_v),
                 BGR(color_gray, color_gray, color_gray));
        //纵坐标
        putText(coll, std::to_string(y - canvas_h / 2),
                cv::Point(canvas_w - 50 + pro_offset.offset_t, canvas_h - y + pro_offset.offset_v),
                1, 1, BGR(
                    color_gray, color_gray, color_gray));
    }

    for(int x = 0; x < canvas_w; x += 50) {
        //纵线
        cv::line(coll, cv::Point(x + pro_offset.offset_t, pro_offset.offset_v),
                 cv::Point(x + pro_offset.offset_t, canvas_h + pro_offset.offset_v),
                 BGR(color_gray, color_gray, color_gray));
        //横坐标
        putText(coll, std::to_string(x),
                cv::Point(canvas_w - x + pro_offset.offset_t, canvas_h - 50 + pro_offset.offset_v),
                1, 1, BGR(
                    color_gray, color_gray, color_gray));
    }
}


void DataPlot::on_MouseHandle(int event, int x, int y, int flags, void* param) {
    ProOffset_t *offset = (struct ProOffset *)param;
    switch(event) {
    //鼠标移动消息
    case cv::EVENT_MOUSEMOVE: {
        if(is_changeOffset) { //如果是否进行绘制的标识符为真，则记录下长和宽到RECT型变量中
            //设置偏移
            offset->offset_v = offset_v_tmp + y - yold;
            offset->offset_t = offset_t_tmp + x - xold;
        }
    }
    break;

    //左键按下消息
    case cv::EVENT_LBUTTONDOWN: {
        is_changeOffset = true;
        xold = x;
        yold = y;
    }
    break;

    //左键抬起消息
    case cv::EVENT_LBUTTONUP: {
        is_changeOffset = false;
        offset_v_tmp = offset->offset_v;
        offset_t_tmp = offset->offset_t;
    }
    break;
    }
}

void DataPlot::dataCollect(float data, BGR color, int data_q_index) {

    data_q_h = data_q_h % canvas_w;
    data_q[data_q_index][data_q_h] = data;

    cv::putText(coll, std::to_string(data), cv::Point(canvas_w - 60 + pro_offset.offset_t,
                canvas_h / 2 - factor * data + pro_offset.offset_v), 1, 1, color);

    //将 data_q 中的所有数据（点）连接起来，画出
    int p_data;
    for(int j = 0; j < canvas_w - 1; j++) {
        p_data = data_q_h - j + canvas_w;
        //第2个参数：前一个点（后一个点的数据位置）Point(x,y) x是时间轴，从前各往后 y是数据的大小
        //第3个参数：当前点（当前的数据位置）
        cv::line(coll, cv::Point((canvas_w - j - 1) + pro_offset.offset_t,
                                 canvas_h / 2 - factor * data_q[data_q_index][(p_data - 1) % canvas_w] * pro_offset.proportion_v + pro_offset.offset_v),
                 cv::Point((canvas_w - j) + pro_offset.offset_t,
                           canvas_h / 2 - factor *  data_q[data_q_index][p_data % canvas_w] * pro_offset.proportion_v + pro_offset.offset_v),
                 color);
    }
}
