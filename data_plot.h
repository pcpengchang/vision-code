#ifndef DATA_PLOT_H
#define DATA_PLOT_H

#include "utils.h"

typedef struct ProOffset {
    float proportion_v;   //  值比例
    float proportion_t;   //  时间比例
    int offset_v;         //  值偏移
    int offset_t;         //  时间偏移
} ProOffset_t;

class DataPlot {
public:
    DataPlot() = default;
    ~DataPlot() = default;
    DataPlot(int canvas_w = 600, int canvas_h = 600);

    /**
     * @brief 调用私有方法dataCollect，通过多次调用，传入多个数据，在一个窗口绘制该组数据的波形
     * @param data 显示的数据
     * @param data_name 数据名
     */
    void dataCollectProc(std::vector<float> data, std::vector<std::string> data_name);

    /**
     * @brief 设置画布
     */
    void setCanvas();

    /**
     * @brief 数据波形绘制
     * @param data 显示的数据
     * @param color 对应的颜色
     * @param data_q_index 当前数据写入的索引（数组下标）
     */
    void dataCollect(float data, BGR color, int data_q_index);

    void static on_MouseHandle(int event, int x, int y, int flags, void* param);

private:
    float maxData;
    float factor;             // 缩放系数
    int row_size_max;
    int col_size_max;
    float data_q[5][1200];    // 循环队列
    int data_q_h;             // 当前数据写入的索引（数组下标）
    int canvas_w;             // 画布宽度
    int canvas_h;             // 画布高度
    cv::Mat coll;             // 画布
    ProOffset pro_offset;
};


#endif //  DATA_PLOT_H
