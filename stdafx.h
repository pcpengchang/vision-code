#ifndef STDAFX_H_INCLUDE
#define STDAFX_H_INCLUDE
#define FMT_HEADER_ONLY

#include <fmt/color.h>
#include <fmt/core.h>

// 编译优化
#pragma GCC optimize(2)
#pragma GCC optimize(3, "Ofast", "inline")

// ---------------------------------------------------------------------------//
//                              硬件设备选项
// ---------------------------------------------------------------------------//
// #define USE_SERIAL // 使用串口

// ---------------------------------------------------------------------------//
//                          FIXME:临场调试选项
// ---------------------------------------------------------------------------//
// #define SAVE_LOG   // 保存日志
// #define SAVE_VIDEO // 保存视频

// ---------------------------------------------------------------------------//
//                              图像DEBUG相关选项
// ---------------------------------------------------------------------------//
#define SHOW_IMG
// #define SHOW_ROI
// #define SHOW_COLOR
// #define SHOW_GRAY
#define SHOW_BINARY

// ---------------------------------------------------------------------------//
//                                调试选项
// ---------------------------------------------------------------------------//
// #define AIM_GRAVITY          // 重力补偿
#define TYPE_ARMOR_CLASSIFIER 0	// 数字识别分类器类型，0:SVM 1:MLP 2:Lenet
#define SHOW_VISION_DATA        // 视觉数据包输出
#define SHOW_DRAW_DATA          // 曲线绘制
#define SHOW_MCU_DATA           // 下位机数据包输出

#endif	//STDAFX_H_INCLUDE
