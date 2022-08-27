#ifndef RM_LOG_H
#define RM_LOG_H

#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <fstream>
#include <opencv2/opencv.hpp>

#define HINT std::cout << "<<<<<\t" + std::string(__FILE__) + ":" + std::to_string(__LINE__) + "\t>>>>>" + "\n";
//在任意行添加HINT会打印HINT所在文件以及行数，类似于断点

template<class T>
void outPut(T obj) {
    std::cout << obj << std::endl;
}

/**
 * @brief 日志系统，比赛时可用于保存重要信息
 */
class LOG {
public:
    std::ofstream stream;

    /**
     * @brief 单例对象
     */
    static LOG &instance() {
        static LOG logger;
        return logger;
    }

    /**
     * @brief 获得日志时间戳
     * @return 时间戳
     */
    static std::string getTime() {
        time_t now = time(0);
        tm *ltm = localtime(&now);
        return std::to_string(ltm->tm_hour) + ":" + std::to_string(ltm->tm_min) + ":" + std::to_string(ltm->tm_sec);
    }

    /**
     * @brief 设置日志存储位置和保存图片位置
     * @param logPath 日志存储位置
     */
    static void setDestination(const std::string &logPath) {
        std::cout << "Save log to : " << logPath << std::endl;

        //打开日志存储文件
        instance().stream.open(logPath, std::ios::out);
        time_t now = time(0);
        tm *ltm = localtime(&now);
        info("Now time : " + std::to_string(1900 + ltm->tm_year) + "." + std::to_string(1 + ltm->tm_mon) + "." + std::to_string(ltm->tm_mday) + "  " + std::to_string(ltm->tm_hour) + ":" + std::to_string(ltm->tm_min));
    }

    /**
     * @brief 关闭日志
     */
    static void close() {
        instance().stream.close();
    }

    /**
     * @brief 打印info日志
     * @param log 日志内容
     */
    static void info(const char *log) {
        std::string _log = "[INFO " + getTime() + "]: ";
        _log.append(log).append("\n");
        instance().stream << _log;
        std::cout << _log;
    }

    /**
     * @brief 打印info日志
     * @param log 日志内容
     */
    static void info(const std::string &log) {
        LOG::info(log.c_str());
    }
};

/**
 * @brief 文件操作
 */
class FileOperation {
public:

    FileOperation() = default;
    ~FileOperation() = default;

    /**
     * @brief 创建路径下的所有文件夹
     * @param folder 文件夹地址
     */
    static bool createDirectory(const std::string &folder) {
        std::string folderBuilder;
        std::string sub;
        sub.reserve(folder.size());
        for (auto it = folder.begin(); it != folder.end(); ++it) {
            const char c = *it;
            sub.push_back(c);
            if (c == '/' || it == folder.end() - 1) {
                folderBuilder.append(sub);
                if (0 != access(folderBuilder.c_str(), 0)) {
                    // this folder not exist
                    if (0 != mkdir(folderBuilder.c_str(), 0777)) {
                        // create failed
                        return false;
                    }
                }
                sub.clear();
            }
        }
        return true;
    }

    /**
     * @brief 保存图片
     * @param path 图片路径
     * @param number 图片编号-函数内部自增
     * @param img 要保存的图片
     * @param picture_format 图片格式
     */
    static void saveImg(const std::string &path, uint32_t &number, const cv::Mat &img, const std::string &picture_format) {
        if (!img.empty()) {
            cv::imwrite(path + std::to_string(number) + picture_format, img);
            number++;
        }
    }

    /**
     * 获取指定目录下的文件数量，从0开始扫描
     * @param path 文件路径
     * @param format 文件格式 如.avi .log .jpg .bmp
     * @return 文件数量
     */
    static uint32_t getFileSizeInOrder(const std::string &path, const std::string &format) {
        std::string filename;
        uint32_t number = 0;
        while (true) {
            filename = path + std::to_string(number) + format;
            std::fstream _file;
            _file.open(filename, std::ios::in);
            if (!_file) {
                break;
            }
            number++;
        }
        return number;
    }

};

#endif // RM_LOG_H
