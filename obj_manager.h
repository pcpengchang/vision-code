#ifndef OBJ_MANGER_H
#define OBJ_MANGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cxxabi.h>
#include <mutex>
#include <unordered_map>

#include <iterator>
#include <thread>
#include <memory>
#include <deque>
#include <atomic>
#include <unistd.h>


/**
 * @brief 用于导出protect构造函数为public，方便使用std::make_shared创建共享对象
 */
template<class T>
class ExportPublicConstructor : public T {
public:
    template<class ...Ts>
    explicit ExportPublicConstructor(Ts &&...args) : T(std::forward<Ts>(args)...) {}
};

/**
 * @brief 命名共享对象管理器
 * @details 通过对象类型和对象名称唯一确定一个共享对象（即std::shared_ptr）
 *          此对象管理器不可用于非class的基本类型（即无法用于int，double等类型）
 *          该对象下的所有函数满足线程安全性
 * @tparam T 被管理的对象类型
 */
template<class T>
class ObjManager : public T {
public:
    using sptr = std::shared_ptr<T>;
    using wptr = std::weak_ptr<T>;

    /**
     * @brief 创建一个命名共享对象
     * @tparam Ts 对象的构造函数参数类型
     * @param name 对象的名称
     * @param args 对象的构造函数参数
     * @return 创建出的共享对象，如果该名称下已经存在一个共享对象，则返回nullptr
     */
    template<class ...Ts>
    static sptr create(const std::string &name, Ts &&...args) {
        std::unique_lock <std::mutex> lock(_mtx);
        if (_map.find(name) != _map.end())
            return nullptr;
        sptr p_obj = std::make_shared<ExportPublicConstructor<ObjManager<T>>>(name,
                     std::forward<Ts>(args)...);
        _map.emplace(name, p_obj);
        return p_obj;
    }

    /**
     * @brief 查找一个命名共享对象
     * @param name 对象的名称
     * @return 查找到的共享对象，如果该名称下不存在一个共享对象，则返回nullptr
     */
    static sptr find(const std::string &name) {
        std::unique_lock <std::mutex> lock(_mtx);
        auto iter = _map.find(name);
        if (iter == _map.end())
            return nullptr;
        else
            return iter->second.lock();
    }

    /**
     * @brief 查找一个命名共享对象，如果不存在则将其创建
     * @tparam Ts 对象的构造函数参数类型
     * @param name 对象的名称
     * @param args 对象的构造函数参数
     * @return 查找或创建出的共享对象
     */
    template<class ...Ts>
    static sptr bind(const std::string &name, Ts &&...args) {
        std::unique_lock <std::mutex> lock(_mtx);
        auto iter = _map.find(name);
        if (iter != _map.end()) {
            return iter->second.lock();
        }
        sptr p_obj = std::make_shared<ExportPublicConstructor<ObjManager<T>>>(name,
                     std::forward<Ts>(args)...);
        _map.emplace(name, p_obj);
        return p_obj;
    }

    /**
    * @brief 析构函数，将该对象从map中删除
    */
    ~ObjManager() {
        std::unique_lock <std::mutex> lock(_mtx);
        _map.erase(_name);
    }
protected:
    /**
     * @brief protect构造函数，无法直接创建该类型的对象
     * @tparam Ts 对象的构造函数参数类型
     * @param name 对象的名称
     * @param args 对象的构造函数参数
     */
    template<class ...Ts>
    explicit ObjManager(std::string name, Ts &&...args) : T(std::forward<Ts>(args)...), _name(std::move(name)) {}

private:
    // / 当前对象名称
    std::string _name;

    // / 全局map互斥锁
    static std::mutex _mtx;

    // / 对象map，用于查找命名对象
    static std::unordered_map<std::string, wptr> _map;

};
template<class T>
__attribute__((visibility("default")))
inline std::mutex ObjManager<T>::_mtx;

template<class T>
__attribute__((visibility("default")))
inline std::unordered_map<std::string, typename ObjManager<T>::wptr> ObjManager<T>::_map;

/**
 * @brief 消息异常类型
 */
class MessageError : public std::runtime_error {
protected:
    using std::runtime_error::runtime_error;
};

/**
 * @brief 消息异常类型，消息读取超时
 */
class MessageError_Timeout : public MessageError {
public:
    MessageError_Timeout() : MessageError("message read timeout!") {}
};

/**
 * @brief 消息类, 使用队列存储收到的消息，可以设置队列最大长度，当超出最大队列长度时，新消息会覆盖最老的消息
 * @param T 消息对象类型
 */
template <typename T>
class MsgFilter {
public:
    /**
     * @brief 设置队列长度
     * @param size 最大队列长度
     */
    explicit MsgFilter(size_t size = 1) {
        buffer_size = size;
    };

    ~MsgFilter() = default;

    /**
     * @brief 发布一条消息
     * @param message 待发布的消息
     * @param time_stamp 时间戳
     */
    bool publish(T &message, int time_stamp);

    /**
     * @brief 发布一条消息
     * @param message 待发布的消息
     */
    bool publish(T &message);

    /**
     * @brief 尝试获取一条消息
     * @param message 待获取的消息
     * @param time_stamp 时间戳
     */
    bool subscribe(T &message, int time_stamp);

    /**
     * @brief 尝试获取一条消息
     * @param message 待获取的消息
     */
    bool subscribe(T &product);

private:
    struct Message {
        T message;     // 消息
        int timestamp; // 时间戳
    };
    std::deque<T> buffer;                 //  队列缓冲区
    std::deque<Message> product_buffer;   //  队列缓冲区
    size_t buffer_size;                   //  最大消息长度
    mutable std::timed_mutex lock;        //  线程锁

protected:
    using Ms = std::chrono::milliseconds;
};

template <typename T>
bool MsgFilter<T>::publish(T &message, int time_stamp) {
    lock.lock();
    Message product = {message, time_stamp};
    if (product_buffer.size() < buffer_size)
        product_buffer.emplace_back(product);
    else {
        product_buffer.pop_front();
        product_buffer.emplace_back(product);
    }
    lock.unlock();

    return true;
}

template <typename T>
bool MsgFilter<T>::publish(T &message) {

    lock.lock();
    if (buffer.size() < buffer_size)
        buffer.emplace_back(message);
    else {
        buffer.pop_front();
        buffer.emplace_back(message);
    }
    lock.unlock();

    return true;
}

template <typename T>
bool MsgFilter<T>::subscribe(T &message) {

    //std::cout << buffer.size() << std::endl;
    while (true) {
        if(lock.try_lock_for(Ms(2))) {
            if (!buffer.empty()) {
                break;
            }
        }
        else {
            //throw MessageError_Timeout();
            fmt::print(fmt::fg(fmt::color::red), "[MsgFilter] failed to lock\n");
        }
        lock.unlock();
        //  throw MessageError_Timeout();
        usleep(1e3);
    }
    message = std::move(buffer.front());
    buffer.pop_front();
    lock.unlock();

    return true;
}

template <typename T>
bool MsgFilter<T>::subscribe(T &message, int time_stamp) {

    //std::cout << buffer.size() << std::endl;
    //队列为空时阻塞消费者
    while (true) {
        if(lock.try_lock_for(Ms(2))) {
            if (!product_buffer.empty()) {
                break;
            }
        }
        else {
            //throw MessageError_Timeout();
            fmt::print(fmt::fg(fmt::color::red), "[MsgFilter] failed to lock\n");
        }
        lock.unlock();
        usleep(1e3);
    }

    //int cnt = 0;
    //for (auto info : buffer)
    //    std::cout << cnt++ << " : " << info.timestamp << std::endl;
    auto it = std::lower_bound(product_buffer.begin(), product_buffer.end(), time_stamp, [](Message &prev, const int &timestamp) {
        return prev.timestamp < timestamp;
    });

    //if((*it).timestamp != timestamp)
    //    std::cout << (*it).timestamp << " : " << timestamp << " | " << buffer.size() << std::endl;

    if (it == product_buffer.end()) {
        //时间戳时间差大于10ms则认为该帧不可用, 无法同步
        if (abs((product_buffer.back().timestamp - time_stamp)) > 10) {
            // throw MessageError_Timeout();
            product_buffer.pop_front();
            lock.unlock();
            return false;
        }
        else {
            message = std::move((product_buffer.back()).message);
            product_buffer.pop_front();
            lock.unlock();
            return true;
        }
    }
    else {
        it--;
        message = std::move((*it).message);
        product_buffer.erase(it);
    }
    lock.unlock();
    //  std::cout << (*it).timestamp << ":" << timestamp << "|" << buffer.size() << std::endl;

    return true;
}

#endif //  OBJ_MANGER_H
