#include "coord_solver.h"

CoordSolver::CoordSolver(Setting& setting) {
    intrinsic = setting.intrinsic;
    dis_coeff = setting.coeff;

    cv::cv2eigen(intrinsic, eigen_intrinsic);
    cv::cv2eigen(setting.t_cam_ptz, t_cam_ptz);
    cv::cv2eigen(setting.t_imu_cam, t_imu_cam);
    cv::cv2eigen(setting.t_cam_imu, t_cam_imu);
}

void CoordSolver::drawCoordinate(cv::Mat& rvec, cv::Mat& tvec) {
    Data& data = Data::getData();

    std::vector<cv::Point2f> img_points;
    static const std::vector <cv::Point3f> reference_obj = {
        {0.0, 0.0, 0.0},
        {0.1, 0.0, 0.0},
        {0.0, 0.1, 0.0},
        {0.0, 0.0, 0.1}
    };
    cv::projectPoints(reference_obj, rvec, tvec, intrinsic,
                      dis_coeff, img_points);

    //  绘制坐标系
    cv::line(data.img_show, img_points[0], img_points[1], {0, 0, 255}, 4);
    cv::line(data.img_show, img_points[0], img_points[2], {0, 255, 0}, 4);
    cv::line(data.img_show, img_points[0], img_points[3], {255, 0, 0}, 4);
}

Eigen::Vector3d CoordSolver::PNP() {
    Data& data = Data::getData();

    static const std::vector <cv::Point3d> pw_small = {  //  单位：m
        {-0.066, -0.027, 0.},
        {0.066,  -0.027, 0.},
        {0.066,  0.027,  0.},
        {-0.066, 0.027,  0.}
    };
    static const std::vector <cv::Point3d> pw_big = {    //  单位：m
        {-0.115, -0.029, 0.},
        {0.115,  -0.029, 0.},
        {0.115,  0.029,  0.},
        {-0.115, 0.029,  0.}
    };

    cv::Mat rvec, tvec;

    //迭代法只能用共面特征点求位置
    if(data.armor.type_armor == ArmorType::SMALL)
        cv::solvePnP(pw_small, data.armor.pts_2d, intrinsic, dis_coeff,
                     rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    else
        cv::solvePnP(pw_big, data.armor.pts_2d, intrinsic, dis_coeff,
                     rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    //drawCoordinate(rvec, tvec);
    Eigen::Vector3d tvec_eigen;
    cv2eigen(tvec, tvec_eigen);

    return tvec_eigen;

    //outPut(t_cam_ptz);
}

void CoordSolver::reProjectPoint(const Eigen::Vector3d &xyz) {
    Data& data = Data::getData();
    auto result = (1.f / xyz[2]) * eigen_intrinsic * (xyz);
    cv::circle(data.img_show, cv::Point2f(result[0], result[1]), 5, {0, 255, 0}, 2);
}

void CoordSolver::clearVisionData(VisionData& vision_data) {
    vision_data.yaw = 0;
    vision_data.pitch = 0;
    vision_data.dist = 0;
}

void CoordSolver::transAngle(double &x1, double &x2, double theta) {
    theta = Tool::angle2Radian(theta);
    double x = x1 * cos(theta) + x2 * sin(theta);
    double y = x2 * cos(theta) - x1 * sin(theta);
    x1 = x;
    x2 = y;
}

void CoordSolver::transAngle(double &x, double &y, double &z,
                             double ptz_pitch, double ptz_yaw) {

//    ptz_yaw = Tool::angle2Radian(ptz_yaw);
//    ptz_pitch = Tool::angle2Radian(ptz_pitch);
//    double absolute_x = x * cos(ptz_yaw) + y * sin(ptz_pitch) * sin(ptz_yaw) -
//                        z * cos(ptz_pitch) * sin(ptz_yaw);
//    double absolute_y = y * cos(ptz_pitch) + z * sin(ptz_pitch);
//    double absolute_z = x * sin(ptz_yaw) - y * sin(ptz_pitch) * cos(ptz_yaw) +
//                        z * cos(ptz_pitch) * cos(ptz_yaw);
//    x = absolute_x;
//    y = absolute_y;
//    z = absolute_z;
    transAngle(x, y, 0);
    transAngle(y, z, ptz_pitch);
    transAngle(x, z, ptz_yaw);
}

Eigen::Vector3d CoordSolver::camToWorld(const Eigen::Vector3d &point_camera, const Eigen::Matrix3d &rmat) {
    //升高维度
    Eigen::Vector4d point_camera_tmp;
    Eigen::Vector4d point_imu_tmp;
    Eigen::Vector3d point_imu;

    //  Pc = R * Pw + T
    point_camera_tmp << point_camera[0], point_camera[1], point_camera[2], 1; //nomalize

    point_imu_tmp = t_imu_cam * point_camera_tmp;
    point_imu << point_imu_tmp[0], point_imu_tmp[1], point_imu_tmp[2];

    return rmat * point_imu;
}

Eigen::Vector3d CoordSolver::worldToCam(const Eigen::Vector3d &point_world, const Eigen::Matrix3d &rmat) {

    Eigen::Vector4d point_camera_tmp;
    Eigen::Vector4d point_imu_tmp;
    Eigen::Vector3d point_imu;
    Eigen::Vector3d point_camera;

    point_imu = rmat.transpose() * point_world;

    point_imu_tmp << point_imu[0], point_imu[1], point_imu[2], 1;
    point_camera_tmp = t_cam_imu * point_imu_tmp;
    point_camera << point_camera_tmp[0], point_camera_tmp[1], point_camera_tmp[2];

    return point_camera;
}

double CoordSolver::getPitchOffset(Eigen::Vector3d &xyz, double depth) {

    Data& data = Data::getData();
    int speed_bullet = data.mcu.speed_bullet;
    speed_bullet = 15;

    //  水平深度
    if(depth != 0)
        xyz(2, 0) = depth;

    double t_2 = 0.f;
    double distance = xyz.norm();
    //std::cout << "distance: " << distance << std::endl;

    if(speed_bullet < 20) {
        t_2 = distance / speed_bullet;
        t_2 *= t_2;
    }
    else {
        double p_pitch = atan2(xyz(2, 0), xyz.topRows<2>().norm());
        //  抛物线计算
        double a = 9.8 * 9.8 * 0.25;

        double b = -speed_bullet * speed_bullet - distance * 9.8 * cos(M_PI_2 + p_pitch);
        double c = distance * distance;

        //  带入求根公式求解
        t_2 = (-sqrt(b * b - 4 * a * c) - b) / (2 * a);
    }

    //  解出抬枪高度，即子弹下坠高度
    double height = 0.5 * 9.8 * t_2;

    return height;
}

Eigen::Vector3d CoordSolver::wrongDataKiller(Eigen::Vector3d &point_world) {

    auto result = point_world;
    //存在非法坐标
    if (isnan(point_world(0, 0)) || isnan(point_world(1, 0)) || isnan(point_world(2, 0))) {
        fmt::print(fmt::fg(fmt::color::red), "[CoordSolver] point_world is nan!\n");
        return last_pw;
    }

    //剔除跳变
    bool bad_flag = false;		//坐标突变标志位
    if (bad_cnt < 5) {
        if (fabs(point_world(0, 0) - last_pw(0, 0)) > 1) {
            bad_flag = true;
            result(0, 0) = last_pw(0, 0);
        }
        // ......
    }
    //连续多次出现跳变数据，则相信该数据
    else {
        bad_cnt = 0;
        bad_flag = false;
    }

    bad_flag == true ? bad_cnt++ : bad_cnt = 0;
    last_pw = point_world;

    return result;
}

ArmorAngleSolver::ArmorAngleSolver(Setting& setting) {

    coord_solver = CoordSolver(setting);

    cv::FileStorage fs("../configs/filter_param.yml", cv::FileStorage::READ);

    // EKF设置
    // 预测过程协方差
    fs["Q00"] >> ekf.Q(0, 0);
    fs["Q11"] >> ekf.Q(1, 1);
    fs["Q22"] >> ekf.Q(2, 2);
    fs["Q33"] >> ekf.Q(3, 3);
    fs["Q44"] >> ekf.Q(4, 4);
    fs["Q55"] >> ekf.Q(5, 5);
    // 观测过程协方差
    fs["R00"] >> ekf.R(0, 0);
    fs["R11"] >> ekf.R(1, 1);
    fs["R22"] >> ekf.R(2, 2);

    fs["init_count_threshold"]    >> ekf.init_count_threshold;
    fs["target_change_threshold"] >> ekf.target_change_threshold;
    fs["ekf"]["predict_coe"]      >> ekf.predict_coe;

    // KF设置
    fs["init_count_threshold"]    >> cv_kf.init_count_threshold;
    fs["control_freq"]            >> cv_kf.control_freq;
    fs["target_change_threshold"] >> cv_kf.target_change_threshold;
    fs["kf"]["predict_coe"]       >> cv_kf.predict_coe;
    //kf.predict_coe = 0;

    fs.release();

    fmt::print(fmt::fg(fmt::color::green), "[coordSolver] Set param finished\n");
}

void ArmorAngleSolver::getAngle(VisionData& vision_data, int time_stamp) {
    Data& data = Data::getData();

    /// 掉帧补偿，保持云台运动的流畅性
    if(data.cnt_lost > 0 && data.cnt_lost < 50) {
        vision_data.yaw = last_yaw;
        vision_data.pitch = last_pitch;
        return;
    }
    else if(data.cnt_lost >= 50) {
        coord_solver.clearVisionData(vision_data);
        return;
    }

    //Timer t;
    /// 生成旋转矩阵
    Eigen::Matrix3d rmat_imu = data.quat.toRotationMatrix();
    rmat_imu = Eigen::Matrix3d::Identity();

    /// pnp解算  point camera: 目标在相机坐标系下的坐标
    Eigen::Vector3d m_pc = coord_solver.PNP();

    /// 通过云台角解算惯性坐标  point world: 目标在惯性坐标系下的坐标
    /*
    s_pc += coord_solver.t_cam_ptz;
    CoordSolver::transAngle(m_pc(0, 0), m_pc(1, 0), m_pc(2, 0),
                            data.euler[0], data.euler[1]);
    Eigen::Vector3d m_pw{m_pc(0, 0), m_pc(0, 0), m_pc(0, 0)};
    */

    /// 通过四元数解算惯性坐标  point world: 目标在惯性坐标系下的坐标
    Eigen::Vector3d m_pw = coord_solver.camToWorld(m_pc, rmat_imu);

    /// 反陀螺模式
    //if(data.armor.is_anti_spin)
    //    getTopAngle(vision_data);

    /// 坐标拟合
    //curveFitting(m_pw, time_stamp);

    /// EKF预测
    Eigen::Vector3d p_pw = m_pw;
    p_pw = ekf.predict(p_pw, time_stamp);
    // 预测点
    Eigen::Vector3d s_pw{p_pw(0, 0), p_pw(1, 0), p_pw(2, 0)};

    /// KF预测
    /*
    cv::Point3f p_pw = {(float)m_pw(0, 0), (float)m_pw(1, 0), (float)m_pw(2, 0)};
    p_pw = kf.predict(p_pw, time_stamp);
    //p_pw = cv_kf.predict(p_pw);
    // 预测点
    Eigen::Vector3d s_pw{p_pw.x, p_pw.y, p_pw.z};
    */

#ifdef AIM_GRAVITY
    s_pw(0, 0) -= coord_solver.getPitchOffset(s_pw, 0);
#endif // 惯性坐标重力补偿

    /// 转为相机坐标系
    Eigen::Vector3d s_pc = coord_solver.worldToCam(s_pw, rmat_imu);
    /// 转为云台坐标系  point ptz: 目标在云台坐标系下的坐标(枪管出口为原点)
    s_pc += coord_solver.t_cam_ptz;

#ifdef AIM_GRAVITY
    s_pc(1, 0) -= coord_solver.getPitchOffset(s_pc, 0);
#endif // 云台坐标重力补偿

    /// 反投影绘制
    coord_solver.reProjectPoint(s_pc);

    /// 计算yaw pitch dist
    vision_data.yaw = Tool::radian2Angle(atan2f(s_pc[0], s_pc[2]));
    vision_data.pitch = -Tool::radian2Angle(atan2f(m_pc[1], sqrt(m_pc[0] * m_pc[0] + m_pc[2] * m_pc[2])));
    vision_data.dist = s_pw.norm();

    data.data_1 = m_pw(0, 0);
    data.data_2 = p_pw(0, 0);
    last_time_stamp = time_stamp;
    last_yaw = vision_data.yaw;
    last_pitch = vision_data.pitch;
    //t._end("getAngle");
}

bool ArmorAngleSolver::curveFitting(Eigen::Vector3d &result, int time_stamp) {

    Data& data = Data::getData();

    TargetInfo target = {result, time_stamp};
    history_info.push_back(target);
    if(history_info.size() < 10) {
        return false;
    }
    else if(target.timestamp - history_info.front().timestamp >= 1000) {
        history_info.pop_front();
        return false;
    }
    else {
        if(history_info.size() > 12) {
            history_info.pop_front();
        }
    }

    int speed_bullet = data.mcu.speed_bullet;
    speed_bullet = 15;
    auto time_estimated = result.norm() / speed_bullet * 1e3 + time_stamp + 100;

    double params_x[4] = {0, 0, 0, 0};     // 参数的估计值
    double params_y[4] = {0, 0, 0, 0};     // 参数的估计值

    ceres::Problem problem_x;
    ceres::Problem problem_y;
    ceres::Solver::Options options_x;
    ceres::Solver::Options options_y;
    ceres::Solver::Summary summary_x;   // 优化信息
    ceres::Solver::Summary summary_y;   // 优化信息
    options_x.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options_y.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解

    // std::cout << history_info.size() << std::endl;

    Eigen::Vector3d sum = {0, 0, 0};
    for (auto target_info : history_info)
        sum += target_info.xyz;
    auto dc = sum / history_info.size();
    // auto dc = history_info.at(history_info.size() - 1).xyz;
    params_x[0] = dc[0];
    params_y[0] = dc[1];

    for (auto target_info : history_info) {
        //cout<<"T : "<<target_info.timestamp / 1e3<<" X:"<<target_info.xyz[0]<<" Y:"<<target_info.xyz[1]<<endl;
        problem_x.AddResidualBlock (
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 2> (
                new CURVE_FITTING_COST (target_info.timestamp / 1e3, (target_info.xyz[0] - params_x[0]))
            ),
            new ceres::CauchyLoss(0.5),
            &params_x[1]
        );
        problem_y.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 2> (
                new CURVE_FITTING_COST (target_info.timestamp / 1e3, (target_info.xyz[1] - params_y[0]))
            ),
            new ceres::CauchyLoss(0.5),
            &params_y[1]
        );
    }

    // 异步计算
    auto status_solve_x = std::async(std::launch::deferred, [&]() {
        ceres::Solve(options_x, &problem_x, &summary_x);
    });
    auto status_solve_y = std::async(std::launch::deferred, [&]() {
        ceres::Solve(options_y, &problem_y, &summary_y);
    });

    status_solve_x.wait();
    status_solve_y.wait();

    double x_cost = summary_x.final_cost;
    double y_cost = summary_y.final_cost;

    std::cout << "x_cost " << x_cost << " y_cost " << y_cost << std::endl;

    if(x_cost <= 10 && y_cost <= 10) {
        double x_pred = params_x[0] + params_x[1] * (time_estimated / 1e3) + params_x[2] * pow((time_estimated / 1e3), 2);
        double y_pred = params_y[0] + params_y[1] * (time_estimated / 1e3) + params_y[2] * pow((time_estimated / 1e3), 2);
        result = {x_pred, y_pred, dc[2]};
        return true;
    }

    return false;
}

void ArmorAngleSolver::getTopAngle(VisionData& vision_data) {

    // //  TODO：在反陀螺模式下对速度做衰减因子
    float offset_plus = +1000; //mm
    float offset_mul = 1;

    float tar_window = 0;

    int flag = 0;

    if(vision_data.dist <= 2 * offset_mul + offset_plus) //  距离在0到2m内的反陀螺的窗口
        tar_window = 3.7465, flag = 2;                    //这些数据是测得的
    else if(vision_data.dist <= 3 * offset_mul + offset_plus)//  距离在2到3m内的反陀螺的窗口
        tar_window = 2.700, flag = 3;
    else if(vision_data.dist <= 4 * offset_mul + offset_plus)//  距离在3到4m内的反陀螺的窗口
        tar_window = 1.3633, flag = 4;
    else if(vision_data.dist <= 5 * offset_mul + offset_plus)//  距离在4到5m内的反陀螺的窗口
        tar_window = 0.3233, flag = 5;
    else //  5m以外，反陀螺效果与没反陀螺没差别
        return;

    //  限制窗口
    if(abs(vision_data.yaw) <= tar_window) { //theta,在范围内就不要动
        vision_data.yaw = 0;

    } else { //  在范围外，就将目标移至范围内
        vision_data.yaw = (vision_data.yaw > 0 ? vision_data.yaw - tar_window : vision_data.yaw + tar_window);
    }
}

void ArmorAngleSolver::getTopAngle2(Eigen::Vector3d &pw) {
    Data& data = Data::getData();

    float x_max = 0.f;
    float x_min = 0.f;

    if(data.armor.is_anti_spin) {
        std::sort(x_history.begin(), x_history.end());
        x_min = x_history[0];
        x_max = x_history[x_history.size() - 1];

        float x = pw(0, 0);
        float correct = 0.0;
        if (x > x_max) {
            correct = x - x_max;
        }
        else if (x < x_min) {
            correct = x - x_min;
        }
        x_min += correct;
        x_max += correct;
        pw(0, 0) = (x_max + x_min) / 2;

        y_history.push_back(pw(1, 0));
        z_history.push_back(pw(2, 0));

        //对Y进行平均滤波
        if (y_history.size() > 20) {
            pw(1, 0) = Tool::getMeanValue(y_history);
            y_history.erase(y_history.begin());
        }

        //对Z进行平均滤波
        if (z_history.size() > 20) {
            pw(2, 0) = Tool::getMeanValue(z_history);
            z_history.erase(z_history.begin());
        }
    }

    //小陀螺模式之前记录历史20个坐标
    x_history.push_back(pw(0, 0));
    y_history.push_back(pw(1, 0));
    z_history.push_back(pw(2, 0));

    if (x_history.size() > 20) {
        x_history.erase(x_history.begin());
    }

    if (y_history.size() > 20) {
        y_history.erase(y_history.begin());
    }

    if (z_history.size() > 20) {
        z_history.erase(z_history.begin());
    }

}

BuffAngleSolver::BuffAngleSolver(Setting& setting):
    h_buff(setting.h_buff),
    r_buff(setting.r_buff),
    dist_buff(setting.dist_buff),
    h_car(setting.h_car),
    h_platform(setting.h_platform) {
    coord_solver = CoordSolver(setting);
}

void BuffAngleSolver::getAngle(VisionData& vision_data) {

    Data& data = Data::getData();

    // 掉帧补偿，保持云台运动的流畅性
    if(data.cnt_lost > 0 && data.cnt_lost < 50) {
        coord_solver.clearVisionData(vision_data);
        return;
    }
    else if(data.cnt_lost >= 50) {
        fmt::print(fmt::fg(fmt::color::red), " 复位归中 \n\n");
        vision_data.yaw = -data.euler[0];
        vision_data.pitch = -data.euler[1];
        return;
    }

    data.armor.type_armor = ArmorType::BIG;

    ///  通过模型计算最终目标点的 depth
    //  枪管与能量机关最底部装甲板的竖直距离
    float h_delta = (h_buff - r_buff) - (h_platform + h_car);

    //  能量机关目标与最底部装甲板的竖直距离
    h_buff = r_buff * (1 + sin(data.buff.angle - CV_PI));

    //  枪管与能量机关目标的竖直距离
    float h_target = h_delta + h_buff;

    //  枪管与能量机关目标的三维距离
    double depth = sqrt(pow(h_target, 2) + pow(dist_buff, 2));

    //std::cout << "h_buff " << h_buff << std::endl;

    ///  pnp解算 point camera: 目标在相机坐标系下的坐标
    Eigen::Vector3d m_pc = coord_solver.PNP();

    ///  point ptz: 目标在云台坐标系下的坐标(枪管出口为原点)
    m_pc += coord_solver.t_cam_ptz;

    ///  重力补偿
    m_pc(1, 0) -= coord_solver.getPitchOffset(m_pc, depth);
    m_pc(2, 0) = depth;

    ///  计算yaw pitch dist
    vision_data.yaw = Tool::radian2Angle(atan2f(m_pc[0], m_pc[2]));
    vision_data.pitch = -Tool::radian2Angle(atan2f(m_pc[1], sqrt(m_pc[0] * m_pc[0] + m_pc[2] * m_pc[2])));
    vision_data.dist = m_pc.norm();

}
