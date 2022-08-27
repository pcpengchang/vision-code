#include "filter.h"

firstKalman::firstKalman() {
    Q_ = 0.01f;
    R_ = 0.02f;
    t_ = 1.0f;
    x_ = 0.0f;
    p_ = 0.01f;
}

firstKalman::firstKalman(float Q, float R, float t, float x0, float p0) {
    Q_ = Q;
    R_ = R;
    t_ = t;
    x_ = x0;
    p_ = p0;
}

float firstKalman::run(float _data) {
    x_pre_ = x_;                               //  x(k|k-1) = AX(k-1|k-1)+BU(k)
    p_pre_ = p_ + Q_;                          //  p(k|k-1) = Ap(k-1|k-1)A'+Q
    kg_    = p_pre_ / (p_pre_ + R_);           //  kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
    x_     = x_pre_ + kg_ * (_data - x_pre_);  //  x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p_     = (1 - kg_) * p_pre_;               //  p(k|k)   = (I-kg(k)H)P(k|k-1)

    return x_;
}

float firstKalman::mergeRun(float _data1, float _data2) {
    x_pre_ = _data1;
    p_pre_ = p_ + Q_;                           //  p(k|k-1) = Ap(k-1|k-1)A'+Q
    kg_    = p_pre_ / (p_pre_ + R_);            //  kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
    x_     = x_pre_ + kg_ * (_data2 - x_pre_);  //  x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p_     = (1 - kg_) * p_pre_;                //  p(k|k)   = (I-kg(k)H)P(k|k-1)

    return x_;
}

Kalman::Kalman() {

    Eigen::Matrix<double, 6, 6> f;
    f.setIdentity();

    Eigen::DiagonalMatrix<double, 6> p;
    p.setIdentity();

    Eigen::DiagonalMatrix<double, 6> q;
    // q.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
    q.diagonal() << 10, 10, 10, 50, 50, 50;

    Eigen::Matrix<double, 3, 6> h;
    h.setIdentity();

    Eigen::DiagonalMatrix<double, 3> r;
    // r.diagonal() << 0.05, 0.05, 0.05;
    r.diagonal() << 1000, 1000, 1000;

    F = f;
    P = p;
    Q = q;
    H = h;
    R = r;
}

void Kalman::initFilter(Eigen::VectorXd& position) {
    X = position;
    for(int i = 0; i < init_count_threshold; i++)
        update();
}

void Kalman::update() {
    X = F * X;
    P = F * P * F.transpose() + Q;
    Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    X = X + K *(Z - H * X);
    P = (Eigen::MatrixXd::Identity(X.size(), X.size()) - K * H) * P;
}

cv::Point3f Kalman::predict(cv::Point3f position, int time_stamp) {

    Data& data = Data::getData();
    float x = position.x, y = position.y, z = position.z;
    Eigen::VectorXd inputX(6, 1);
    inputX << x, y, z, 0, 0, 0;

    if(Tool::getDistance3D(position, last_position) > target_change_threshold
            || data.tracker_state == TrackerState::DETECTING) {
        std::cout << "kf need init" << std::endl;
        initFilter(inputX);  // 更新当前观测量
    }

    F(0, 3) = F(1, 4) = F(2, 5) = (time_stamp - last_time) / 1000.f;

    Eigen::VectorXd h(3, 1);	//本次观测值
    h << x, y, z;
    Z = h;
    update();

    last_time = time_stamp;
    last_position = position;

    return cv::Point3f(X(0, 0), X(1, 0), X(2, 0));
}


CVKalman::CVKalman() {
    KF = cv::KalmanFilter(9, 3, 0);
    measurement = cv::Mat::zeros(3, 1, CV_32F);
    cv::setIdentity(KF.measurementMatrix);                          // 观测模型
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));     // 过程噪声 Q
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-3)); // 测量噪声 R
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));           // P后验误差估计协方差矩阵，初始化为单位阵
    float dt = 1.f / control_freq;
    dt = 0.015; // s
    KF.transitionMatrix = (cv::Mat_<float>(9, 9) <<                 // 状态转移矩阵
                           1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0, 0,
                           0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0,
                           0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt,
                           0, 0, 0, 1, 0, 0, dt, 0, 0,
                           0, 0, 0, 0, 1, 0, 0, dt, 0,
                           0, 0, 0, 0, 0, 1, 0, 0, dt,
                           0, 0, 0, 0, 0, 0, 1, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 1, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 1);
}

cv::Point3f CVKalman::predict(cv::Point3f position) {

    Data& data = Data::getData();
    cv::Mat prediction;
    float cur_dis = cv::norm(position - cv::Point3f(0, 0, 0));

    if(cv::norm(position - last_position) > target_change_threshold
            || data.tracker_state == TrackerState::DETECTING) {
        std::cout << "kf need init" << std::endl;
        last_position = position;
        initFilter(position);
        return cv::Point3f(prediction.at<float>(0),
                           prediction.at<float>(1),
                           prediction.at<float>(2));
    }

    measurement.at<float>(0) = position.x;
    measurement.at<float>(1) = position.y;
    measurement.at<float>(2) = position.z;

    KF.correct(measurement);
    prediction = KF.predict();

    last_position = position;

    int speed = data.mcu.speed_bullet;
    return cv::Point3f(prediction.at<float>(0) + predict_coe * cur_dis / speed * prediction.at<float>(3),
                       prediction.at<float>(1) + predict_coe * cur_dis / speed * prediction.at<float>(4),
                       prediction.at<float>(2) + predict_coe * cur_dis / speed * prediction.at<float>(5));
}

void CVKalman::initFilter(cv::Point3f position) {
    KF.statePost = (cv::Mat_<float>(9, 1) <<
                    position.x, position.y, position.z, 0, 0, 0, 0, 0, 0);
    KF.predict();
    measurement.at<float>(0) = position.x;
    measurement.at<float>(1) = position.y;
    measurement.at<float>(2) = position.z;

    for(int i = 0; i < init_count_threshold; i++) {
        KF.correct(measurement);
        KF.predict();
    }
}

void AdaptiveKalman::initFilter() {
    Xe = Xr;
    for(int i = 0; i < init_count_threshold; i++)
        update();
}

void AdaptiveKalman::update() {
    // predict
    ceres::Jet<double, 6> Xe_auto_jet[6];
    for(int i = 0; i < 6; i++) {
        Xe_auto_jet[i].a = Xe[i]; // 估计值Xe
        Xe_auto_jet[i].v[i] = 1;
    }
    ceres::Jet<double, 6> Xp_auto_jet_p[6];
    predict_tool(Xe_auto_jet, Xp_auto_jet_p);  // 线性模型预测先验结果
    for(int i = 0; i < 6; i++) {
        Xp[i] = Xp_auto_jet_p[i].a;
        F.block(i, 0, 1, 6) = Xp_auto_jet_p[i].v.transpose();
    } // 更新预测雅可比F，得到先验预测量Xp
    P_ = F * P * F.transpose() + Q; // 计算先验噪声协方差P_

    // update
    coor_tool(Xr.data(), Yr.data());
    ceres::Jet<double, 6> Xp_auto_jet_u[6];
    for(int i = 0; i < 6; i++) {
        Xp_auto_jet_u[i].a = Xp[i];
        Xp_auto_jet_u[i].v[i] = 1;
    }
    ceres::Jet<double, 6> Zp_auto_jet[3];
    coor_tool(Xp_auto_jet_u, Zp_auto_jet);
    for(int i = 0; i < 3; i++) {
        Zp[i] = Zp_auto_jet[i].a;
        //动态大小的 block 运算
        H.block(i, 0, 1, 6) = Zp_auto_jet[i].v.transpose();
    } // 更新观测雅可比H，得到预测观测量Zp

    // 更新卡尔曼增益K、估计量Xe、噪声协方差P
    K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
    Xe = Xp + K * (Yr - Zp);
    P = (MatrixXX::Identity() - K * H) * P_;

    //std::cout << "measure position : [" << Xr(0, 0) << "," << Xr(2, 0) << "," << Xr(4, 0) << "]\n";
}

Eigen::Vector3d AdaptiveKalman::predict(Eigen::Vector3d position, int time_stamp) {

    Data& data = Data::getData();

    predict_tool.dt = _min((time_stamp - last_time) / 1000.f, 1.f);

    //std::cout << predict_tool.dt<< std::endl;
    last_time = time_stamp;

    // 更新当前观测量
    Xr << position(0, 0), 0.0, position(1, 0), 0.0, position(2, 0), 0.0;

    if(std::fabs(Xr.norm() - Yr(2, 0)) > target_change_threshold
            || data.tracker_state == TrackerState::DETECTING) {
        std::cout << "Ekf need init" << std::endl;
        initFilter();
        Eigen::Vector3d p_pw{Xe(0, 0), Xe(2, 0), Xe(4, 0)};
        return p_pw;
    }

    update();

    int speed = data.mcu.speed_bullet;
    Eigen::Vector3d p_pw{ Xe(0, 0) + predict_coe * position.norm() / speed * Xe(1, 0),
                          Xe(2, 0) + predict_coe * position.norm() / speed * Xe(3, 0),
                          Xe(4, 0) + predict_coe * position.norm() / speed * Xe(5, 0)};
    return p_pw;
}

bool ParticleFilter::initFilter() {

    //  过程噪声矩阵
    process_noise_cov(0, 0) = process_noise;

    //  观测噪声矩阵
    observe_noise_cov(0, 0) = observe_noise;

    //初始化粒子矩阵及粒子权重
    matrix_particle = Eigen::MatrixXd::Zero(num_particle, 1);

    randomlizedGaussianColwise(matrix_particle);

    matrix_particle = 3 * Eigen::MatrixXd::Random(num_particle, 1);
    matrix_weights = Eigen::MatrixXd::Ones(num_particle, 1) / static_cast<float>(num_particle);
    is_ready = false;

    return true;
}

Eigen::VectorXd ParticleFilter::predict() {
    Eigen::VectorXd particles_weighted = matrix_particle.transpose() * matrix_weights;
    return particles_weighted;
}

bool ParticleFilter::update(Eigen::VectorXd measure) {
    Eigen::MatrixXd gaussian = Eigen::MatrixXd::Zero(num_particle, 1);
    Eigen::MatrixXd mat_measure = measure.replicate(1, num_particle).transpose();
    double err = ((measure - (matrix_particle.transpose() * matrix_weights)).norm());
    //std::cout << "num_particle " << num_particle << "  err " << err << std::endl;

    if (is_ready) {
        //序列重要性采样
        matrix_weights = Eigen::MatrixXd::Ones(num_particle, 1);
        //按照高斯分布概率密度函数曲线右半侧计算粒子权重
        for(int i = 0; i < matrix_particle.cols(); i++) {
            double sigma = observe_noise_cov(i, i);
            Eigen::MatrixXd weights_dist = (matrix_particle.col(i) - mat_measure.col(i)).rowwise().squaredNorm();
            Eigen::MatrixXd tmp = ((-(weights_dist / pow(sigma, 2)) / matrix_particle.cols()).array().exp() / (sqrt(CV_2PI) * sigma)).array();
            matrix_weights = matrix_weights.array() * tmp.array();
        }
        matrix_weights /= matrix_weights.sum();
        double n_eff = 1.0 / (matrix_weights.transpose() * matrix_weights).value();

        //std::cout << "有效粒子数: " << err << std::endl;
        //有效粒子数少于一定值时进行重采样,该值需在实际调试过程中修改
        //  if (n_eff < 1 || isnan(n_eff))
        //  if (n_eff < 0.5 * num_particle)
        if (err > observe_noise_cov(0, 0) || (n_eff < 0.5 * num_particle))
        {
            //std::cout << "res" << num_particle << std::endl;
            resample();
        }
    }
    else {
        matrix_particle += mat_measure;
        is_ready = true;
        return false;
    }
    return true;
}

bool ParticleFilter::randomlizedGaussianColwise(Eigen::MatrixXd &matrix) {
    std::random_device rd;
    std::default_random_engine e(rd());
    std::vector<std::normal_distribution<double>> normal_distribution_list;

    //假设各个变量不相关
    normal_distribution_list.emplace_back(std::normal_distribution<double>(0, process_noise_cov(0, 0)));

    for (int col = 0; col < matrix.cols(); col++) {
        for(int row = 0; row < matrix.rows(); row++) {
            double tmp = normal_distribution_list[col](e);
            matrix(row, col) = tmp;
        }
    }

    return true;
}


bool ParticleFilter::resample() {

    //重采样采用低方差采样,复杂度为O(N),较轮盘法的O(NlogN)更小
    std::random_device rd;
    std::default_random_engine e(rd());
    std::uniform_real_distribution<> random {0.0, 1.0 / num_particle};

    int i = 0;
    double c = matrix_weights(0,0);
    double r = random(e);
    Eigen::MatrixXd matrix_particle_tmp = matrix_particle;

    for (int m = 0; m < num_particle; m++) {
        double u = r + m * (1.0 / num_particle);
        //  当 u > c 不进行采样
        while (u > c) {
            i++;
            c += matrix_weights(i, 0);
        }
        matrix_particle_tmp.row(m) = matrix_particle.row(i);
    }
    Eigen::MatrixXd gaussian = Eigen::MatrixXd::Zero(num_particle, 1);
    randomlizedGaussianColwise(gaussian);
    matrix_particle = matrix_particle_tmp + gaussian;
    matrix_weights = Eigen::MatrixXd::Ones(num_particle, 1) / static_cast<float>(num_particle);
    return true;
}
