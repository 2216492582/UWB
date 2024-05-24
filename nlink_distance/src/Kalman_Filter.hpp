#include <iostream>
#include <vector>

class KalmanFilter {
public:
    // 构造函数，用于初始化卡尔曼滤波器参数
    KalmanFilter(double processNoise, double measurementNoise, double estimatedError, double initialValue)
        : Q(processNoise), R(measurementNoise), P(estimatedError), X(initialValue) {}

    // 过滤函数，用于输入测量值并输出滤波后的值
    double filter(double measurement) {
        // 预测更新
        P = P + Q;

        // 计算卡尔曼增益
        K = P / (P + R);

        // 更新估计值
        X = X + K * (measurement - X);

        // 更新估计误差
        P = (1 - K) * P;

        return X;
    }

    // 重置函数，允许重新初始化滤波器
    void reset(double processNoise, double measurementNoise, double estimatedError, double initialValue) {
        Q = processNoise;
        R = measurementNoise;
        P = estimatedError;
        X = initialValue;
    }

private:
    double Q; // 过程噪声协方差
    double R; // 测量噪声协方差
    double P; // 估计误差协方差
    double K; // 卡尔曼增益
    double X; // 当前估计值
};

// // 示例用法
// int main() {
//     // 创建卡尔曼滤波器对象，并设置初始参数
//     KalmanFilter kf(0.1, 0.1, 0.1, 0.0);

//     // 示例测量数据
//     std::vector<double> raw_data = {1.0, 2.1, 3.2, 4.0, 5.1, 6.2, 7.3, 8.0, 9.1, 10.0};

//     // 过滤并输出数据
//     for (double val : raw_data) {
//         double smooth_val = kf.filter(val);
//         std::cout << "Raw: " << val << ", Smooth: " << smooth_val << std::endl;
//     }

//     return 0;
// }