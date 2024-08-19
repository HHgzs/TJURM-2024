// Reference:
// https://blog.csdn.net/qq_39568245/article/details/115312690
// github: nlohmann json

#include "data_manager/param.h"

using nlohmann::json;
using std::ifstream;
using std::ofstream;
using std::string;

// 加载json文件
bool Param::load(const string& path) {
    ifstream input_json(path);
    input_json >> params_;
    return !params_.is_null();
}

// 保存为json文件
void Param::dump(const std::string& path) {
    ofstream output_json(path);
    output_json << params_;
}

// 重载[]运算符，使用方法类似python字典
json& Param::operator[](const std::string& key) {
    return params_.at(key);
}

// 将Mat转化为json对象
void Param::to_json(json& json_info, const cv::Mat& mat_info) {
    // 获取矩阵的行数，列数和通道数
    int rows = mat_info.rows;
    int cols = mat_info.cols;
    int channel = mat_info.channels();

    // 将矩阵重塑为单通道32位浮点数类型
    cv::Mat array = mat_info.reshape(1, 1);
    cv::Mat float_array;
    array.convertTo(float_array, CV_32FC1);

    json_info = { { "rows", rows },
                  { "cols", cols },
                  { "channel", channel },
                  { "data", (std::vector<float>)float_array } };
}

// 将json对象转化为Mat
void Param::from_json(const json& json_info, cv::Mat& mat_info) {
    int rows = json_info["rows"];
    int cols = json_info["cols"];
    int channel = json_info["channel"];
    std::vector<float> data = json_info.at("data");
    mat_info = cv::Mat(data).reshape(channel, rows).clone();
}
