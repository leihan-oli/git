#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "Types.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>

class Config {
public:
    Config();
    explicit Config(const std::string& filename);  // 添加带文件名参数的构造函数
    ~Config();

    bool loadFromFile(const std::string& filename);  // 从文件加载配置
    bool validate() const;                   // 参数合法性检查
    void print() const;                      // 打印配置内容

    const RTStitching::ConfigParams& getParams() const { return params_; }

private:
    RTStitching::ConfigParams params_;

    static cv::Mat readMat(const YAML::Node& node, int rows, int cols);
};

#endif // CONFIG_HPP
