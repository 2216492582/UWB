#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>

// 定义一个函数来读取文件并提取值
int getQValueFromFile(const std::string& filename, const std::string& key) {
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    int value = 0;
    bool found = false;

    // 逐行读取文件内容
    while (std::getline(file, line)) {
        std::size_t pos = line.find(key);
        if (pos != std::string::npos) {
            // 找到包含 key 的行，提取其值
            std::istringstream iss(line.substr(pos + key.length()));
            iss >> value;
            found = true;
            break;
        }
    }

    file.close();

    if (!found) {
        throw std::runtime_error("The key '" + key + "' was not found in the file.");
    }

    return value;
}
