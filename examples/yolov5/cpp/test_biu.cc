#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>


bool process_last_n_lines(const std::string& txt_path, const std::string& save_dir, int keep_last_n) {
    std::ifstream infile(txt_path);
    if (!infile.is_open()) {
        std::cerr << "无法打开文件: " << txt_path << std::endl;
        return false;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(infile, line)) {
        if (!line.empty())
            lines.push_back(line);
    }
    infile.close();

    if (lines.empty()) {
        std::cout << "TXT文件为空，不处理" << std::endl;
        return false;
    }

    while ((int)lines.size() < keep_last_n) {
        lines.push_back(lines.back());
    }

    std::filesystem::create_directories(save_dir);  // 创建保存目录

    int saved_count = 0;
    int total_lines = lines.size();
    for (int i = total_lines - keep_last_n; i < total_lines; ++i) {
        std::istringstream iss(lines[i]);
        std::string img_path;
        int cls_id, x1, y1, x2, y2;
        if (!(iss >> img_path >> cls_id >> x1 >> y1 >> x2 >> y2)) {
            std::cerr << "格式错误，跳过: " << lines[i] << std::endl;
            continue;
        }

      

        x1 = std::max(0, x1); y1 = std::max(0, y1);
      

        std::string img_name = std::filesystem::path(img_path).stem().string();
        std::string save_path = save_dir + "/" + img_name + "_cls" + std::to_string(cls_id) + "_crop_" + std::to_string(i) + ".jpg";
        std::cout << save_path <<std::endl;
    }

    return saved_count > 0;
}

int main() {
    std::string txt_path  = "/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/test_file/action_id_1.txt";
    std::string save_dir  = "/mnt/c/Users/11198/Documents/RV1103_demo/test_demo/rknn_model_zoo-2.3.2/examples/yolov5/cpp/action_id";
    int keep_last_n = 5;
    process_last_n_lines(txt_path, save_dir, keep_last_n);
}