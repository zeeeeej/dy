#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "yolov5.h"

#include <filesystem>
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "photo.h"
#include "common.h"
#include <mutex>
#include "utils_biu.hpp"
#include <vector>


double getUptimeSeconds() {
    std::ifstream uptimeFile("/proc/uptime");
    double uptimeSeconds = 0;
    if (uptimeFile) {
        uptimeFile >> uptimeSeconds;
    }
    return uptimeSeconds;
}



// 清理旧 watchdog 进程组
void kill_old_watchdog() {
    std::string pidPath = "/userdata/watchdog.pid";
    if (std::filesystem::exists(pidPath)) {
        std::ifstream pidFile(pidPath);
        pid_t old_pid = 0;
        pidFile >> old_pid;
        pidFile.close();

        if (old_pid > 0) {
            pid_t pgid = getpgid(old_pid);
            if (pgid > 0) {
                std::cout << "[Watchdog] 杀掉旧进程组 PGID=" << pgid << std::endl;
                kill(-pgid, SIGKILL);
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
        }
    }
}


void start_watchdog() {
    kill_old_watchdog();
    char exePath[1024] = {0};
    ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
    if (len == -1) {
        std::cerr << "无法获取当前程序路径" << std::endl;
        return;
    }
    exePath[len] = '\0';
    std::string programPath = exePath;


    {
        std::ofstream pidFile("/userdata/main.pid");
        pidFile << getpid();
    }

    // 写 watchdog.sh
    std::string watchdogPath = "/userdata/watchdog.sh";
    std::ofstream script(watchdogPath);
    script << "#!/bin/sh\n";
    script << "echo $$ > /userdata/watchdog.pid\n";
    script << "while true; do\n";
    script << "  if [ -f /userdata/main.pid ]; then\n";
    script << "    pid=$(cat /userdata/main.pid)\n";
    script << "    if ! kill -0 \"$pid\" 2>/dev/null; then\n";
    script << "      echo \"检测到主程序挂了，重启中...\"\n";
    script << "      setsid " << programPath << " &\n";  // 用setsid脱离watchdog进程组
    script << "      echo $! > /userdata/main.pid\n";
    script << "    fi\n";
    script << "  else\n";
    script << "    echo \"未找到主程序 PID，尝试启动...\"\n";
    script << "    setsid " << programPath << " &\n";    // 用setsid脱离watchdog进程组
    script << "    echo $! > /userdata/main.pid\n";
    script << "  fi\n";
    script << "  sleep 10\n";
    script << "done\n";
    script.close();

   
    std::string chmodCmd = "chmod +x " + watchdogPath;
    system(chmodCmd.c_str());

    // fork启动watchdog进程
    pid_t pid = fork();
    if (pid == 0) {
        setpgid(0, 0);  // 子进程独立进程组
        execl("/bin/sh", "sh", watchdogPath.c_str(), (char*)nullptr);
        std::cerr << "watchdog启动失败 errno=" << errno << " 原因：" << strerror(errno) << std::endl;
        exit(1);
    }

    std::cout << "[Watchdog] 启动成功，PID=" << pid << std::endl;
}



bool moveFile(const std::string& srcPath, const std::string& dstFolder) {
    namespace fs = std::filesystem;

    if (!std::filesystem::exists(srcPath)) {
        std::cerr << "源文件不存在：" << srcPath << std::endl;
        return false;
    }

    if (!std::filesystem::exists(dstFolder)) {
        std::cerr << "目标文件夹不存在：" << dstFolder << std::endl;
        return false;
    }

    std::filesystem::path src(srcPath);
    std::filesystem::path dst = std::filesystem::path(dstFolder) / src.filename();

    std::error_code ec;
    std::filesystem::rename(src, dst, ec);
    if (ec) {
        // 如果是跨设备错误，做复制+删除
        if (ec == std::errc::cross_device_link) {
            // 复制文件
            std::filesystem::copy(src, dst, std::filesystem::copy_options::overwrite_existing, ec);
            if (ec) {
                std::cerr << "复制文件失败：" << ec.message() << std::endl;
                return false;
            }
            // 删除源文件
            std::filesystem::remove(src, ec);
            if (ec) {
                std::cerr << "删除源文件失败：" << ec.message() << std::endl;
                return false;
            }
            return true;
        } else {
            std::cerr << "移动失败：" << ec.message() << std::endl;
            return false;
        }
    }

    return true;
}




bool deleteFile(const std::string& filepath) {
    namespace fs = std::filesystem;
    std::error_code ec;
    bool result = fs::remove(filepath, ec);  // 删除文件，失败不会抛异常
    if (ec) {
        std::cerr << "删除失败：" << ec.message() << std::endl;
    }
    return result;
}


std::string findRknnFile(const std::string& folderPath) {
    for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".rknn") {
            return std::filesystem::absolute(entry.path()).string();  
        }
    }
    return "";  
}



bool compressImageToTargetSize(const std::string& inputPath,
                               const std::string& outputPath,
                               int targetSizeKB,
                               int minQuality,
                               int maxQuality) {
   
    std::error_code ec;
    auto fileSize = std::filesystem::file_size(inputPath, ec);
    if (!ec && fileSize / 1024 <= targetSizeKB) {
        std::cout << "原图已小于目标大小，直接复制: " << fileSize / 1024 << "KB\n";
        std::ifstream src(inputPath, std::ios::binary);
        std::ofstream dst(outputPath, std::ios::binary);
        dst << src.rdbuf();
        return true;
    }

    cv::Mat image = cv::imread(inputPath);
    if (image.empty()) {
        std::cerr << "读取图片失败: " << inputPath << std::endl;
        return false;
    }

    std::vector<uchar> buf;
    int quality = maxQuality;
    bool success = false;

    for (; quality >= minQuality; quality -= 5) {
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
        buf.clear();
        if (!cv::imencode(".jpg", image, buf, params)) {
            std::cerr << "图片编码失败\n";
            return false;
        }

        size_t sizeKB = buf.size() / 1024;
        std::cout << "当前质量: " << quality << ", 大小: " << sizeKB << "KB\n";

        if (sizeKB <= targetSizeKB) {
            success = true;
            break;
        }
    }

    if (!success) {
        std::cerr << "无法压缩到目标大小以内\n";
        return false;
    }

    std::ofstream ofs(outputPath, std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(buf.data()), buf.size());
    ofs.close();
    std::cout << "压缩成功，保存到: " << outputPath << std::endl;

    return true;
}








void createBlankImage(const std::string& filename) {
   
    cv::Mat image(6, 6, CV_8UC3, cv::Scalar(255, 255, 255));

    if (!cv::imwrite(filename, image)) {
        std::cerr << "保存图像失败: " << filename << std::endl;
    } else {
        std::cout << "图像已保存: " << filename << std::endl;
    }
}



void trim_folder_images(const std::filesystem::path& folder_path, size_t max_images) {
    std::vector<std::pair<std::filesystem::path, std::filesystem::file_time_type>> image_files;

    if (!std::filesystem::exists(folder_path)) {
        std::cerr << "路径不存在: " << folder_path << std::endl;
        return;
    }

    for (const auto& file : std::filesystem::directory_iterator(folder_path)) {
        if (file.is_regular_file()) {
            std::string ext = file.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (ext == ".jpg" || ext == ".jpeg" || ext == ".png") {
                try {
                    auto time = std::filesystem::last_write_time(file);
                    image_files.emplace_back(file.path(), time);
                } catch (const std::exception& e) {
                    std::cerr << "获取文件时间失败: " << file.path() << " - " << e.what() << std::endl;
                }
            }
        }
    }

    if (image_files.size() <= max_images) return;

    std::sort(image_files.begin(), image_files.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    size_t num_to_delete = image_files.size() - max_images;
    for (size_t i = 0; i < num_to_delete; ++i) {
        std::cout << "删除旧图片: " << image_files[i].first << std::endl;
        std::error_code ec;
        std::filesystem::remove(image_files[i].first, ec);
        if (ec) {
            std::cerr << "删除失败: " << image_files[i].first << " - " << ec.message() << std::endl;
        }
    }
}






bool isImageBlurry(const cv::Mat& image, double& variance_out) {
    if (image.empty()) {
        std::cerr << "图像为空，无法判断是否模糊" << std::endl;
        variance_out = 0.0;
        return true;
    }

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);

    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);

    double variance = stddev[0] * stddev[0];
    variance_out = variance;

    std::cout << "图像方差: " << variance << std::endl;

    return false; // 这里不做模糊判断，只提取方差
}




bool delete_folder_contents_only(const std::string& folder_path) {
    try {
        if (std::filesystem::exists(folder_path) && std::filesystem::is_directory(folder_path)) {
            std::uintmax_t count = 0;
            for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
                count += std::filesystem::remove_all(entry.path());
            }
            std::cout << "成功删除文件夹内内容，共删除了 " << count << " 个文件/文件夹" << std::endl;
            return true;
        } else {
            std::cerr << "路径不存在或不是文件夹：" << folder_path << std::endl;
            return false;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "删除文件夹内容时异常：" << e.what() << std::endl;
        return false;
    }
}


bool delete_specified_folder(const std::string& folder_path) {
    try {
        if (std::filesystem::exists(folder_path) && std::filesystem::is_directory(folder_path)) {
            std::uintmax_t count = std::filesystem::remove_all(folder_path);
            std::cout << "成功删除文件夹及内容，共删除了 " << count << " 个文件/文件夹" << std::endl;
            return true;
        } else {
            std::cerr << "路径不存在或不是文件夹：" << folder_path << std::endl;
            return false;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "删除文件夹异常：" << e.what() << std::endl;
        return false;
    }
}






std::string replace_folder_name_in_path(const std::string& path_str, 
                                       const std::string& old_name, 
                                       const std::string& new_name) {
    std::filesystem::path path(path_str);
    std::filesystem::path new_path;

    for (const auto& part : path) {
        if (part == old_name) {
            new_path /= new_name;
        } else {
            new_path /= part;
        }
    }

    return new_path.string();
}







bool copy_folder_to(const std::filesystem::path& src_folder, const std::filesystem::path& dst_root) {
    try {
        if (!std::filesystem::exists(src_folder) || !std::filesystem::is_directory(src_folder)) {
            std::cerr << "源文件夹不存在或不是目录: " << src_folder << std::endl;
            return false;
        }

        std::filesystem::path dst_folder = dst_root / src_folder.filename(); 
        std::filesystem::create_directories(dst_folder); 

        std::filesystem::copy(
            src_folder,
            dst_folder,
            std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing
        );

        std::cout << "复制成功：" << src_folder << " -> " << dst_folder << std::endl;
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "复制失败: " << e.what() << std::endl;
        return false;
    }
}







void delete_oldest_folders(const std::filesystem::path& parent_path, size_t max_folders) {
    std::vector<std::pair<std::filesystem::path, std::filesystem::file_time_type>> folders;

    // 遍历该目录下所有文件夹
    for (const auto& entry : std::filesystem::directory_iterator(parent_path)) {
        if (std::filesystem::is_directory(entry)) {
            try {
                auto time = std::filesystem::last_write_time(entry);  // 获取修改时间
                folders.emplace_back(entry.path(), time);
            } catch (const std::exception& e) {
                std::cerr << "Failed to get time for: " << entry.path() << " - " << e.what() << std::endl;
            }
        }
    }

    // 如果文件夹数量不超过最大值，直接返回
    if (folders.size() <= max_folders) return;

    // 按时间升序排列（越早的排在前面）
    std::sort(folders.begin(), folders.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });

    size_t folders_to_delete = folders.size() - max_folders;

    for (size_t i = 0; i < folders_to_delete; ++i) {
        std::cout << "Deleting folder: " << folders[i].first << std::endl;
        std::filesystem::remove_all(folders[i].first);  // 删除整个文件夹
    }
}




void resize_images_in_folder(const std::string& folder_path, int max_length) {
    for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
        if (entry.is_regular_file()) {
           std::string file_path = entry.path().string();
            std::string extension = entry.path().extension().string();

            // 支持的图片格式
            if (extension == ".jpg" || extension == ".png" || extension == ".bmp") {
                cv::Mat img = cv::imread(file_path);
                if (img.empty()) {
                    std::cerr << "无法读取图片: " << file_path << std::endl;
                    continue;
                }

                int width = img.cols;
                int height = img.rows;
                std::cout << "处理图片: " << file_path << " (原始尺寸: " << width << "x" << height << ")" << std::endl;
                int long_side = std::max(width, height);

                // 如果已经小于等于 max_length，则跳过
                if (long_side <= max_length) continue;

                // 计算缩放比例
                double scale = static_cast<double>(max_length) / long_side;
                int new_width = static_cast<int>(width * scale);
                int new_height = static_cast<int>(height * scale);

                cv::Mat resized;
                cv::resize(img, resized, cv::Size(new_width, new_height));

                // 覆盖保存
                if (!cv::imwrite(file_path, resized)) {
                    std::cerr << "保存失败: " << file_path << std::endl;
                } else {
                    std::cout << "处理完成: " << file_path << std::endl;
                }
            }
        }
    }
}



// std::string read_txt_file(const std::string& file_path) {
//     std::ifstream ifs(file_path);
//     if (!ifs.is_open()) {
//         std::cerr << "打开文件失败：" << file_path << std::endl;
//         return "";
//     }
    
//     std::stringstream buffer;
//     buffer << ifs.rdbuf();  // 读取整个文件内容到buffer
//     return buffer.str();    // 返回字符串
// }

std::string read_txt_file(const std::string& file_path) {
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        std::cerr << "打开文件失败：" << file_path << std::endl;
        return "";
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();  // 读取整个文件内容
    std::string content = buffer.str();

    // 删除所有换行符（\n 和 \r）
    content.erase(std::remove(content.begin(), content.end(), '\n'), content.end());
    content.erase(std::remove(content.begin(), content.end(), '\r'), content.end());

    return content;
}




void remove_folder_if_exists(const std::string& folder_path) {
   

    std::filesystem::path dir(folder_path);

    if (std::filesystem::exists(dir)) {
        if (std::filesystem::is_directory(dir)) {
            std::error_code ec;
            std::filesystem::remove_all(dir, ec);  // 删除整个目录及其内容
            if (ec) {
                std::cerr << "删除目录失败：" << ec.message() << std::endl;
            } else {
                std::cout << "已删除目录：" << folder_path << std::endl;
            }
        } else {
            std::cerr << "路径存在但不是目录：" << folder_path << std::endl;
        }
    } else {
        std::cout << "目录不存在，无需删除：" << folder_path << std::endl;
    }
}




void movePhotos(ThreadSafeSet<std::string>& photo_names, const std::string& dest_folder, ThreadSafeSet<std::string>& action_id_record) {
    std::string src_path_biu;
    while (photo_names.try_pop(src_path_biu)) {

        std::filesystem::path src(src_path_biu);
        std::filesystem::path dest = std::filesystem::path(dest_folder) / src.filename();
       
        std::error_code ec;
        std::filesystem::rename(src, dest, ec);  // 尝试移动文件
        if (!ec) {
            std::cout << "移动成功: " << src_path_biu << " -> " << dest.string() << std::endl;
            action_id_record.insert(dest_folder);
        } else {
            std::cerr << "移动失败: " << src_path_biu << " 错误: " << ec.message() << std::endl;
        }
    }

}


void clearFile(const std::string& filepath) {
    std::ofstream ofs(filepath, std::ofstream::trunc);  // trunc表示截断（清空）
    if (!ofs) {
        std::cerr << "无法打开文件: " << filepath << std::endl;
    }
    // 文件自动清空，析构时自动关闭
}


// void take_video(const std::string& save_path) {
//     std::cout << "🎥 正在拍摄视频，保存至：" << save_path << std::endl;
//     // 模拟耗时
//     std::this_thread::sleep_for(std::chrono::seconds(5));
// }

//常量应用参数

bool take_photo(int device_id, const std::string& save_path, const std::string& img_name) {
    if (save_path.empty() || img_name.empty()) {
        std::cerr << "错误：保存路径或文件名为空!" << std::endl;
        return false;
    }

    uint8_t id_buf[40]        = {0};
    uint8_t img_name_buf[40] = {0};
    uint8_t path_buf[40]     = {0};

    // 拷贝路径和文件名（确保不会溢出）
    std::strncpy((char*)img_name_buf, img_name.c_str(), sizeof(img_name_buf) - 1);
    std::strncpy((char*)path_buf, save_path.c_str(), sizeof(path_buf) - 1);

    int ret = qjy_take_photo(device_id, id_buf, img_name_buf, path_buf);

    if (ret == 0) {
        std::cout << "拍照成功，图像保存至: " << save_path << "/" << img_name << std::endl;
        return true;
    } else {
        std::cerr << "拍照失败，错误码: " << ret << std::endl;
        return false;
    }
}



// void ensure_path_exists(const std::string& dir_path) {
//     if (!std::filesystem::exists(dir_path)) {
//         std::cout << "路径不存在，正在创建：" << dir_path << std::endl;
//         std::filesystem::create_directories(dir_path);  // 递归创建目录
//     } else {
//         std::cout << "路径已存在：" << dir_path << std::endl;
//     }
// }


void create_empty_txt(const std::string& file_path) {
    // 提取目录路径
    std::filesystem::path path_obj(file_path);
    std::filesystem::path parent_dir = path_obj.parent_path();

    // 如果目录不存在则创建
    if (!std::filesystem::exists(parent_dir)) {
        std::cout << "目录不存在，创建中：" << parent_dir << std::endl;
        std::filesystem::create_directories(parent_dir);
    }

    // 创建空文件（覆盖写入空内容）
    std::ofstream file(file_path, std::ios::trunc); // trunc表示清空已有内容
    if (!file) {
        std::cerr << "创建文件失败：" << file_path << std::endl;
        return;
    }

    std::cout << "已成功创建空 txt 文件：" << file_path << std::endl;
}




void ensure_path_exists(const char* dir_path) {
    if (dir_path == nullptr) {
        std::cerr << "路径为空，无法创建目录" << std::endl;
        return;
    }
    std::filesystem::path path_obj(dir_path);

    if (!std::filesystem::exists(path_obj)) {
        std::cout << "路径不存在，正在创建：" << dir_path << std::endl;
        std::filesystem::create_directories(path_obj);  // 递归创建目录
    } else {
        std::cout << "路径已存在：" << dir_path << std::endl;
    }
}



bool is_valid_video(const std::filesystem::path& file_path, int recent_seconds) {
    // 1. 只检查常见视频后缀
    std::vector<std::string> exts = {".mp4", ".avi", ".mkv", ".mov"};
    if (std::find(exts.begin(), exts.end(), file_path.extension()) == exts.end())
        return false;

    // 2. 判断是否最近刚修改（比如正在写入中）
    auto ftime = std::filesystem::last_write_time(file_path);
    auto now = std::filesystem::file_time_type::clock::now();
    auto age = std::chrono::duration_cast<std::chrono::seconds>(now - ftime).count();

    if (age < recent_seconds) {
        std::cout << "可能正在录制中，跳过：" << file_path << std::endl;
        return false;
    }

    // 3. 可扩展用 ffprobe 检查完整性（下面再讲）
    return true;
}

std::vector<std::string> get_complete_videos(const std::string& dir_path) {
    std::vector<std::string> result;

    for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
        if (entry.is_regular_file()) {
            const auto& path = entry.path();
            if (is_valid_video(path)) {
                result.push_back(std::filesystem::absolute(path).string());
            }
        }
    }

    return result;
}




// std::atomic<bool> need_take_video(false);
// std::atomic<bool> video_done(false);  // 告诉别的线程：“拍完视频了！”
// std::atomic<bool> running(true);



// void watch_for_takevideo_signal() {
//     while (true) {
//         // 模拟判断是否要拍摄（你可以换成外部指令判断等）
//         if (need_take_video.load()) {
//             std::string path = "/mnt/videos/" + std::to_string(std::time(nullptr)) + ".mp4";
//             take_video(path);
//             need_take_video.store(false);  // 重置标志
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 检查频率
//     }
// }


float tly_detect1(int angle1) {
    // float detect_angle = (float)angle1;
    // float therold = 20.0f;
    // if (detect_angle >= therold) {
    //     return detect_angle;
    // }
    return float(angle1);
}

float tly_detect2(int angle1) {
    // float detect_angle = (float)angle1;
    // float therold = 20.0f;
    // if (detect_angle >= therold) {
    //     return detect_angle;
    // }
    return -float(angle1);
}

// void* worker(void* arg) {
//     while (running) {
//         tly_detect();
//         sleep(1);
//     }
//     return nullptr;
// }





// bool extract_frames(const std::string& video_path, const std::string& output_dir, int target_fps = 25) {
//     // 打开视频
//     cv::VideoCapture cap(video_path);
//     if (!cap.isOpened()) {
//         std::cerr << "无法打开视频: " << video_path << std::endl;
//         return false;
//     }

//     // 视频原始帧率
//     double original_fps = cap.get(cv::CAP_PROP_FPS);
//     if (original_fps <= 0) original_fps = 25;  // 防止获取失败

//     // 创建输出目录
//     std::filesystem::create_directories(output_dir);

//     // 帧间隔
//     int frame_interval = static_cast<int>(original_fps / target_fps);
//     if (frame_interval <= 0) frame_interval = 1;

//     cv::Mat frame;
//     int frame_count = 0;

//     while (cap.read(frame)) {
//         if (frame_count % frame_interval == 0) {
//             std::string filename = output_dir + "/" + std::to_string(frame_count) + ".jpg";
//             cv::imwrite(filename, frame);
//         }
//         frame_count++;
//     }
//     std::cout << "保存帧数: " << frame_count << " 到目录: " << output_dir << std::endl;
//     return true;
// }



std::vector<std::string> get_image_paths(const std::string& folder_path) {
    std::vector<std::string> image_paths;

    for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
        if (entry.is_regular_file()) {
            const auto& path = entry.path();
            // 支持的图片后缀（可按需扩展）
            std::string ext = path.extension().string();
            if (ext == ".jpg" || ext == ".png" || ext == ".jpeg" || ext == ".bmp") {
                image_paths.push_back(std::filesystem::absolute(path).string());
            }
        }
    }

    std::sort(image_paths.begin(), image_paths.end());

    return image_paths;
}


bool file_exists_and_not_empty(const std::string& filename) {
    std::ifstream file(filename, std::ios::ate | std::ios::binary); // 打开并跳到末尾

    if (!file.is_open()) {
        return false; // 不存在 或 无法打开
    }

    return file.tellg() > 0; // 文件大小大于0
}


float calculate_iou(const std::vector<float>& box1, const std::vector<float>& box2) {
    float x1 = box1[0], y1 = box1[1], x2 = box1[2], y2 = box1[3];
    float x1_2 = box2[0], y1_2 = box2[1], x2_2 = box2[2], y2_2 = box2[3];

    float inter_x1 = std::max(x1, x1_2);
    float inter_y1 = std::max(y1, y1_2);
    float inter_x2 = std::min(x2, x2_2);
    float inter_y2 = std::min(y2, y2_2);

    if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1) {
        return 0.0f;
    }

    float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
    float area1 = (x2 - x1) * (y2 - y1);
    float area2 = (x2_2 - x1_2) * (y2_2 - y1_2);

    return inter_area / (area1 + area2 - inter_area);
}

int count_nonzero_iou(const std::vector<float>& ious) {
    return std::count_if(ious.begin(), ious.end(), [](float iou) {
        return iou > 0.0f;
    });
}



std::vector<float> expand_box(const std::vector<float>& box, float scale) {
    float x_min = box[0];
    float y_min = box[1];
    float x_max = box[2];
    float y_max = box[3];

    float w = x_max - x_min;
    float h = y_max - y_min;
    float x_center = (x_min + x_max) / 2.0f;
    float y_center = (y_min + y_max) / 2.0f;

    float new_w = w * (1.0f + scale);
    float new_h = h * (1.0f + scale);

    float new_x_min = x_center - new_w / 2.0f;
    float new_y_min = y_center - new_h / 2.0f;
    float new_x_max = x_center + new_w / 2.0f;
    float new_y_max = y_center + new_h / 2.0f;

    return {new_x_min, new_y_min, new_x_max, new_y_max};
}


std::string analyse_one(const std::string& file_path) {
    std::vector<std::vector<float>> frames;
    std::string line;

    std::ifstream file(file_path);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<float> frame_data;
        float value;
        while (ss >> value) {
            frame_data.push_back(value);
        }
        frames.push_back(frame_data);
    }

    if (frames.empty()) return "";

    std::vector<float> first_frame = frames[0];
    std::vector<float> last_frame = frames[frames.size() - 1];

    std::vector<float> first_box(first_frame.begin() + 3, first_frame.end());
    std::vector<float> last_box(last_frame.begin() + 3, last_frame.end());

    float iou = calculate_iou(first_box, last_box);

    float first_center_x = (first_box[0] + first_box[2]) / 2;
    float last_center_x = (last_box[0] + last_box[2]) / 2;

    std::string result_print;

    if (first_center_x < last_center_x && iou == 0) {
        std::cout << "送入饮料" << std::endl;
        result_print = "送入饮料";
    } else if (first_center_x > last_center_x && iou == 0) {
        std::cout << "拿出饮料" << std::endl;
        result_print = "拿出饮料";
    } else if (iou != 0) {
        std::cout << "拿了又放回饮料" << std::endl;
        result_print = "拿了又放回饮料";
    }

    return result_print;
}




std::string analyse_two(const std::string& file_path) {
    std::vector<std::vector<float>> frames;
    std::string line;

    std::ifstream file(file_path);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<float> frame_data;
        float value;
        while (ss >> value) {
            frame_data.push_back(value);
        }
        frames.push_back(frame_data);
    }

    if (frames.empty()) return "";

    std::vector<float> first_frame1 = frames[0];
    std::vector<float> first_frame2 = frames[1];

    std::vector<float> last_frame1 = frames[frames.size() - 1];
    std::vector<float> last_frame2 = frames[frames.size() - 2];



    std::vector<float> last_frame1_box(last_frame1.begin() + 3, last_frame1.end());
    std::vector<float> last_frame2_box(last_frame2.begin() + 3, last_frame2.end());

    std::vector<float> first_frame1_box(first_frame1.begin() + 3, first_frame1.end());
    std::vector<float> first_frame2_box(first_frame2.begin() + 3, first_frame2.end());

    float iou1 = calculate_iou(last_frame1_box, first_frame1_box);
    float iou2 = calculate_iou(last_frame1_box, first_frame2_box);
    float iou3 = calculate_iou(last_frame2_box, first_frame1_box);
    float iou4 = calculate_iou(last_frame2_box, first_frame2_box);

    std::vector<float> ious_biu = {
        iou1,
        iou2,
        iou3,
        iou4
    };

    int count_biu = count_nonzero_iou(ious_biu);

    std::vector<float> last_frame1_box_expanded = expand_box(last_frame1_box, 0.2f);
    std::vector<float> last_frame2_box_expanded = expand_box(last_frame2_box, 0.2f);

    float iou = calculate_iou(last_frame1_box_expanded, last_frame2_box_expanded);

    std::string result_print;

    if (iou != 0) {
        std::cout << "拿了两瓶饮料" << std::endl;
        result_print = "拿了两瓶饮料";
    } else if (iou == 0) {
       if (count_biu == 2){
        std::cout << "拿了一瓶饮料" << std::endl;
       }
       else {
        std::cout << "拿了两瓶饮料" << std::endl;
       }
    } 

    return result_print;
}


bool delete_txt_file(const std::string& file_path) {
    std::error_code ec;
    if (std::filesystem::remove(file_path, ec)) {
        std::cout << "文件已删除: " << file_path << std::endl;
        return true;
    } else {
        std::cerr << "删除失败: " << ec.message() << std::endl;
        return false;
    }
}



void keepTopSharpImages(const std::string& save_dir, size_t keep_top_n = 5) {
    struct ImageInfo {
        std::string path;
        double variance;
    };

    std::vector<ImageInfo> image_infos;

    for (const auto& entry : std::filesystem::directory_iterator(save_dir)) {
        if (!entry.is_regular_file()) continue;

        std::string file_path = entry.path().string();
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext != ".jpg" && ext != ".png" && ext != ".jpeg" && ext != ".bmp") continue;

        cv::Mat img = cv::imread(file_path);
        double variance = 0.0;
        std::cout << "检查图像: " << file_path << std::endl;

        if (!img.empty()) {
            isImageBlurry(img, variance);
            image_infos.push_back({file_path, variance});
        } else {
            std::cerr << "读取图像失败: " << file_path << std::endl;
        }
    }

    if (image_infos.empty()) {
        std::cout << "没有找到图像文件，跳过处理" << std::endl;
        return;
    }

    // 按照清晰度从高到低排序
    std::sort(image_infos.begin(), image_infos.end(), [](const ImageInfo& a, const ImageInfo& b) {
        return a.variance > b.variance;
    });

    // 保留前 keep_top_n 张，其余删除
    for (size_t i = keep_top_n; i < image_infos.size(); ++i) {
        std::cout << "删除模糊图像: " << image_infos[i].path << " (方差: " << image_infos[i].variance << ")" << std::endl;
        std::filesystem::remove(image_infos[i].path);
    }

    std::cout << "清晰度最高的前 " << std::min(keep_top_n, image_infos.size()) << " 张图像已保留" << std::endl;
}






bool process_last_n_lines(const std::string& txt_path, const std::string& save_dir, int keep_last_n, int& pic_id) {
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

    // while ((int)lines.size() < keep_last_n) {
    //     lines.push_back(lines.back());
    // }

    std::filesystem::create_directories(save_dir);  // 创建保存目录

    int saved_count = 0;
    int total_lines = lines.size();

    std::filesystem::path file_path_biu;
    // for (int i = total_lines - keep_last_n; i < total_lines; ++i) 
    for (int i = 0; i < total_lines; ++i){
        std::istringstream iss(lines[i]);
        std::string img_path;
        
        int cls_id, x1, y1, x2, y2;
        if (!(iss >> img_path >> cls_id >> x1 >> y1 >> x2 >> y2)) {
            std::cerr << "格式错误，跳过: " << lines[i] << std::endl;
            continue;
        }
        std::cout <<img_path << " " << cls_id << " " << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
        file_path_biu = std::filesystem::path(img_path);

        cv::Mat img = cv::imread(img_path);
        if (img.empty()) {
            std::cerr << "读取图像失败: " << img_path << std::endl;
            continue;
        }

        x1 = std::max(0, x1); y1 = std::max(0, y1);
        x2 = std::min(img.cols - 1, x2);
        y2 = std::min(img.rows - 1, y2);
        if (x2 <= x1 || y2 <= y1) {
            std::cerr << "无效坐标: " << lines[i] << std::endl;
            continue;
        }

        cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
        cv::Mat cropped = img(roi);

        std::string img_name = std::filesystem::path(img_path).stem().string();

        pic_id = (pic_id < 240) ? (pic_id + 1) : 65;
        
        // std::string save_path = save_dir + "/" + img_name + "_cls" + std::to_string(cls_id) + "_crop_" + std::to_string(i) + ".jpg";
        std::string save_path = save_dir + "/" + std::to_string(i) + "_" + img_name + "_" + std::to_string(pic_id) + ".jpg";

        if (cv::imwrite(save_path, cropped)) {
            std::cout << "截图完成: " << save_path << std::endl;
            ++saved_count;
        } else {
            std::cerr << "保存失败: " << save_path << std::endl;
        }
    }

    keepTopSharpImages(save_dir, 5);  

    std::cout << "处理完成。" << std::endl;

    std::string original_path = replace_folder_name_in_path(file_path_biu, "images_oridinal_dir_path", "images_dir_path");   //***4444444 */
    
    std::filesystem::path original_path1(original_path);

    delete_specified_folder(file_path_biu.parent_path().string());

    delete_specified_folder(original_path1.parent_path().string());

    delete_txt_file(txt_path);

    return saved_count > 0;
}

bool is_folder_empty(const std::filesystem::path& folder_path) {
    try {
        if (!std::filesystem::exists(folder_path) || !std::filesystem::is_directory(folder_path)) {
            return true; // 不存在或不是目录，视为“空”
        }

        return std::filesystem::directory_iterator(folder_path) == std::filesystem::directory_iterator();
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "检查文件夹时出错: " << e.what() << std::endl;
        return true; // 出错也当空处理
    }
}







