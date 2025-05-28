#ifndef UTILS_BIU_HPP
#define UTILS_BIU_HPP

#include <string>
#include <set>
#include <mutex>

void take_video(const std::string& save_path);

// void ensure_path_exists(const std::string& dir_path);
void ensure_path_exists(const char* dir_path);

bool is_valid_video(const std::filesystem::path& file_path, int recent_seconds = 10);

std::vector<std::string> get_complete_videos(const std::string& dir_path);

void watch_for_takevideo_signal();

float tly_detect(int angle1);



bool extract_frames(const std::string& video_path, const std::string& output_dir, int target_fps = 25);

std::vector<std::string> get_image_paths(const std::string& folder_path);

bool file_exists_and_not_empty(const std::string& filename);

float calculate_iou(const std::vector<float>& box1, const std::vector<float>& box2);

int count_nonzero_iou(const std::vector<float>& ious);

std::vector<float> expand_box(const std::vector<float>& box, float scale = 0.1f);

std::string analyse_one(const std::string& file_path);

std::string analyse_two(const std::string& file_path);


bool process_last_n_lines(const std::string& txt_path, const std::string& save_dir, int keep_last_n);

std::string read_txt_file(const std::string& file_path);

void clearFile(const std::string& filepath);

bool take_photo(int device_id, const std::string& save_path, const std::string& img_name);

void resize_images_in_folder(const std::string& folder_path, int max_length);

void create_empty_txt(const std::string& file_path);

void remove_folder_if_exists(const std::string& folder_path);

template <typename T>
class ThreadSafeSet {
    std::set<T> data_;
    std::mutex mutex_;
public:
    void insert(const T& val) {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.insert(val);
    }

    bool try_pop(T& val) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (data_.empty()) return false;
        val = *data_.begin();
        data_.erase(data_.begin());
        return true;
    }

    bool empty() {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.empty();
    }

    bool contains(const T& val) {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.count(val) > 0;
    }
};
void movePhotos(ThreadSafeSet<std::string>& photo_names, const std::string& dest_folder, ThreadSafeSet<std::string>& action_id_record);
void delete_oldest_folders(const std::filesystem::path& parent_path, size_t max_folders=10);

#endif