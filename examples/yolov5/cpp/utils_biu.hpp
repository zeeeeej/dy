#ifndef UTILS_BIU_HPP
#define UTILS_BIU_HPP

#include <string>
#include <set>
#include <mutex>



template <typename T>
class ThreadSafeSet {
    std::set<T> data_;
    std::list<T> insertion_order_;  // 保留插入顺序
    std::mutex mutex_;
    size_t max_size_;

public:
    explicit ThreadSafeSet(size_t max_size) : max_size_(max_size) {}

    void insert(const T& val) {
        std::lock_guard<std::mutex> lock(mutex_);
      
        if (data_.count(val) > 0) return;

    
        if (data_.size() >= max_size_) {
            const T& oldest = insertion_order_.front();
            data_.erase(oldest);
            insertion_order_.pop_front();
        }


        data_.insert(val);
        insertion_order_.push_back(val);
    }

    bool try_pop(T& val) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (data_.empty()) return false;
        val = *data_.begin();
        data_.erase(val);

        insertion_order_.remove(val);
        return true;
    }

    bool empty() {
        std::lock_guard<std::mutex> lock(mutex_);
         return data_.empty() && insertion_order_.empty();
    }

    bool contains(const T& val) {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.count(val) > 0;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.clear();
        insertion_order_.clear();
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.size();
    }

    size_t capacity() const {
        return max_size_;
    }
};


void take_video(const std::string& save_path);

// void ensure_path_exists(const std::string& dir_path);
void ensure_path_exists(const char* dir_path);

bool is_valid_video(const std::filesystem::path& file_path, int recent_seconds = 10);

std::vector<std::string> get_complete_videos(const std::string& dir_path);

void watch_for_takevideo_signal();

float tly_detect1(int angle1);

float tly_detect2(int angle1);



bool extract_frames(const std::string& video_path, const std::string& output_dir, int target_fps = 25);

std::vector<std::string> get_image_paths(const std::string& folder_path);

bool file_exists_and_not_empty(const std::string& filename);

float calculate_iou(const std::vector<float>& box1, const std::vector<float>& box2);

int count_nonzero_iou(const std::vector<float>& ious);

std::vector<float> expand_box(const std::vector<float>& box, float scale = 0.1f);

std::string analyse_one(const std::string& file_path);

std::string analyse_two(const std::string& file_path);


bool process_last_n_lines(const std::string& txt_path, const std::string& save_dir, int keep_last_n, int& pic_id);

std::string read_txt_file(const std::string& file_path);

void clearFile(const std::string& filepath);

bool take_photo(int device_id, const std::string& save_path, const std::string& img_name);



void resize_images_in_folder(const std::string& folder_path, int max_length);

void create_empty_txt(const std::string& file_path);

void remove_folder_if_exists(const std::string& folder_path);

void movePhotos(ThreadSafeSet<std::string>& photo_names, const std::string& dest_folder, ThreadSafeSet<std::string>& action_id_record);
void delete_oldest_folders(const std::filesystem::path& parent_path, size_t max_folders);

bool copy_folder_to(const std::filesystem::path& src_folder, const std::filesystem::path& dst_root);

std::string replace_folder_name_in_path(const std::string& path_str, 
                                       const std::string& old_name, 
                                       const std::string& new_name);

bool delete_specified_folder(const std::string& folder_path);

bool isImageBlurry(const cv::Mat& image, double& variance_out);

bool delete_txt_file(const std::string& file_path);
bool is_folder_empty(const std::filesystem::path& folder_path);
void trim_folder_images(const std::filesystem::path& parent_path, size_t max_images_per_folder = 10);
void createBlankImage(const std::string& filename);

bool delete_folder_contents_only(const std::string& folder_path);

bool compressImageToTargetSize(const std::string& inputPath,
                               const std::string& outputPath,
                               int targetSizeKB,
                               int minQuality = 10,
                               int maxQuality = 95);

#endif