#ifndef UTILS_BIU_HPP
#define UTILS_BIU_HPP

#include <string>

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

void movePhotos(std::set<std::string>& photo_names, const std::string& dest_folder, std::set<std::string>& action_id_record);

void clearFile(const std::string& filepath);

bool take_photo(int device_id, const std::string& save_path, const std::string& img_name);

#endif