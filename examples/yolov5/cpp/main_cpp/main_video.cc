// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "yolov5.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"

#include <filesystem>
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <utils_biu.hpp>

#if defined(RV1106_1103) 
    #include "dma_alloc.hpp"
#endif



// std::atomic<bool> need_take_video(false);
// std::atomic<bool> video_done(false);  // 告诉别的线程：“拍完视频了！”
// std::atomic<bool> running(true);


/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("%s <model_path> \n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    


/*--------------判断视频路径是否存在并创建---------------------*/
    std::string videos_path = "/mnt/videos";
    ensure_path_exists(videos_path)
/*--------------判断视频路径是否存在并创建---------------------*/


/*--------------创建陀螺仪检测---------------------*/
    float angle1 = 20.0f;
    std::thread t1([angle1]() {
        while (true) {
            float result = tly_detect(angle1);
            std::cout << "检测到陀螺仪角度: " << result << std::endl;
            take_video(image_path);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 控制循环频率
        }
    });
/*--------------创建陀螺仪检测---------------------*/



    int ret;
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

    init_post_process();

    ret = init_yolov5_model(model_path, &rknn_app_ctx);
    if (ret != 0)
    {
        printf("init_yolov5_model fail! ret=%d model_path=%s\n", ret, model_path);
        goto out;
    }


    
    std::string images_dir = "/mnt/frames_dir";
    ensure_path_exists(images_dir);

    std::string txt_dir = "/mnt/txt_dir";
    ensure_path_exists(txt_dir);



    std::set<std::string> processed;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::vector<std::string> videos = get_complete_videos(videos_path);

        for (const auto& video_path : videos) {
            if (processed.count(video_path)) continue;

            std::string video_name = std::fs::path(video_path).stem().string();
            std::string output_subdir = images_dir + "/" + video_name;
            ensure_path_exists(output_subdir);

            if (!extract_frames(video_path, output_subdir, 30)) {
                std::cerr << "❌ 提取失败: " << video_path << std::endl;
            } else {
                std::cout << "✅ 处理完成: " << video_path << std::endl;
                processed.insert(video_path);

                std::vector<std::string> frames = get_image_paths(output_subdir);
                for (const auto& img_path : frames) {
                    std::cout << "图片路径: " << img_path << std::endl;

                    std::string frame_id = std::fs::path(img_path).stem().string();

                    image_buffer_t src_image;
                    memset(&src_image, 0, sizeof(image_buffer_t));
                    ret = read_image(img_path, &src_image);

                #if defined(RV1106_1103) 
                    //RV1106 rga requires that input and output bufs are memory allocated by dma
                    ret = dma_buf_alloc(RV1106_CMA_HEAP_PATH, src_image.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                                    (void **) & (rknn_app_ctx.img_dma_buf.dma_buf_virt_addr));
                    memcpy(rknn_app_ctx.img_dma_buf.dma_buf_virt_addr, src_image.virt_addr, src_image.size);
                    dma_sync_cpu_to_device(rknn_app_ctx.img_dma_buf.dma_buf_fd);
                    free(src_image.virt_addr);
                    src_image.virt_addr = (unsigned char *)rknn_app_ctx.img_dma_buf.dma_buf_virt_addr;
                    src_image.fd = rknn_app_ctx.img_dma_buf.dma_buf_fd;
                    rknn_app_ctx.img_dma_buf.size = src_image.size;
                #endif

                    if (ret != 0)
                    {
                        printf("read image fail! ret=%d image_path=%s\n", ret, image_path);
                        goto out;
                    }
                
                    object_detect_result_list od_results;
                
                    ret = inference_yolov5_model(&rknn_app_ctx, &src_image, &od_results);
                    if (ret != 0)
                    {
                        printf("init_yolov5_model fail! ret=%d\n", ret);
                        goto out;
                    }
                
                

                    for (int i = 0; i < od_results.count; i++)
                    {
                        object_detect_result *det_result = &(od_results.results[i]);
                        printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
                            det_result->box.left, det_result->box.top,
                            det_result->box.right, det_result->box.bottom,
                            det_result->prop);
                        int x1 = det_result->box.left;
                        int y1 = det_result->box.top;
                        int x2 = det_result->box.right;
                        int y2 = det_result->box.bottom;

                        std::string frame_number_str = video_name + "_" + frame_id + "_" + i

                        if (od_results.count == 1 && std::strcmp(coco_cls_to_name(det_result->cls_id), "full")==0 && det_result->prop >= 0.9){
                            std::string txt1_name_path = txt_dir + "/" + video_name + "_1" + ".txt";
                            std::ofstream outfile(txt1_name_path, std::ios::app); 
                            if (outfile.is_open()) {
                                outfile << frame_number_str << " "
                                        << det_result->cls_id << " "
                                        << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
                                outfile.close();  // 关闭文件
                                std::cout << "写入成功！" << std::endl;
                            } else {
                                std::cout << "无法打开文件！" << std::endl;
                            }
                        }

                        if (od_results.count == 2 && std::strcmp(coco_cls_to_name(det_result->cls_id), "full")==0 && det_result->prop >= 0.5){
                            std::string txt2_name_path = txt_dir + "/" + video_name + "_2" + ".txt";
                            std::ofstream outfile(txt2_name_path, std::ios::app); 
                            if (outfile.is_open()) {
                                outfile << frame_number_str << " "
                                        << det_result->cls_id << " "
                                        << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
                                outfile.close();  // 关闭文件
                                std::cout << "写入成功！" << std::endl;
                            } else {
                                std::cout << "无法打开文件！" << std::endl;
                            }        
                        }

                    }


                out:
                    deinit_post_process();
                
                    ret = release_yolov5_model(&rknn_app_ctx);
                    if (ret != 0)
                    {
                        printf("release_yolov5_model fail! ret=%d\n", ret);
                    }
                
                    if (src_image.virt_addr != NULL)
                    {
                #if defined(RV1106_1103) 
                        dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                                rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);
                #else
                        free(src_image.virt_addr);
                #endif
                    }
                    
                }

                std::string txt1_name_path_result = txt_dir + "/" + video_name + "_1" + ".txt";

                std::string txt2_name_path_result = txt_dir + "/" + video_name + "_2" + ".txt";

                std::string crop_img_dirs = "/mnt/crop_images";


                std::string final_result;
                if (file_exists_and_not_empty(txt2_name_path_result)){
                    final_result = analyse_two(txt2_name_path_result);
                    int line_n = 10;
                    std::string crop_img_path = crop_img_dirs + "/" + video_name;
                    if (process_last_n_lines(txt2_name_path_result, crop_img_path, line_n)) {
                        std::cout << "👍 有截图保存成功" << std::endl;
                    } else {
                        std::cout << "😢 没有任何截图保存成功" << std::endl;
                    }
                   

                }else{
                    final_result = analyse_one(txt1_name_path_result);
                    int line_n = 5;
                    std::string crop_img_path = crop_img_dirs + "/" + video_name;
                    if (process_last_n_lines(txt2_name_path_result, crop_img_path, line_n)) {
                        std::cout << "👍 有截图保存成功" << std::endl;
                    } else {
                        std::cout << "😢 没有任何截图保存成功" << std::endl;
                    }
                }

            }
        }
    }

    return 0;
}

