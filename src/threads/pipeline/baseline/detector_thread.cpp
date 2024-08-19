#include "threads/pipeline.h"
#include <unistd.h>
#include <iostream>
#include <openrm/cudatools.h>
using namespace rm;
using namespace nvinfer1;
using namespace nvonnxparser;

void Pipeline::detector_baseline_thread(
    std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in, 
    std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out
) {
    auto param = Param::get_instance();
    auto garage = Garage::get_instance();

    std::string yolo_type = (*param)["Model"]["YoloArmor"]["Type"];

    int    infer_width       = (*param)["Model"]["YoloArmor"][yolo_type]["InferWidth"];
    int    infer_height      = (*param)["Model"]["YoloArmor"][yolo_type]["InferHeight"];
    int    class_num         = (*param)["Model"]["YoloArmor"][yolo_type]["ClassNum"];
    int    locate_num        = (*param)["Model"]["YoloArmor"][yolo_type]["LocateNum"];
    int    color_num         = (*param)["Model"]["YoloArmor"][yolo_type]["ColorNum"];
    int    bboxes_num        = (*param)["Model"]["YoloArmor"][yolo_type]["BboxesNum"];
    double confidence_thresh = (*param)["Model"]["YoloArmor"][yolo_type]["ConfThresh"];
    double nms_thresh        = (*param)["Model"]["YoloArmor"][yolo_type]["NMSThresh"];

    size_t yolo_struct_size = sizeof(float) * static_cast<size_t>(locate_num + 1 + color_num + class_num);
    std::mutex mutex;
    TimePoint tp0, tp1, tp2;
    while (true) {
        if (!Data::armor_mode) {
            std::unique_lock<std::mutex> lock(mutex);
            armor_cv_.wait(lock, [this]{return Data::armor_mode;});
        }

        tp0 = getTime();
        while(!flag_in) {
            if (getDoubleOfS(tp0, getTime()) > 10.0 && Data::timeout_flag) {
                rm::message("Detector timeout", rm::MSG_ERROR);
                exit(-1);
            }
        }
        
        std::unique_lock<std::mutex> lock_in(mutex_in);
        std::shared_ptr<rm::Frame> frame = frame_in;
        flag_in = false;
        lock_in.unlock();


        tp1 = getTime();

        detectOutput(
            armor_output_host_buffer_,
            armor_output_device_buffer_,
            &detect_stream_,
            yolo_struct_size,
            bboxes_num
        );
        
        if (yolo_type == "V5") {
            frame->yolo_list = yoloArmorNMS_V5(
                armor_output_host_buffer_,
                bboxes_num,
                class_num,
                confidence_thresh,
                nms_thresh,
                frame->width,
                frame->height,
                infer_width,
                infer_height
            );
        } else if (yolo_type == "FP") {
            frame->yolo_list = yoloArmorNMS_FP(
                armor_output_host_buffer_,
                bboxes_num,
                class_num,
                confidence_thresh,
                nms_thresh,
                frame->width,
                frame->height,
                infer_width,
                infer_height
            );
        } else if (yolo_type == "FPX") {
            frame->yolo_list = yoloArmorNMS_FPX(
                armor_output_host_buffer_,
                bboxes_num,
                class_num,
                confidence_thresh,
                nms_thresh,
                frame->width,
                frame->height,
                infer_width,
                infer_height
            );
        } else {
            rm::message("Invalid yolo type", rm::MSG_ERROR);
            exit(-1);
        }
        
        if (frame->yolo_list.empty()) {
            if (Data::image_flag) imshow(frame);
            continue;
        } 

        tp2 = getTime();
        if (Data::pipeline_delay_flag) rm::message("detect time", getDoubleOfS(tp1, tp2) * 1000);


        std::unique_lock<std::mutex> lock_out(mutex_out);
        frame_out = frame;
        flag_out = true;
        lock_out.unlock();
        tracker_in_cv_.notify_one();
    }
}