#include "threads/pipeline.h"
#include <unistd.h>
#include <iostream>
#include <openrm/cudatools.h>

using namespace rm;
using namespace nvinfer1;
using namespace nvonnxparser;

void Pipeline::preprocessor_baseline_thread(
    std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out
) {
    auto param = Param::get_instance();
    auto garage = Garage::get_instance();

    std::string yolo_type   = (*param)["Model"]["YoloArmor"]["Type"];
    std::string onnx_file   = (*param)["Model"]["YoloArmor"][yolo_type]["DirONNX"];
    std::string engine_file = (*param)["Model"]["YoloArmor"][yolo_type]["DirEngine"];
    

    int  infer_width     = (*param)["Model"]["YoloArmor"][yolo_type]["InferWidth"];
    int  infer_height    = (*param)["Model"]["YoloArmor"][yolo_type]["InferHeight"];
    int  class_num       = (*param)["Model"]["YoloArmor"][yolo_type]["ClassNum"];
    int  locate_num      = (*param)["Model"]["YoloArmor"][yolo_type]["LocateNum"];
    int  color_num       = (*param)["Model"]["YoloArmor"][yolo_type]["ColorNum"];
    int  bboxes_num      = (*param)["Model"]["YoloArmor"][yolo_type]["BboxesNum"];
    bool hist_input_flag = (*param)["Model"]["YoloArmor"][yolo_type]["NeedHist"];

    if (access(engine_file.c_str(), F_OK) == 0) {
        if (!rm::initTrtEngine(engine_file, &armor_context_)) exit(-1);
    } else if (access(onnx_file.c_str(), F_OK) == 0){
        if (!rm::initTrtOnnx(onnx_file, engine_file, &armor_context_, 1U)) exit(-1);
    } else {
        rm::message("No model file found!", rm::MSG_ERROR);
        exit(-1);
    }

    size_t yolo_struct_size = sizeof(float) * static_cast<size_t>(locate_num + 1 + color_num + class_num);
    mallocYoloDetectBuffer(
        &armor_input_device_buffer_, 
        &armor_output_device_buffer_, 
        &armor_output_host_buffer_, 
        infer_width, 
        infer_height, 
        yolo_struct_size,
        bboxes_num);

    std::mutex mutex;
    TimePoint frame_wait, flag_wait;
    TimePoint tp0, tp1, tp2;
        while(true) {
        if (!Data::armor_mode) {
            std::unique_lock<std::mutex> lock(mutex);
            armor_cv_.wait(lock, [this]{return Data::armor_mode;});
        }

        Camera* camera = Data::camera[Data::camera_index];
        std::shared_ptr<rm::Frame> frame = camera->buffer->pop();

        frame_wait = tp1 = getTime();
        while(frame == nullptr) {
            frame = camera->buffer->pop();
            double delay = getDoubleOfS(frame_wait, getTime());
            if (delay > 0.5 && Data::timeout_flag) {
                rm::message("Capture timeout", rm::MSG_ERROR);
                exit(-1);
            }
        }

        memcpyYoloCameraBuffer(
            frame->image->data,
            camera->rgb_host_buffer,
            camera->rgb_device_buffer,
            frame->width,
            frame->height);
        
        resize(
            camera->rgb_device_buffer,
            frame->width,
            frame->height,
            armor_input_device_buffer_,
            infer_width,
            infer_height,
            (void*)resize_stream_
        );
        detectEnqueue(
            armor_input_device_buffer_,
            armor_output_device_buffer_,
            &armor_context_,
            &detect_stream_
        );

        if (Data::record_mode) { record(frame); }

        tp2 = getTime();
        if (Data::pipeline_delay_flag) rm::message("preprocess", getDoubleOfS(tp1, tp2) * 1000);

        flag_wait = getTime();
        while(flag_out) {
            if (getDoubleOfS(flag_wait, getTime()) > 10.0 && Data::timeout_flag) {
                rm::message("Preprocessor timeout", rm::MSG_ERROR);
                exit(-1);
            }
        }
        
        std::unique_lock<std::mutex> lock_out(mutex_out);
        frame_out = frame;
        flag_out = true;
    }
}