#include "threads/pipeline.h"
#include <unistd.h>
#include <iostream>
#include <openrm/cudatools.h>
using namespace rm;
using namespace nvinfer1;
using namespace nvonnxparser;

void Pipeline::detector_rune_thread(
    std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in, 
    std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out
) {
    auto param = Param::get_instance();
    auto garage = Garage::get_instance();

    int    infer_width       = (*param)["Model"]["YoloRune"]["InferWidth"];
    int    infer_height      = (*param)["Model"]["YoloRune"]["InferHeight"];
    int    class_num         = (*param)["Model"]["YoloRune"]["ClassNum"];
    int    bboxes_num        = (*param)["Model"]["YoloRune"]["BboxesNum"];
    double confidence_thresh = (*param)["Model"]["YoloRune"]["ConfThresh"];
    double nms_thresh        = (*param)["Model"]["YoloRune"]["NMSThresh"];

    size_t yolo_struct_size = sizeof(float) * static_cast<size_t>(class_num + 9);
    std::mutex mutex;
    TimePoint tp0, tp1, tp2;
    rm::CycleQueue<double> delay_list(100);

    while (true) {
        if (!Data::rune_mode) {
            std::unique_lock<std::mutex> lock(mutex);
            rune_cv_.wait(lock, [this]{return Data::rune_mode;});
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
            rune_output_host_buffer_,
            rune_output_device_buffer_,
            &detect_stream_,
            yolo_struct_size,
            bboxes_num
        );
        frame->yolo_list = yoloArmorNMS_FP(
            rune_output_host_buffer_,
            bboxes_num,
            class_num,
            confidence_thresh,
            nms_thresh,
            frame->width,
            frame->height,
            infer_width,
            infer_height
        );
        
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
        rune_in_cv_.notify_one();
    }
}