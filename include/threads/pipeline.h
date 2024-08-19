#ifndef RM2024_THREADS_PIPELINE_H_
#define RM2024_THREADS_PIPELINE_H_

#include <mutex>
#include <memory>
#include <condition_variable>

#include "data_manager/base.h"
#include "data_manager/param.h"

#include "garage/garage.h"
#include "garage/wrapper_car.h"
#include "garage/wrapper_rune.h"
#include "garage/wrapper_tower.h"

class Pipeline {
public:
    static std::shared_ptr<Pipeline> get_instance() {
        static std::shared_ptr<Pipeline> instance(new Pipeline());
        return instance;
    }
    void autoaim_fourpoints();
    void autoaim_baseline();
    void autoaim_rune();
    void autoaim_combine();

    void init_pointer();
    void init_locater();
    void init_updater();
    void init_fourpoints();

    bool pointer(std::shared_ptr<rm::Frame> frame);
    bool locater(std::shared_ptr<rm::Frame> frame);
    bool updater(std::shared_ptr<rm::Frame> frame);
    bool rector(std::shared_ptr<rm::Frame> frame);
    bool classifier(std::shared_ptr<rm::Frame> frame);
    bool fourpoints(std::shared_ptr<rm::Frame> frame);
    bool UI(std::shared_ptr<rm::Frame> frame);
    bool monitor(std::shared_ptr<rm::Frame> frame);

    void preprocessor_fourpoints_thread(
        std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out);

    void detector_fourpoints_thread(
        std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in);

    void preprocessor_rune_thread(
        std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out);

    void detector_rune_thread(
        std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in, 
        std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out);

    void tracker_rune_thread(
        std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in);

    void preprocessor_baseline_thread(
        std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out);

    void detector_baseline_thread(
        std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in,
        std::mutex& mutex_out, bool& flag_out, std::shared_ptr<rm::Frame>& frame_out);

    void tracker_baseline_thread(
        std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in);

    void recording_thread(
        std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in);

    void image_thread();

    void start_record();
    void stop_record();
    void switch_armor_to_rune();
    void switch_rune_to_armor();
    void record(std::shared_ptr<rm::Frame> frame_record);
    void imshow(std::shared_ptr<rm::Frame> frame_show);
    void imshow(std::shared_ptr<rm::Frame> frame_show, std::string& frame_msg);



private:
    std::shared_ptr<rm::Frame> preprocessor_register_;
    std::shared_ptr<rm::Frame> detector_register_;
    std::shared_ptr<rm::Frame> record_register_;
    std::shared_ptr<rm::Frame> imshow_register_;

    std::shared_ptr<rm::Frame> preprocessor_register2_;
    std::shared_ptr<rm::Frame> detector_in_register_;
    std::shared_ptr<rm::Frame> detector_in_register2_;
    std::shared_ptr<rm::Frame> detector_out_register_;
    std::shared_ptr<rm::Frame> detector_out_register2_;

    std::condition_variable tracker_in_cv_;
    std::condition_variable rune_in_cv_;
    std::condition_variable locater_in_cv_;

    std::condition_variable armor_cv_;
    std::condition_variable rune_cv_;
    std::condition_variable record_cv_;

    std::mutex preprocessor_mutex_;
    std::mutex detector_mutex_;
    std::mutex detector_in_mutex_;
    std::mutex detector_out_mutex_;
    std::mutex record_mutex_;
    
    bool preprocessor_over_ = false;
    bool detector_over_ = false;
    bool detector_in_over_ = false;
    bool detector_out_over_ = false;

    bool imshow_in_ = false;
    bool record_in_ = false;

    cudaStream_t resize_stream_;
    cudaStream_t detect_stream_;
    nvinfer1::IExecutionContext* armor_context_;
    nvinfer1::IExecutionContext* rune_context_;

    float* armor_input_device_buffer_ = nullptr;
    float* armor_output_device_buffer_ = nullptr;
    float* armor_output_host_buffer_ = nullptr;

    float* rune_input_device_buffer_ = nullptr;
    float* rune_output_device_buffer_ = nullptr;
    float* rune_output_host_buffer_ = nullptr;
private:
    Pipeline() = default;
    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;
};

#endif