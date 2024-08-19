#include "threads/pipeline.h"
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
void Pipeline::record(std::shared_ptr<rm::Frame> frame_record) {
    std::unique_lock<std::mutex> lock(record_mutex_);
    record_register_ = frame_record;
    record_in_ = true;
}

void Pipeline::recording_thread(std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in) {
    auto param = Param::get_instance();
    unsigned long long int frame_count = 0;

    cv::VideoWriter writer;
    std::mutex mutex;
    while(true) {
        if(!Data::record_mode) {
            std::unique_lock<std::mutex> lock(mutex);
            record_cv_.wait(lock, [this]{return Data::record_mode;});
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        if(!flag_in) continue;

        std::unique_lock<std::mutex> lock(mutex_in);
        std::shared_ptr<rm::Frame> frame = frame_in;
        frame_in = nullptr;
        flag_in = false;
        lock.unlock();

        if(frame == nullptr) continue;
        if(frame_count == 0) {
            std::string filedir = (*param)["Camera"]["VideoSaveDir"];
            filedir = filedir +  "/" + getTimeStr() + ".avi";
            rm::message("Recording to " + filedir, rm::MSG_NOTE);
            
            writer.open(
                filedir, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                25, cv::Size(frame->width, frame->height));

            if(!writer.isOpened()) { frame_count = 0; continue; }
        }
        frame_count++;
        writer.write(*(frame->image));
        
        if(frame_count > 100) {
            writer.release();
            frame_count = 0;
        }
    }
}
