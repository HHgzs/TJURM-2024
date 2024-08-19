#include "threads/pipeline.h"
#include "threads/control.h"

using namespace rm;

void Pipeline::tracker_baseline_thread(
    std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in
) {
    auto garage = Garage::get_instance();
    auto param = Param::get_instance();
    auto control = Control::get_instance();

    // 获取装甲板长宽
    float big_width    = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Width"];
    float big_height   = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Height"];
    float small_width  = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Width"];
    float small_height = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Height"];

    std::string small_path = (*param)["Debug"]["SmallDecal"];
    std::string big_path   = (*param)["Debug"]["BigDecal"];

    init_pointer();
    init_locater();
    init_updater();
    
    if(Data::reprojection_flag) {
        initReprojection(small_width, small_height, big_width, big_height, small_path, big_path);
    }

    rm::CycleQueue<double> delay_list(100);
    TimePoint tp0, tp1, tp2;

    std::mutex mutex;
    while (true) {
        if (!Data::armor_mode) {
            std::unique_lock<std::mutex> lock(mutex);
            armor_cv_.wait(lock, [this]{return Data::armor_mode;});
        }

        std::unique_lock<std::mutex> lock_in(mutex_in);
        tracker_in_cv_.wait(lock_in, [&flag_in]{return flag_in;});

        std::shared_ptr<rm::Frame> frame = frame_in;
        flag_in = false;
        lock_in.unlock();

        tp1 = getTime();
        bool track_flag = true;
        if (track_flag) track_flag = pointer(frame);
        if (track_flag) track_flag = locater(frame);
        if (track_flag) track_flag = updater(frame);
        tp2 = getTime();

        if (Data::pipeline_delay_flag) rm::message("tracker time", getDoubleOfS(tp1, tp2) * 1000);
        if (track_flag) delay_list.push(getDoubleOfS(tp0, tp2));

        tp0 = tp2;
        double fps = 1.0 / delay_list.getAvg();
        rm::message("fps", fps);
        
        if (Data::image_flag) {
            if (Data::ui_flag) UI(frame);
            imshow(frame);
        }
        // if (Data::ui_flag) monitor(frame);
    }
}