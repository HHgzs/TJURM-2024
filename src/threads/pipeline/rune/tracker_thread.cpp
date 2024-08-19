#include "threads/pipeline.h"
#include "threads/control.h"
using namespace std;
using namespace rm;


void Pipeline::tracker_rune_thread(
    std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in
) {
    auto garage = Garage::get_instance();
    auto param = Param::get_instance();
    auto control = Control::get_instance();

    float rune_width = (*param)["Points"]["PnP"]["Rune"]["Width"];
    float rune_height = (*param)["Points"]["PnP"]["Rune"]["Height"];

    // 符的3D坐标, 以符在最右为原点
    // 四点顺序：左上，左下，右下，右上
    std::vector<cv::Point3f> Rune3D;
    Rune3D.emplace_back(-rune_height / 2, -rune_width / 2, 0);
    Rune3D.emplace_back(rune_height / 2, -rune_width / 2, 0);
    Rune3D.emplace_back(-rune_height / 2, rune_width / 2, 0);
    Rune3D.emplace_back(rune_height / 2, rune_width / 2, 0);

    cv::Mat rvec, tvec, rotate_cv;
    Eigen::Vector4d pose_pnp, pose_world;
    Eigen::Matrix3d rotate_pnp, rotate_world;

    Eigen::Matrix3d rotate_pnp2head, rotate_head2world;
    Eigen::Matrix4d trans_pnp2head, trans_head2world;


    rm::CycleQueue<double> delay_list(100);
    TimePoint tp0, tp1, tp2;

    std::mutex mutex;
    while (true) {
        if (!Data::rune_mode) {
            std::unique_lock<std::mutex> lock(mutex);
            rune_cv_.wait(lock, [this]{return Data::rune_mode;});
        }

        std::unique_lock<std::mutex> lock_in(mutex_in);
        rune_in_cv_.wait(lock_in, [&flag_in]{return flag_in;});

        std::shared_ptr<rm::Frame> frame = frame_in;
        flag_in = false;
        lock_in.unlock();

        tp1 = getTime();

        rotate_pnp2head = Data::camera[frame->camera_id]->Rotate_pnp2head;
        rm::tf_rotate_head2world(rotate_head2world, frame->yaw, frame->pitch);

        trans_pnp2head = Data::camera[frame->camera_id]->Trans_pnp2head;
        rm::tf_trans_head2world(trans_head2world, frame->yaw, frame->pitch);

        for (auto& yolo_rect : frame->yolo_list) {

            // 符的网络输出结果中target的类别为0，只保留target
            if(yolo_rect.class_id != 0) continue;

            // 如果检测到的装甲板不是四个点，跳过
            if(yolo_rect.four_points.size() != 4) continue;

            rm::Armor armor;
            armor.four_points = yolo_rect.four_points;

            if (Data::image_flag && Data::ui_flag) {
                rm::displaySingleArmorLine(*(frame->image), armor);
            }
            
            try {
                cv::solvePnP(Rune3D, armor.four_points,
                    Data::camera[frame->camera_id]->intrinsic_matrix,
                    Data::camera[frame->camera_id]->distortion_coeffs,
                    rvec, tvec, false, cv::SOLVEPNP_EPNP);
            } catch (cv::Exception& e) {
                rm::message("solvePnP error", rm::MSG_ERROR);
                continue;
            }

            // 保存符的目标
            rm::Target target;

            // 计算符面旋转角度
            cv::Rodrigues(rvec, rotate_cv);
            rm::tf_Mat3d(rotate_cv, rotate_pnp);
            rotate_world = rotate_head2world * rotate_pnp2head * rotate_pnp;
            target.rune_angle = rm::tf_rotation2runeroll(rotate_pnp);
            target.armor_yaw_world = rm::tf_rotation2armoryaw(rotate_world);

            // 计算符面在世界坐标系下的坐标
            rm::tf_Vec4d(tvec, pose_pnp);
            target.pose_world = trans_head2world * trans_pnp2head * pose_pnp;

            // 计算符的距离
            double distance = sqrt(target.pose_world(0, 0) * target.pose_world(0, 0) + 
                                target.pose_world(1, 0) * target.pose_world(1, 0) + 
                                target.pose_world(2, 0) * target.pose_world(2, 0));
            if(distance > 10.0) continue;
            rm::message("pnp dist", distance);
            
            // 从车库中获取符的目标
            ObjPtr objptr = garage->getObj(rm::ARMOR_ID_RUNE);
            objptr->push(target, frame->time_point);

            // 当出现一个target后，不再更新
            break;
        }

        tp2 = getTime();
        if (Data::pipeline_delay_flag) rm::message("tracker time", getDoubleOfS(tp1, tp2) * 1000);
        
        delay_list.push(getDoubleOfS(tp0, tp2));
        tp0 = tp2;
        double fps = 1.0 / delay_list.getAvg();
        rm::message("fps", fps);

        if (Data::imshow_flag) {
            if (Data::ui_flag) UI(frame);
            imshow(frame);
        }
        // if (Data::ui_flag) monitor(frame);
    }
    
}