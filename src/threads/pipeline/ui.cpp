#include "threads/pipeline.h"
#include "threads/control.h"
#include "garage/garage.h"


static cv::Mat rvec_project = cv::Mat::zeros(3, 1, CV_64F);
static cv::Mat tvec_project = cv::Mat::zeros(3, 1, CV_64F);

static std::vector<cv::Point3f> project_in;
static std::vector<cv::Point2f> project_out;

static Eigen::Vector4d predict_pnp;
static Eigen::Vector4d predict_world;

static Eigen::Matrix4d trans_pnp2head;
static Eigen::Matrix4d trans_head2world;

static bool UI_TargetX(std::shared_ptr<rm::Frame> frame) {
    auto garage = Garage::get_instance();
    if (Data::target_id == rm::ARMOR_ID_UNKNOWN) return false;

    auto objptr = garage->getObj(Data::target_id);
    double dt = getDoubleOfS(frame->time_point, getTime());
    objptr->getTarget(predict_world, -dt, 0.0, 0.0);

    if ((std::abs(predict_world[0]) > 1e-2) || (std::abs(predict_world[1]) > 1e-2)) {
        predict_world(3, 0) = 1;
        predict_pnp = trans_pnp2head.inverse() * trans_head2world.inverse() * predict_world;
        
        project_in.clear();
        project_out.clear();
        project_in.emplace_back(
            static_cast<float>(predict_pnp(0, 0)), 
            static_cast<float>(predict_pnp(1, 0)),
            static_cast<float>(predict_pnp(2, 0)));
        cv::projectPoints(
            project_in, rvec_project, tvec_project,
            Data::camera[frame->camera_id]->intrinsic_matrix,
            Data::camera[frame->camera_id]->distortion_coeffs,
            project_out
        );
        if(!project_out.empty() && Data::image_flag) {
            rm::displayPredictTargetX(*(frame->image), project_out[0], Data::target_id);
        }
    }
    return true; 
}

static bool UI_CheckYaw(std::shared_ptr<rm::Frame> frame) {
    return true;
}

bool Pipeline::UI(std::shared_ptr<rm::Frame> frame) {
    auto garage = Garage::get_instance();

    rm::Camera* camera = Data::camera[frame->camera_id];
    trans_pnp2head = camera->Trans_pnp2head;
    rm::tf_trans_head2world(trans_head2world, frame->yaw, frame->pitch, frame->roll);

    
    if (!UI_TargetX(frame)) return false;
    if (!UI_CheckYaw(frame)) return false;
    return true;
}

bool Pipeline::monitor(std::shared_ptr<rm::Frame> frame) {
    auto garage = Garage::get_instance();

    for (auto yolo_rect : frame->yolo_list) {
        cv::Rect rect = yolo_rect.box;
        std::string info = "ID: " + std::to_string(yolo_rect.class_id);
        rm::message(info, frame->width, frame->height, rect);
    }

    for (auto armor : frame->armor_list) {
        std::string info = "Color: " + std::to_string(armor.color) + " Size: " + std::to_string(armor.size);
        rm::message(info, frame->width, frame->height, armor.four_points);
    }

    if (Data::target_id == rm::ARMOR_ID_UNKNOWN) {
        return false;
    }

    rm::Camera* camera = Data::camera[frame->camera_id];
    trans_pnp2head = camera->Trans_pnp2head;
    rm::tf_trans_head2world(trans_head2world, frame->yaw, frame->pitch, frame->roll);
    
    auto objptr = garage->getObj(Data::target_id);
    double dt = getDoubleOfS(frame->time_point, getTime());
    objptr->getTarget(predict_world, -dt, 0.0, 0.0);

    if ((std::abs(predict_world[0]) > 1e-2) || (std::abs(predict_world[1]) > 1e-2)) {
        predict_world(3, 0) = 1;
        predict_pnp = trans_pnp2head.inverse() * trans_head2world.inverse() * predict_world;
        
        project_in.clear();
        project_out.clear();
        project_in.emplace_back(
            static_cast<float>(predict_pnp(0, 0)), 
            static_cast<float>(predict_pnp(1, 0)),
            static_cast<float>(predict_pnp(2, 0)));
        cv::projectPoints(
            project_in, rvec_project, tvec_project,
            Data::camera[frame->camera_id]->intrinsic_matrix,
            Data::camera[frame->camera_id]->distortion_coeffs,
            project_out
        );
        if(!project_out.empty() && Data::image_flag) {
            std::string info = "ID: " + std::to_string(Data::target_id);
            rm::message(info, frame->width, frame->height, project_out[0]);
        }
    }
    return true; 
}