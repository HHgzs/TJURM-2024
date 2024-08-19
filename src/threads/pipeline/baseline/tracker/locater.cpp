#include "threads/pipeline.h"
#include "garage/garage.h"

static std::vector<cv::Point3f>* BigArmorRed3D, *SmallArmorRed3D;
static std::vector<cv::Point3f>* BigArmorBlue3D, *SmallArmorBlue3D;
static bool   plus_pnp_cost_image;
static double plus_pnp_cost_ratio;

void Pipeline::init_locater() {
    auto param = Param::get_instance();

    // 获取装甲板长宽
    float bigArmorRed_width     = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Width"];
    float bigArmorRed_height    = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Height"];
    float smallArmorRed_width   = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Width"];
    float smallArmorRed_height  = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Height"];

    // 获取装甲板长宽
    float bigArmorBlue_width    = (*param)["Points"]["PnP"]["Blue"]["BigArmor"]["Width"];
    float bigArmorBlue_height   = (*param)["Points"]["PnP"]["Blue"]["BigArmor"]["Height"];
    float smallArmorBlue_width  = (*param)["Points"]["PnP"]["Blue"]["SmallArmor"]["Width"];
    float smallArmorBlue_height = (*param)["Points"]["PnP"]["Blue"]["SmallArmor"]["Height"];

    // 获取PnP损失函数参数
    plus_pnp_cost_image = (*param)["Debug"]["PlusPnP"]["CostImage"];

    // 设置装甲板3D坐标，顺序为左上-右上-左下-右下，即矩阵行优先输出顺序
    BigArmorRed3D = new std::vector<cv::Point3f>();
    SmallArmorRed3D = new std::vector<cv::Point3f>();

    BigArmorRed3D->emplace_back(-bigArmorRed_width / 2, -bigArmorRed_height / 2, 0);
    BigArmorRed3D->emplace_back(bigArmorRed_width / 2, -bigArmorRed_height / 2, 0);
    BigArmorRed3D->emplace_back(-bigArmorRed_width / 2, bigArmorRed_height / 2, 0);
    BigArmorRed3D->emplace_back(bigArmorRed_width / 2, bigArmorRed_height / 2, 0);

    SmallArmorRed3D->emplace_back(-smallArmorRed_width / 2, -smallArmorRed_height / 2, 0);
    SmallArmorRed3D->emplace_back(smallArmorRed_width / 2, -smallArmorRed_height / 2, 0);
    SmallArmorRed3D->emplace_back(-smallArmorRed_width / 2, smallArmorRed_height / 2, 0);
    SmallArmorRed3D->emplace_back(smallArmorRed_width / 2, smallArmorRed_height / 2, 0);

    // 设置装甲板3D坐标，顺序为左上-右上-左下-右下，即矩阵行优先输出顺序
    BigArmorBlue3D = new std::vector<cv::Point3f>();
    SmallArmorBlue3D = new std::vector<cv::Point3f>();

    BigArmorBlue3D->emplace_back(-bigArmorBlue_width / 2, -bigArmorBlue_height / 2, 0);
    BigArmorBlue3D->emplace_back(bigArmorBlue_width / 2, -bigArmorBlue_height / 2, 0);
    BigArmorBlue3D->emplace_back(-bigArmorBlue_width / 2, bigArmorBlue_height / 2, 0);
    BigArmorBlue3D->emplace_back(bigArmorBlue_width / 2, bigArmorBlue_height / 2, 0);

    SmallArmorBlue3D->emplace_back(-smallArmorBlue_width / 2, -smallArmorBlue_height / 2, 0);
    SmallArmorBlue3D->emplace_back(smallArmorBlue_width / 2, -smallArmorBlue_height / 2, 0);
    SmallArmorBlue3D->emplace_back(-smallArmorBlue_width / 2, smallArmorBlue_height / 2, 0);
    SmallArmorBlue3D->emplace_back(smallArmorBlue_width / 2, smallArmorBlue_height / 2, 0);
}

bool Pipeline::locater(std::shared_ptr<rm::Frame> frame) {
    auto garage = Garage::get_instance();
    
    cv::Mat rvec, tvec, rotate_cv;
    std::vector<cv::Point3f> *Armor3D;

    Eigen::Vector4d pose_pnp, pose_head, pose_world;
    Eigen::Matrix3d rotate_pnp, rotate_world;
    
    Eigen::Matrix3d rotate_pnp2head, rotate_head2world;
    Eigen::Matrix4d trans_pnp2head, trans_head2world;

    rm::Camera* camera = Data::camera[frame->camera_id];

    rotate_pnp2head = camera->Rotate_pnp2head;
    rm::tf_rotate_head2world(rotate_head2world, frame->yaw, frame->pitch, frame->roll);

    trans_pnp2head = camera->Trans_pnp2head;
    rm::tf_trans_head2world(trans_head2world, frame->yaw, frame->pitch, frame->roll);


    for(auto& armor : frame->armor_list) {
        if(armor.four_points.size() != 4) { 
            continue;
        }

        auto objptr = garage->getObj(armor.id);
        rm::ArmorSize obj_size = objptr->getArmorSize();
        rm::ArmorSize curr_size = obj_size;
        if (obj_size == rm::ARMOR_SIZE_UNKNOWN) curr_size = armor.size;
        else curr_size = obj_size;

        if(curr_size == rm::ARMOR_SIZE_BIG_ARMOR) {
            if(armor.color == rm::ARMOR_COLOR_RED) Armor3D = BigArmorRed3D;
            else if(armor.color == rm::ARMOR_COLOR_BLUE) Armor3D = BigArmorBlue3D;
            else continue;
        } else if(curr_size == rm::ARMOR_SIZE_SMALL_ARMOR) {
            if(armor.color == rm::ARMOR_COLOR_RED) Armor3D = SmallArmorRed3D;
            else if(armor.color == rm::ARMOR_COLOR_BLUE) Armor3D = SmallArmorBlue3D;
            else continue;
        } else {
            continue;
        }

        rm::Target target;
        target.armor_id = armor.id;
        target.armor_size = armor.size;

        if (Data::plus_pnp) {
            target.armor_yaw_world = rm::solveYawPnP(
                frame->yaw, camera, pose_world, *Armor3D, armor.four_points, 
                rotate_head2world, trans_head2world, armor.id, plus_pnp_cost_image);
            target.pose_world = pose_world;
            
        } else {
            try {
                cv::solvePnP(*Armor3D, armor.four_points,
                            Data::camera[frame->camera_id]->intrinsic_matrix,
                            Data::camera[frame->camera_id]->distortion_coeffs,
                            rvec, tvec, false, cv::SOLVEPNP_IPPE);
            } catch (cv::Exception e) {
                rm::message("solvePnP error", rm::MSG_ERROR);
                continue;
            }

            cv::Rodrigues(rvec, rotate_cv);
            rm::tf_Mat3d(rotate_cv, rotate_pnp);
            rotate_world = rotate_head2world * rotate_pnp2head * rotate_pnp;
            target.armor_yaw_world = rm::tf_rotation2armoryaw(rotate_world);

            rm::tf_Vec4d(tvec, pose_pnp);
            pose_world = trans_head2world * trans_pnp2head * pose_pnp;
            target.pose_world = pose_world;
        }
        
        frame->target_list.push_back(target);

        double distance = sqrt(pow(target.pose_world(0), 2) + pow(target.pose_world(1), 2) + pow(target.pose_world(2), 2));
        rm::message("pnp dist", distance);
        rm::message("pnp yaw", target.armor_yaw_world * (180 / M_PI));
    }

    if(frame->target_list.size() == 0) {
        if (Data::image_flag) imshow(frame);
        return false;
    }
    
    return true;
}