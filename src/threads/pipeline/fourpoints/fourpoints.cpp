#include "threads/pipeline.h"
#include "threads/control.h"

static double armor_size_ratio;

static std::vector<cv::Point3f>* BigArmor3D, *SmallArmor3D;
static cv::Mat rvec_project = cv::Mat::zeros(3, 1, CV_64F);
static cv::Mat tvec_project = cv::Mat::zeros(3, 1, CV_64F);

static std::vector<double> referee_offset;
static std::vector<double> axis_offset;

static std::vector<int> armor_class_map;
static std::vector<int> armor_color_map;

double small_width, small_heigth, big_width, big_heigth;
std::string small_path, big_path;

void Pipeline::init_fourpoints() {
    auto param = Param::get_instance();
    
    armor_size_ratio = (*param)["Points"]["Armor"]["SizeRatio"];

    // 获取装甲板长宽
    float bigArmor_width = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Width"];
    float bigArmor_height = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Height"];
    float smallArmor_width = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Width"];
    float smallArmor_height = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Height"];

    // 设置装甲板3D坐标，顺序为左上-右上-左下-右下，即矩阵行优先输出顺序
    BigArmor3D = new std::vector<cv::Point3f>();
    SmallArmor3D = new std::vector<cv::Point3f>();

    BigArmor3D->emplace_back(-bigArmor_width / 2, -bigArmor_height / 2, 0);
    BigArmor3D->emplace_back(bigArmor_width / 2, -bigArmor_height / 2, 0);
    BigArmor3D->emplace_back(-bigArmor_width / 2, bigArmor_height / 2, 0);
    BigArmor3D->emplace_back(bigArmor_width / 2, bigArmor_height / 2, 0);

    SmallArmor3D->emplace_back(-smallArmor_width / 2, -smallArmor_height / 2, 0);
    SmallArmor3D->emplace_back(smallArmor_width / 2, -smallArmor_height / 2, 0);
    SmallArmor3D->emplace_back(-smallArmor_width / 2, smallArmor_height / 2, 0);
    SmallArmor3D->emplace_back(smallArmor_width / 2, smallArmor_height / 2, 0);

    // 获取图传偏移参数
    std::vector<double> temp_referee_offset = (*param)["Car"]["RefereeOffset"];
    std::vector<double> temp_axis_offset    = (*param)["Car"]["AxisOffset"];
    referee_offset = temp_referee_offset;
    axis_offset    = temp_axis_offset;

    // 设置装甲板ID映射
    std::string      yolo_type      = (*param)["Model"]["YoloArmor"]["Type"];
    std::vector<int> temp_class_map = (*param)["Model"]["YoloArmor"][yolo_type]["ClassMap"];
    std::vector<int> temp_color_map = (*param)["Model"]["YoloArmor"][yolo_type]["ColorMap"];
    armor_class_map = temp_class_map;
    armor_color_map = temp_color_map;

    // 设置重投影参数
    double small_width     = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Width"];
    double small_heigth    = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Height"];
    double big_width       = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Width"];
    double big_heigth      = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Height"];
    std::string small_path = (*param)["Debug"]["SmallDecal"];
    std::string big_path   = (*param)["Debug"]["BigDecal"];

    // 初始化重投影
    if(Data::reprojection_flag) {
        rm::initReprojection(small_width, small_heigth, big_width, big_heigth, small_path, big_path);
    }
}

bool Pipeline::fourpoints(std::shared_ptr<rm::Frame> frame) {
    auto garage = Garage::get_instance();
    auto param = Param::get_instance();
    auto control = Control::get_instance();

    cv::Mat rvec, tvec, rotate_cv;
    Eigen::Vector4d pose_pnp, pose_world;
    Eigen::Vector4d predict_pnp, predict_world;
    Eigen::Matrix3d rotate_pnp, rotate_world;

    Eigen::Matrix3d rotate_pnp2head, rotate_head2world;
    Eigen::Matrix4d trans_pnp2head, trans_head2world;

    std::vector<cv::Point3f> project_in;
    std::vector<cv::Point2f> project_out;


    // 根据相机ID获取相机参数，得到旋转矩阵和平移矩阵
    rotate_pnp2head = Data::camera[frame->camera_id]->Rotate_pnp2head;
    rm::tf_rotate_head2world(rotate_head2world, frame->yaw, frame->pitch);

    trans_pnp2head = Data::camera[frame->camera_id]->Trans_pnp2head;
    rm::tf_trans_head2world(trans_head2world, frame->yaw, frame->pitch);


    // 遍历所有检测到的装甲板
    for (auto& yolo_rect : frame->yolo_list) {

        // 如果检测到的装甲板不是四个点，跳过
        if(yolo_rect.four_points.size() != 4) continue;

        // 根据推理结果设置装甲板ID
        rm::ArmorID armor_id = (rm::ArmorID)armor_class_map[yolo_rect.class_id];
        rm::ArmorColor armor_color = (rm::ArmorColor)armor_class_map[yolo_rect.color_id];

        // 创建装甲板对象并设置参数
        rm::Armor armor;
        armor.id = armor_id;
        armor.color = armor_color;
        armor.four_points = yolo_rect.four_points;
        armor.rect = yolo_rect.box;

        rm::setArmorSizeByPoints(armor, armor_size_ratio);
        rm::resetArmorFourPoints(*(frame->image), armor, 0.3);

        // 显示装甲板分类信息
        if (Data::imshow_flag) {
            rm::displaySingleArmorClass(*(frame->image), armor);
        }
        
        // 显示装甲板投影信息
        if (Data::imshow_flag && Data::reprojection_flag) {
            rm::setReprojection(*(frame->image), *(frame->image), armor.four_points, armor.size);
        } else {
            rm::displaySingleArmorLine(*(frame->image), armor);
        }

        // 根据装甲板大小选择3D坐标
        std::vector<cv::Point3f> *Armor3D;
        if(armor.size == rm::ARMOR_SIZE_BIG_ARMOR) {
            Armor3D = BigArmor3D;
        } else if(armor.size == rm::ARMOR_SIZE_SMALL_ARMOR) {
            Armor3D = SmallArmor3D;
        } else {
            continue;
        }
        
        // 使用solvePnP求解旋转和平移参数
        try {
            cv::solvePnP(*Armor3D, armor.four_points,
                Data::camera[frame->camera_id]->intrinsic_matrix,
                Data::camera[frame->camera_id]->distortion_coeffs,
                rvec, tvec, false, cv::SOLVEPNP_EPNP);
        } catch (cv::Exception& e) {
            rm::message("solvePnP error", rm::MSG_ERROR);
            continue;
        }


        // 创建目标对象并设置参数
        rm::Target target;
        target.armor_id = armor_id;
        target.armor_size = armor.size;

        // 计算旋转矩阵和装甲板角度
        cv::Rodrigues(rvec, rotate_cv);
        rm::tf_Mat3d(rotate_cv, rotate_pnp);
        rotate_world = rotate_head2world * rotate_pnp2head * rotate_pnp;
        target.armor_yaw_world = rm::tf_rotation2armoryaw(rotate_world);

        // 计算装甲板世界坐标
        rm::tf_Vec4d(tvec, pose_pnp);
        pose_world = trans_head2world * trans_pnp2head * pose_pnp;
        target.pose_world = pose_world;


        // 从车库中获取目标
        ObjPtr objptr = garage->getObj(armor_id);
        objptr->push(target, frame->time_point);

        // 计算目标与图传的角度偏差
        double angle = rm::getAngleOffsetTargetToReferee(
            control->get_yaw(), control->get_pitch(),
            target.pose_world(0, 0), target.pose_world(1, 0), target.pose_world(2, 0),
            referee_offset[0], referee_offset[1], referee_offset[2], referee_offset[3], referee_offset[4],
            axis_offset[0], axis_offset[1], axis_offset[2]
        );

        // 更新攻击队列
        Data::attack->push(armor_id, angle, frame->time_point);
    }

    if (Data::image_flag) {
        if (Data::ui_flag) UI(frame);
        imshow(frame);
    }
    // if (Data::ui_flag) monitor(frame);
    
    return true;
}