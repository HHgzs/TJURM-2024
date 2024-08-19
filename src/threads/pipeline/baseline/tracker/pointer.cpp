#include "threads/pipeline.h"

using namespace rm;

static double roi_extend_w;
static double roi_extend_h;
static double point_line_dist;
static double point_radius_ratio;

static double lb_min_rect_side;
static double lb_max_rect_side;
static double lb_min_area;
static double lb_min_ratio_area;
static double lb_max_angle;

static double armor_max_ratio_length;
static double armor_max_ratio_area;
static double armor_min_ratio_side;
static double armor_max_ratio_side;
static double armor_max_angle_diff;
static double armor_max_angle_avg;
static double armor_max_offset;
static double armor_size_ratio;
static double armor_tower_size_ratio;
static double armor_min_area_percent;

static float big_red_width;
static float big_red_height;
static float small_red_width;
static float small_red_height;

static float big_blue_width;
static float big_blue_height;
static float small_blue_width;
static float small_blue_height;

static double binary_ratio;
static double enemy_split;

static std::vector<int> armor_class_map;
static std::vector<int> armor_color_map;

void Pipeline::init_pointer() {
    auto param = Param::get_instance();

    // 设置装甲板ID映射
    std::string      yolo_type      = (*param)["Model"]["YoloArmor"]["Type"];
    std::vector<int> temp_class_map = (*param)["Model"]["YoloArmor"][yolo_type]["ClassMap"];
    std::vector<int> temp_color_map = (*param)["Model"]["YoloArmor"][yolo_type]["ColorMap"];
    armor_class_map = temp_class_map;
    armor_color_map = temp_color_map;

    roi_extend_w           = (*param)["Points"]["Extend"]["ROIwidth"];
    roi_extend_h           = (*param)["Points"]["Extend"]["ROIheight"];
    point_line_dist        = (*param)["Points"]["Extend"]["PointLineDist"];
    point_radius_ratio     = (*param)["Points"]["Extend"]["PointRadiusRatio"];

    lb_min_rect_side       = (*param)["Points"]["Lightbar"]["MinRectSide"];
    lb_max_rect_side       = (*param)["Points"]["Lightbar"]["MaxRectSide"];
    lb_min_area            = (*param)["Points"]["Lightbar"]["MinArea"];
    lb_min_ratio_area      = (*param)["Points"]["Lightbar"]["MinRatioArea"];
    lb_max_angle           = (*param)["Points"]["Lightbar"]["MaxAngle"];

    armor_max_ratio_length = (*param)["Points"]["Armor"]["MaxRatioLength"];
    armor_max_ratio_area   = (*param)["Points"]["Armor"]["MaxRatioArea"];
    armor_min_ratio_side   = (*param)["Points"]["Armor"]["RatioSide"]["Min"];
    armor_max_ratio_side   = (*param)["Points"]["Armor"]["RatioSide"]["Max"];
    armor_max_angle_diff   = (*param)["Points"]["Armor"]["MaxAngleDiff"];
    armor_max_angle_avg    = (*param)["Points"]["Armor"]["MaxAngleAvg"];
    armor_max_offset       = (*param)["Points"]["Armor"]["MaxOffset"];
    armor_size_ratio       = (*param)["Points"]["Armor"]["SizeRatio"];
    armor_tower_size_ratio = (*param)["Points"]["Armor"]["TowerSizeRatio"];
    armor_min_area_percent = (*param)["Points"]["Armor"]["AreaPercent"];


    // 获取装甲板长宽
    big_red_width          = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Width"];
    big_red_height         = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Height"];
    small_red_width        = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Width"];
    small_red_height       = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Height"];

    // 获取装甲板长宽
    big_blue_width         = (*param)["Points"]["PnP"]["Blue"]["BigArmor"]["Width"];
    big_blue_height        = (*param)["Points"]["PnP"]["Blue"]["BigArmor"]["Height"];
    small_blue_width       = (*param)["Points"]["PnP"]["Blue"]["SmallArmor"]["Width"];
    small_blue_height      = (*param)["Points"]["PnP"]["Blue"]["SmallArmor"]["Height"];

    enemy_split = (*param)["Points"]["Threshold"]["EnemySplit"];
}

bool Pipeline::pointer(std::shared_ptr<rm::Frame> frame) {
    auto param = Param::get_instance();
    if(Data::enemy_color == rm::ARMOR_COLOR_RED) {
        binary_ratio = (*param)["Points"]["Threshold"]["RatioRed"];
    } else {
        binary_ratio = (*param)["Points"]["Threshold"]["RatioBlue"];
    }

    for (auto& yolo_rect : frame->yolo_list) {
        rm::Armor armor;
        armor.id = (rm::ArmorID)(armor_class_map[yolo_rect.class_id]);
        armor.color = (rm::ArmorColor)(armor_color_map[yolo_rect.color_id]);
        setArmorExtendRectIOU(armor, yolo_rect.box, frame->width, frame->height, roi_extend_w, roi_extend_h);
        armor.size = ARMOR_SIZE_SMALL_ARMOR;
        setArmorRectCenter(armor);


        #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO)
        if ((Data::state == 1) && (armor.id != rm::ARMOR_ID_TOWER)) continue;
        #endif

        if (!isRectValidInImage(*frame->image, armor.rect)) continue;
        cv::Mat roi = (*frame->image)(armor.rect);

        cv::Mat gray, binary;
        rm::getGrayScale(roi, gray, Data::enemy_color, rm::GRAY_SCALE_METHOD_CVT);

        // rm::getBinary(gray, binary, binary_ratio, rm::BINARY_METHOD_MAX_MIN_RATIO);

        int threshold_from_hist = rm::getThresholdFromHist(roi, 8, binary_ratio);
        threshold_from_hist = std::clamp(threshold_from_hist, 10, 100);
        rm::getBinary(gray, binary, threshold_from_hist, rm::BINARY_METHOD_DIRECT_THRESHOLD);

        if (Data::image_flag && Data::binary_flag) {
            cv::imshow("gray", gray);
            cv::imshow("binary", binary);
            cv::waitKey(1);
        }

        if (Data::image_flag && Data::histogram_flag) {
            cv::Mat showHist;
            rm::getThresholdFromHist(roi, showHist, 8, binary_ratio);
            cv::imshow("histogram", showHist);
            cv::waitKey(1);
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);


        std::vector<rm::Lightbar> lightbar_list;
        rm::getLightbarsFromContours(
            contours, 
            lightbar_list, 
            lb_min_rect_side,
            lb_max_rect_side,
            lb_min_area,
            lb_min_ratio_area,
            lb_max_angle);

        rm::LightbarPair best_pair;

        bool flag = rm::getBestMatchedLightbarPair(
            lightbar_list, 
            armor, 
            best_pair, 
            armor_max_ratio_length,
            armor_max_ratio_area,
            armor_min_ratio_side,
            armor_max_ratio_side,
            armor_max_angle_diff,
            armor_max_angle_avg,
            armor_max_offset);

        armor.color = rm::getArmorColorFromHSV(roi, best_pair);

        if (!flag) {
            if (Data::point_skip_flag) rm::message("No lightbar pair found", rm::MSG_NOTE);
            if (Data::image_flag && Data::ui_flag) {
                rm::displaySingleArmorClass(*(frame->image), armor);
                rm::displaySingleArmorRect(*(frame->image), armor);
            }
            continue;
        }

        bool color_skip_flag = false;
        
        #ifdef TJURM_SENTRY
        color_skip_flag = color_skip_flag || !rm::isArmorColorEnemy(roi, best_pair, Data::enemy_color, enemy_split);
        color_skip_flag = color_skip_flag || (armor.color != Data::enemy_color);
        #endif
        
        #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO) || defined(TJURM_DRONSE)
        color_skip_flag = color_skip_flag || (armor.color == Data::self_color);
        color_skip_flag = color_skip_flag || (armor.color == rm::ARMOR_COLOR_NONE);
        #endif
        

        if (Data::auto_enemy && color_skip_flag) {
            if (Data::point_skip_flag) rm::message("Color is on our part", rm::MSG_NOTE);
            if (Data::image_flag && Data::ui_flag) {
                rm::displaySingleArmorClass(*(frame->image), armor);
                rm::displaySingleArmorRect(*(frame->image), armor);
            }
            continue;
        }
        
        rm::setArmorFourPoints(
            armor, 
            findPointPairBarycenter(best_pair.first, gray, point_line_dist, point_radius_ratio),
            findPointPairBarycenter(best_pair.second, gray, point_line_dist, point_radius_ratio)
        );

        if (rm::isLightBarAreaPercentValid(armor, armor_min_area_percent)) {
            if (Data::point_skip_flag) rm::message("Area percent is invalid", rm::MSG_NOTE);
            if (Data::image_flag && Data::ui_flag) {
                rm::displaySingleArmorClass(*(frame->image), armor);
                rm::displaySingleArmorRect(*(frame->image), armor);
            }
            continue;
        }

        if(armor.four_points.size() != 4) {
            if (Data::point_skip_flag) rm::message("No four points found", rm::MSG_NOTE);
            if (Data::image_flag && Data::ui_flag) {
                rm::displaySingleArmorClass(*(frame->image), armor);
                rm::displaySingleArmorRect(*(frame->image), armor);
            }
            continue;
        }

        #if defined(TJURM_SENTRY) || defined(TJURM_DRONSE)
        if (armor.id == rm::ARMOR_ID_TOWER) setArmorSizeByPoints(armor, armor_tower_size_ratio);
        else setArmorSizeByPoints(armor, armor_size_ratio);
        #endif

        #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO)
        setArmorSizeByPoints(armor, armor_size_ratio);
        #endif

        frame->armor_list.push_back(armor);

        if (Data::image_flag && Data::ui_flag) {
            rm::displaySingleArmorClass(*(frame->image), armor);
            rm::displaySingleArmorRect(*(frame->image), armor);
        }
        
        if (Data::image_flag && Data::reprojection_flag && Data::ui_flag) {
            if(armor.color == rm::ARMOR_COLOR_RED) {
                rm::paramReprojection(small_red_width, small_red_height, big_red_width, big_red_height);
                rm::setReprojection(*(frame->image), *(frame->image), armor.four_points, armor.size);
            } else if (armor.color == rm::ARMOR_COLOR_BLUE) {
                rm::paramReprojection(small_blue_width, small_blue_height, big_blue_width, big_blue_height);
                rm::setReprojection(*(frame->image), *(frame->image), armor.four_points, armor.size);
            }
            
        } else if (Data::image_flag && Data::ui_flag) {
            rm::displaySingleArmorLine(*(frame->image), armor);
        }

    }

    if (frame->armor_list.size() == 0) {
        if (Data::point_skip_flag) rm::message("No armor found", rm::MSG_NOTE);
        
        if (Data::image_flag) imshow(frame);
        return false;
    }
    return true;
}