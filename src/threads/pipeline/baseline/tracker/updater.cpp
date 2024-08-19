#include "threads/pipeline.h"
#include "threads/control.h"
using namespace rm;

static std::vector<double> referee_offset;
static std::vector<double> axis_offset;

void Pipeline::init_updater() {
    auto param = Param::get_instance();
    std::vector<double> temp_referee_offset = (*param)["Car"]["RefereeOffset"];
    std::vector<double> temp_axis_offset    = (*param)["Car"]["AxisOffset"];
    referee_offset = temp_referee_offset;
    axis_offset    = temp_axis_offset;
}

bool Pipeline::updater(std::shared_ptr<rm::Frame> frame) {
    auto garage = Garage::get_instance();
    auto param = Param::get_instance();
    auto control = Control::get_instance();

    for(auto& target : frame->target_list) {
        rm::ArmorID armor_id = target.armor_id;
        ObjPtr objptr = garage->getObj(armor_id);
        objptr->push(target, frame->time_point);

        double angle = rm::getAngleOffsetTargetToReferee(
            control->get_yaw(), control->get_pitch(),
            target.pose_world(0, 0), target.pose_world(1, 0), target.pose_world(2, 0),
            referee_offset[0], referee_offset[1], referee_offset[2], referee_offset[3], referee_offset[4],
            axis_offset[0], axis_offset[1], axis_offset[2]
        );

        Data::attack->push(armor_id, angle, frame->time_point);
    }

    for (auto& objptr : garage->obj_)
        objptr->update();
    
    if ((frame->target_list).empty()) return false;
    return true;
}