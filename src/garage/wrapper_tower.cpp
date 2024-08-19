#include "garage/wrapper_tower.h"
using namespace rm;
using namespace std;


WrapperTower::WrapperTower(ArmorID id) : ObjInterface(id) {
    id_ = id;
    size_ = ARMOR_SIZE_BIG_ARMOR;
    auto param = Param::get_instance();

    double track_count = (*param)["Kalman"]["TrackQueue"]["Count"];
    double track_dist = (*param)["Kalman"]["TrackQueue"]["Distance"];
    double track_delay = (*param)["Kalman"]["TrackQueue"]["Delay"];
    int outpost_fire_update = (*param)["Kalman"]["Outpost"]["FireUpdate"];
    double outpost_fire_delay = (*param)["Kalman"]["Outpost"]["FireDelay"];
    double outpost_fire_angle = (*param)["Kalman"]["Outpost"]["FireAngle"]["Armor"];
    double outpost_fire_angle_small_ = (*param)["Kalman"]["Outpost"]["FireAngle"]["Center"];
    
    vector<double> outpostQ = (*param)["Kalman"]["Outpost"]["Q"];
    vector<double> outpostR = (*param)["Kalman"]["Outpost"]["R"];

    vector<double> outpostOmegaQ = (*param)["Kalman"]["Outpost"]["OmegaQ"];
    vector<double> outpostOmegaR = (*param)["Kalman"]["Outpost"]["OmegaR"];
    
    vector<double> trackqueueQ = (*param)["Kalman"]["TrackQueue"]["TowerQ"];
    vector<double> trackqueueR = (*param)["Kalman"]["TrackQueue"]["TowerR"];

    track_queue_ = TrackQueueV3(track_count, track_dist, track_delay);
    track_queue_.setMatrixQ(
        trackqueueQ[0], trackqueueQ[1], trackqueueQ[2], trackqueueQ[3], trackqueueQ[4], trackqueueQ[5],
        trackqueueQ[6], trackqueueQ[7], trackqueueQ[8], trackqueueQ[9], trackqueueQ[10]);
    track_queue_.setMatrixR(trackqueueR[0], trackqueueR[1], trackqueueR[2], trackqueueR[3]);
    
    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO) || defined(TJURM_SENTRY)
    outpost_ = OutpostV1();
    outpost_.setMatrixQ(outpostQ[0], outpostQ[1], outpostQ[2], outpostQ[3], outpostQ[4]);
    #endif    

    #if defined(TJURM_DRONSE)
    outpost_ = OutpostV2();
    outpost_.setMatrixQ(outpostQ[0], outpostQ[1], outpostQ[2], outpostQ[3], outpostQ[4], outpostQ[5], outpostQ[6], outpostQ[7]);
    #endif

    outpost_.setMatrixR(outpostR[0], outpostR[1], outpostR[2], outpostR[3]);
    outpost_.setMatrixOmegaQ(outpostOmegaQ[0], outpostOmegaQ[1]);
    outpost_.setMatrixOmegaR(outpostOmegaR[0]);
    outpost_.setFireValue(outpost_fire_update, outpost_fire_delay, outpost_fire_angle, outpost_fire_angle_small_);

}

void WrapperTower::push(const Target& target, TimePoint t) {
    Eigen::Vector4d pose(
        target.pose_world[0], target.pose_world[1], target.pose_world[2], target.armor_yaw_world
    );
    track_queue_.push(pose, t);
}

void WrapperTower::update() {
    track_queue_.update();

    Eigen::Vector4d pose;
    TimePoint t;
    if (!track_queue_.getPose(pose, t)) return;

    outpost_.push(pose,t);
}

bool WrapperTower::getTarget(Eigen::Vector4d& pose_rotate, const double fly_delay, const double rotate_delay, const double shoot_delay) {
    
    pose_rotate = track_queue_.getPose(fly_delay + rotate_delay);
    Eigen::Vector4d pose_shoot = track_queue_.getPose(fly_delay + shoot_delay);

    Data::target_omega = outpost_.getOmega();
    rm::message("target omg", Data::target_omega);

    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE)
    if (Data::state == 0) {
        rm::message("mode", 'T');
        return track_queue_.getFireFlag();
    } else if (Data::state == 1) {
        rm::message("mode", 'A');
        pose_shoot = outpost_.getPose(fly_delay + shoot_delay);
        pose_rotate = outpost_.getPose(fly_delay + rotate_delay);
        return outpost_.getFireArmor(pose_shoot);
    }
    #endif

    #if defined(TJURM_DRONSE) || defined(TJURM_SENTRY)
    rm::message("mode", 'A');
    pose_shoot = outpost_.getPose(fly_delay + shoot_delay);
    pose_rotate = outpost_.getPose(fly_delay + rotate_delay);
    return outpost_.getFireArmor(pose_shoot);
    #endif

    #ifdef TJURM_HERO
    if (Data::state == 1) {
        rm::message("mode", 'C');
        pose_shoot = outpost_.getCenter(fly_delay + shoot_delay);
        pose_rotate = outpost_.getCenter(0.0);
        return outpost_.getFireCenter(pose_shoot);
    }
    #endif

    return track_queue_.getFireFlag();
}

rm::ArmorSize WrapperTower::getArmorSize() {
    return rm::ARMOR_SIZE_UNKNOWN;
}

void WrapperTower::getState(std::vector<std::string>& lines) {
    lines.clear();
    outpost_.getStateStr(lines);
    track_queue_.getStateStr(lines);
}