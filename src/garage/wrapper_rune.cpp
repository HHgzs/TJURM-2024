#include "garage/wrapper_rune.h"
#include "threads/pipeline.h"
using namespace rm;
using namespace std;


WrapperRune::WrapperRune(ArmorID id) : ObjInterface(id) {
    this->id_ = id;
    auto param = Param::get_instance();

    vector<double> runeSmallQ = (*param)["Kalman"]["Rune"]["SmallQ"];
    vector<double> runeSmallR = (*param)["Kalman"]["Rune"]["SmallR"];
    vector<double> runeBigQ = (*param)["Kalman"]["Rune"]["BigQ"];
    vector<double> runeBigR = (*param)["Kalman"]["Rune"]["BigR"];
    vector<double> runeSpdQ = (*param)["Kalman"]["Rune"]["SpdQ"];
    vector<double> runeSpdR = (*param)["Kalman"]["Rune"]["SpdR"];

    double   big_rune_fire_spd = (*param)["Kalman"]["Rune"]["BigRuneFireSpd"];                         // 大符开火角速度
    double   fire_after_trans_delay = (*param)["Kalman"]["Rune"]["FireAfterTransDelay"];               // 符切换后多久开火
    double   fire_flag_keep_delay = (*param)["Kalman"]["Rune"]["FireFlagKeepDelay"];                   // 开火信号保留时间
    double   fire_interval_delay = (*param)["Kalman"]["Rune"]["FireIntervalDelay"];                    // 两次开火间隔
    double   turn_to_center_delay = (*param)["Kalman"]["Rune"]["TureToCenterDelay"];                   // 模型保留时间

    rune_ = RuneV2();
    rune_.setSmallMatrixQ(runeSmallQ[0], runeSmallQ[1], runeSmallQ[2], runeSmallQ[3], runeSmallQ[4], runeSmallQ[5]);
    rune_.setSmallMatrixR(runeSmallR[0], runeSmallR[1], runeSmallR[2], runeSmallR[3], runeSmallR[4]);
    rune_.setBigMatrixQ(runeBigQ[0], runeBigQ[1], runeBigQ[2], runeBigQ[3], runeBigQ[4], runeBigQ[5], runeBigQ[6], runeBigQ[7]);
    rune_.setBigMatrixR(runeBigR[0], runeBigR[1], runeBigR[2], runeBigR[3], runeBigR[4]);
    rune_.setSpdMatrixQ(runeSpdQ[0], runeSpdQ[1]);
    rune_.setSpdMatrixR(runeSpdR[0]);
    rune_.setAutoFire(big_rune_fire_spd, fire_after_trans_delay, fire_flag_keep_delay, fire_interval_delay, turn_to_center_delay);
    rune_.setRuneType(false);
}

void WrapperRune::push(const Target& target, TimePoint t) {
    Eigen::Matrix<double, 5, 1> pose;
    pose << target.pose_world[0], target.pose_world[1], target.pose_world[2], target.armor_yaw_world, target.rune_angle;
    rune_.push(pose, t);
}

bool WrapperRune::getTarget(Eigen::Vector4d& pose, const double fly_delay, const double rotate_delay, const double shoot_delay) {
    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE)
    auto pipeline = Pipeline::get_instance();
    
    if (Data::auto_rune) {
        if (Data::state == 3) rune_.setRuneType(true);
        else rune_.setRuneType(false);
    } else if (Data::manu_rune) {
        if (Data::big_rune) rune_.setRuneType(true);
        else rune_.setRuneType(false);
    }
    #endif
    
    pose = rune_.getPose(fly_delay + rotate_delay);
    return rune_.getFireFlag(fly_delay + rotate_delay);
    
}

rm::ArmorSize WrapperRune::getArmorSize() {
    return rm::ARMOR_SIZE_UNKNOWN;
}

void WrapperRune::getState(vector<string>& lines) {
    lines.clear();
    rune_.getStateStr(lines);
}