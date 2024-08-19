#include "garage/wrapper_car.h"
using namespace rm;
using namespace std;

WrapperCar::WrapperCar(ArmorID id) : ObjInterface(id) {
    this->id_ = id;
    auto param = Param::get_instance();

    double track_count = (*param)["Kalman"]["TrackQueue"]["Count"];
    double track_dist = (*param)["Kalman"]["TrackQueue"]["Distance"];
    double track_delay = (*param)["Kalman"]["TrackQueue"]["Delay"];

    double antitop_min_r = (*param)["Kalman"]["Antitop"]["MinR"];
    double antitop_max_r = (*param)["Kalman"]["Antitop"]["MaxR"];

    int antitop_fire_update = (*param)["Kalman"]["Antitop"]["FireUpdate"];
    double antitop_fire_delay = (*param)["Kalman"]["Antitop"]["FireDelay"];
    double antitop_fire_angle = (*param)["Kalman"]["Antitop"]["FireAngle"]["Armor"];
    double antitop_fire_angle_big_ = (*param)["Kalman"]["Antitop"]["FireAngle"]["CenterBig"];
    double antitop_fire_angle_small_ = (*param)["Kalman"]["Antitop"]["FireAngle"]["CenterSmall"];

    vector<double> antitopQ = (*param)["Kalman"]["Antitop"]["Q"];
    vector<double> antitopR = (*param)["Kalman"]["Antitop"]["R"];
    vector<double> antitopCenterQ = (*param)["Kalman"]["Antitop"]["CenterQ"];
    vector<double> antitopCenterR = (*param)["Kalman"]["Antitop"]["CenterR"];

    vector<double> antitopOmegaQ = (*param)["Kalman"]["Antitop"]["OmegaQ"];
    vector<double> antitopOmegaR = (*param)["Kalman"]["Antitop"]["OmegaR"];
    vector<double> antitopBalanceOmegaQ = (*param)["Kalman"]["Antitop"]["BalanceOmegaQ"];
    vector<double> antitopBalanceOmegaR = (*param)["Kalman"]["Antitop"]["BalanceOmegaR"];

    vector<double> trackqueueQ = (*param)["Kalman"]["TrackQueue"]["CarQ"];
    vector<double> trackqueueR = (*param)["Kalman"]["TrackQueue"]["CarR"];

    track_to_antitop_ = (*param)["Kalman"]["Switch"]["TrackToAntitop"];
    antitop_to_track_ = (*param)["Kalman"]["Switch"]["AntitopToTrack"];

    armor_to_center_ = (*param)["Kalman"]["Switch"]["ArmorToCenter"];
    center_to_armor_ = (*param)["Kalman"]["Switch"]["CenterToArmor"];

    track_queue_ = TrackQueueV4(track_count, track_dist, track_delay);
    antitop_4_ = new AntitopV3(antitop_min_r, antitop_max_r, 4);
    antitop_2_ = new AntitopV3(antitop_min_r, antitop_max_r, 2);

    track_queue_.setMatrixQ(
        trackqueueQ[0], trackqueueQ[1], trackqueueQ[2], trackqueueQ[3], trackqueueQ[4], trackqueueQ[5],
        trackqueueQ[6], trackqueueQ[7]);
    track_queue_.setMatrixR(trackqueueR[0], trackqueueR[1], trackqueueR[2]);

    antitop_4_->setMatrixQ(
        antitopQ[0], antitopQ[1], antitopQ[2], antitopQ[3], antitopQ[4], 
        antitopQ[5], antitopQ[6], antitopQ[7], antitopQ[8]);
    antitop_4_->setMatrixR(antitopR[0], antitopR[1], antitopR[2], antitopR[3]);

    antitop_4_->setCenterMatrixQ(antitopCenterQ[0], antitopCenterQ[1], antitopCenterQ[2], antitopCenterQ[3]);
    antitop_4_->setCenterMatrixR(antitopCenterR[0], antitopCenterR[1]);

    antitop_4_->setOmegaMatrixQ(antitopOmegaQ[0], antitopOmegaQ[1], antitopOmegaQ[2]);
    antitop_4_->setOmegaMatrixR(antitopOmegaQ[0]);
    
    if (id_ == rm::ARMOR_ID_HERO)
        antitop_4_->setFireValue(antitop_fire_update, antitop_fire_delay, antitop_fire_angle, antitop_fire_angle_big_);
    else
        antitop_4_->setFireValue(antitop_fire_update, antitop_fire_delay, antitop_fire_angle, antitop_fire_angle_small_);


    antitop_2_->setMatrixQ(
        antitopQ[0], antitopQ[1], antitopQ[2], antitopQ[3], antitopQ[4], 
        antitopQ[5], antitopQ[6], antitopQ[7], antitopQ[8]);
    antitop_2_->setMatrixR(antitopR[0], antitopR[1], antitopR[2], antitopR[3]);

    antitop_2_->setCenterMatrixQ(antitopCenterQ[0], antitopCenterQ[1], antitopCenterQ[2], antitopCenterQ[3]);
    antitop_2_->setCenterMatrixR(antitopCenterR[0], antitopCenterR[1]);

    antitop_2_->setOmegaMatrixQ(antitopBalanceOmegaQ[0], antitopBalanceOmegaQ[1], antitopBalanceOmegaQ[2]);
    antitop_2_->setOmegaMatrixR(antitopBalanceOmegaR[0]);
    
    antitop_2_->setFireValue(antitop_fire_update, antitop_fire_delay, antitop_fire_angle, antitop_fire_angle_big_);

}

void WrapperCar::push(const Target& target, TimePoint t) {
    Eigen::Vector4d pose(
        target.pose_world[0], target.pose_world[1], target.pose_world[2], target.armor_yaw_world
    );
    track_queue_.push(pose, t);

    curr_armor_num_++;
    if (target.armor_size == ARMOR_SIZE_BIG_ARMOR) {
        armor_size_count_ += 1;
    }
    
}

void WrapperCar::update() {
    track_queue_.update();

    if (curr_armor_num_ > 1) {
        armor_size_count_  -= 1;
    }
    curr_armor_num_ = 0;

    Eigen::Vector4d pose;
    TimePoint t;
    if (!track_queue_.getPose(pose, t)) return;

    rm::AntitopV3* antitop = nullptr;
    if (id_ == rm::ARMOR_ID_INFANTRY_3 || id_ == rm::ARMOR_ID_INFANTRY_4 || id_ == rm::ARMOR_ID_INFANTRY_5) {

        if (armor_size_count_ > 0) {
            antitop = antitop_2_;
            rm::message("antitop armor", 2);
        }
        else {
            antitop = antitop_4_;
            rm::message("antitop armor", 4);
        }
    } else {
        antitop = antitop_4_;
        rm::message("antitop armor", 4);
    }

    antitop->push(pose, t);
}

bool WrapperCar::getTarget(Eigen::Vector4d& pose_rotate, const double fly_delay, const double rotate_delay, const double shoot_delay) {
    rm::AntitopV3* antitop = nullptr;

    if (id_ == rm::ARMOR_ID_INFANTRY_3 || id_ == rm::ARMOR_ID_INFANTRY_4 || id_ == rm::ARMOR_ID_INFANTRY_5) {

        if (armor_size_count_ > 0) {
            antitop = antitop_2_;
            rm::message("antitop armor", 2);
        }
        else {
            antitop = antitop_4_;
            rm::message("antitop armor", 4);
        }
    } else {
        antitop = antitop_4_;
        rm::message("antitop armor", 4);
    }


    Eigen::Vector4d pose_shoot = track_queue_.getPose(fly_delay + shoot_delay);
    pose_rotate = track_queue_.getPose(fly_delay + rotate_delay);

    Data::target_omega = antitop->getOmega();
    rm::message("target omg", Data::target_omega);


    if(abs(Data::target_omega) > track_to_antitop_) flag_antitop_ = true;
    else if(abs(Data::target_omega) < antitop_to_track_) flag_antitop_ = false; 

    if(abs(Data::target_omega) > armor_to_center_) flag_center_ = true;
    else if(abs(Data::target_omega) < center_to_armor_) flag_center_ = false;

    if (flag_antitop_) {
        if(flag_center_) {
            rm::message("mode", 'C');
            pose_shoot = antitop->getCenter(fly_delay + shoot_delay);
            pose_rotate = antitop->getCenter(fly_delay + rotate_delay);
            return antitop->getFireCenter(pose_shoot);
        } else {
            rm::message("mode", 'A');
            pose_shoot = antitop->getPose(fly_delay + shoot_delay);
            pose_rotate = antitop->getPose(fly_delay + rotate_delay);

            return antitop->getFireArmor(pose_shoot);
        }      
    } else {
        rm::message("mode", 'T');
        return true;
    }
    
}

rm::ArmorSize WrapperCar::getArmorSize() {
    double abs_big, abs_small;
    switch (id_) {
        case rm::ARMOR_ID_SENTRY:
            size_ = rm::ARMOR_SIZE_SMALL_ARMOR;
            return rm::ARMOR_SIZE_SMALL_ARMOR;
            
        case rm::ARMOR_ID_HERO:
            size_ = rm::ARMOR_SIZE_BIG_ARMOR;
            return rm::ARMOR_SIZE_BIG_ARMOR;

        case rm::ARMOR_ID_INFANTRY_3:
        case rm::ARMOR_ID_INFANTRY_4:
        case rm::ARMOR_ID_INFANTRY_5:

            if (armor_size_count_ > 0) {
                size_ = rm::ARMOR_SIZE_BIG_ARMOR;
                return rm::ARMOR_SIZE_BIG_ARMOR;
            } else {
                size_ = rm::ARMOR_SIZE_SMALL_ARMOR;
                return rm::ARMOR_SIZE_SMALL_ARMOR;
            }

        default:
            size_ = rm::ARMOR_SIZE_UNKNOWN;
            return rm::ARMOR_SIZE_UNKNOWN;
    }
}

void WrapperCar::getState(std::vector<std::string>& lines) {
    lines.clear();

    if (id_ == rm::ARMOR_ID_INFANTRY_3 || id_ == rm::ARMOR_ID_INFANTRY_4 || id_ == rm::ARMOR_ID_INFANTRY_5) {
        if (armor_size_count_ > 0) antitop_2_->getStateStr(lines);
        else antitop_4_->getStateStr(lines);
    } else antitop_4_->getStateStr(lines);
    track_queue_.getStateStr(lines);
}