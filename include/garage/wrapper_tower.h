#ifndef RM2024_GARAGE_WRAPPER_TOWER_H_
#define RM2024_GARAGE_WRAPPER_TOWER_H_

#include "garage/interface.h"

class WrapperTower : public ObjInterface {
    
public:
    WrapperTower(rm::ArmorID id);
    ~WrapperTower() {}
    void push(const rm::Target& target, TimePoint t) override;
    void update() override;
    bool getTarget(Eigen::Vector4d& pose, const double fly_delay, const double rotate_delay, const double shoot_delay) override;
    rm::ArmorSize getArmorSize() override;
    void getState(std::vector<std::string>& lines) override;
    void setState(int state) override {};
    
public:
    rm::TrackQueueV3 track_queue_;

    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO) || defined(TJURM_SENTRY)
    rm::OutpostV1 outpost_;
    #endif

    #if defined(TJURM_DRONSE)
    rm::OutpostV2 outpost_;
    #endif

};




#endif