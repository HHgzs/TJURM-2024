#ifndef RM2024_GARAGE_WRAPPER_OUTPOSTS_H_
#define RM2024_GARAGE_WRAPPER_OUTPOSTS_H_

#include "garage/interface.h"

class WrapperRune : public ObjInterface {
public:
    WrapperRune(rm::ArmorID id);
    ~WrapperRune() {}
    
    void push(const rm::Target& target, TimePoint t) override;
    void update() override {}
    bool getTarget(Eigen::Vector4d& pose, const double fly_delay, const double rotate_delay, const double shoot_delay) override;
    rm::ArmorSize getArmorSize() override;
    void getState(std::vector<std::string>& lines) override;
    void setState(int state) override {};

public:
    rm::RuneV2 rune_;

};

#endif