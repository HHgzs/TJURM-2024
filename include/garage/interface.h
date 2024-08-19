#ifndef RM2024_GARAGE_INTERFACE_H_
#define RM2024_GARAGE_INTERFACE_H_

#include "data_manager/base.h"
#include "data_manager/param.h"

class ObjInterface {
public:
    
    ObjInterface() {};
    ObjInterface(rm::ArmorID id) : id_(id) {};
    ~ObjInterface() {};

    virtual void push(const rm::Target& target, TimePoint t) = 0;
    virtual void update() = 0;
    virtual bool getTarget(Eigen::Vector4d& pose, const double fly_delay, const double rotate_delay, const double shoot_delay) = 0;
    virtual rm::ArmorSize getArmorSize() = 0;
    virtual void getState(std::vector<std::string>& lines) = 0;
    virtual void setState(int state) = 0;

    void setArmorSize(rm::ArmorSize size) {
        this->size_ = size;
    }

public:
    TimePoint last_t_;
    rm::ArmorSize size_ = rm::ARMOR_SIZE_UNKNOWN;
    rm::ArmorID id_ = rm::ARMOR_ID_UNKNOWN;
};

using ObjPtr = std::shared_ptr<ObjInterface>;

#endif