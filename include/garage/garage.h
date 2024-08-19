#ifndef RM2024_GARAGE_GARAGE_H_
#define RM2024_GARAGE_GARAGE_H_

#include "data_manager/base.h"
#include "data_manager/param.h"
#include "garage/interface.h"

class Garage {
public:
    static std::shared_ptr<Garage> get_instance() {
        static std::shared_ptr<Garage> instance(new Garage());
        return instance;
    }
    
    ObjPtr getObj(rm::ArmorID id);

private:
    Garage();
    Garage(const Garage&) = delete;
    Garage& operator=(const Garage&) = delete;

public:
    std::vector<ObjPtr> obj_;
    
};

#endif