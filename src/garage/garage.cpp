#include "garage/garage.h"
#include "garage/wrapper_car.h"
#include "garage/wrapper_rune.h"
#include "garage/wrapper_tower.h"

static int objmap[10] = {0, 1, 2, 3, 4, 5, 6, 7, 6, 7};

using namespace rm;
using namespace std;

Garage::Garage() {
    obj_ = vector<ObjPtr>(8);

    obj_[0] = make_shared<WrapperCar>(ARMOR_ID_SENTRY);
    obj_[1] = make_shared<WrapperCar>(ARMOR_ID_HERO);
    obj_[2] = make_shared<WrapperCar>(ARMOR_ID_ENGINEER);
    obj_[3] = make_shared<WrapperCar>(ARMOR_ID_INFANTRY_3);
    obj_[4] = make_shared<WrapperCar>(ARMOR_ID_INFANTRY_4);
    obj_[5] = make_shared<WrapperCar>(ARMOR_ID_INFANTRY_5);
    obj_[6] = make_shared<WrapperTower>(ARMOR_ID_TOWER);
    obj_[7] = make_shared<WrapperRune>(ARMOR_ID_RUNE);
}

ObjPtr Garage::getObj(rm::ArmorID id) {
    int index = objmap[id];
    return this->obj_[index];
}