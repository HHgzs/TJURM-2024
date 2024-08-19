#include "garage/garage.h"
#include "threads/control.h"
#include "threads/pipeline.h"
#include <thread>
#include <cmath>
#include <fstream>
using namespace rm;

static double shoot_speed, shoot_delay;
static double start_fire_delay;
static int iteration_num;
static double base_to_far_dist, far_to_base_dist;

static bool fire = false;
static double target_yaw, target_pitch, fly_delay, delay;
static double rotate_delay, rotate_delay_outpost, rotate_delay_rune;
static Eigen::Vector4d pose;

static TimePoint start_autoaim;
static bool last_autoaim = false;

static std::ofstream speed_file;
static bool speed_write_flag;
static rm::SpeedQueue<float> speed_queue(3, 15.75f, {0.5, 0.3, 0.2});


static void init_send() {
    auto param = Param::get_instance();
    shoot_speed = (*param)["Car"]["ShootSpeed"];
    shoot_delay = (*param)["Car"]["ShootDelay"];
    rotate_delay = (*param)["Car"]["RotateDelay"];
    rotate_delay_outpost = (*param)["Car"]["RotateDelayOutpost"];
    rotate_delay_rune = (*param)["Car"]["RotateDelayRune"];
    start_fire_delay = (*param)["Car"]["StartFireDelay"];
    iteration_num = (*param)["Kalman"]["IterationNum"];
    base_to_far_dist = (*param)["Camera"]["Switch"]["BaseToFarDist"];
    far_to_base_dist = (*param)["Camera"]["Switch"]["FarToBaseDist"];
    speed_write_flag = (*param)["Debug"]["SaveSpeed"]["SpeedWrite"];

    std::string speed_save_path = (*param)["Debug"]["SaveSpeed"]["SavePath"];
    if (speed_write_flag) {
        speed_file.open(speed_save_path, std::ios_base::app);
    }
}

void Control::message() {
    rm::message_send();
    rm::message("system state", Data::state);
    rm::message("system yaw", get_yaw());
    rm::message("system pit", get_pitch());
    rm::message("system rol", get_roll());
    rm::message("system omg", get_yaw_omega());
    rm::message("target id", Data::target_id);
    rm::message("target yaw", target_yaw);
    rm::message("target pit", target_pitch);
    rm::message("target -x-", pose(0, 0));
    rm::message("target -y-", pose(1, 0));
    rm::message("target -z-", pose(2, 0));
    rm::message("target -0-", pose(3, 0) * 180 / M_PI);
    rm::message("target fire", fire);
    rm::message("enemy color", Data::enemy_color);
    rm::message("camera id", Data::camera_index);
}

void Control::state() {
    auto pipeline = Pipeline::get_instance();
    auto garage = Garage::get_instance();

    // 通过电控获取敌方颜色
    Data::enemy_color = get_enemy();
    Data::self_color = (Data::enemy_color == rm::ARMOR_COLOR_BLUE) ? rm::ARMOR_COLOR_RED : rm::ARMOR_COLOR_BLUE;

    // 确定自瞄状态，开始录制
    if (Data::auto_capture && !get_autoaim()) pipeline->start_record();
    else if (!Data::auto_capture && Data::manu_capture) pipeline->start_record();
    else pipeline->stop_record();

    // 确定自瞄状态，记录开始自瞄时间点
    if (!last_autoaim && get_autoaim()) {
        start_autoaim = getTime();
        Data::attack->clear();
    }
    last_autoaim = get_autoaim();

    // 获取攻击目标
    if(Data::armor_mode) Data::target_id = Data::attack->pop();
    else if (Data::rune_mode) Data::target_id = rm::ARMOR_ID_RUNE;
    else Data::target_id = rm::ARMOR_ID_UNKNOWN;
    
    Data::state = get_state();
    // Data::state = 2;

    #ifdef TJURM_SENTRY
    if (Data::target_id != rm::ARMOR_ID_TOWER) {
        Data::camera_index = Data::camera_base;
        rm::message("camera type", 'B');
    } else if (Data::target_dist > base_to_far_dist) {
        Data::camera_index = Data::camera_far;
        rm::message("camera type", 'F');
    } else if (Data::target_dist < far_to_base_dist) {
        Data::camera_index = Data::camera_base;
        rm::message("camera type", 'B');
    }
    #endif

    #ifdef TJURM_SENTRY
    rm::message("shoot config", (int)get_shoot_config());
    Data::attack->setValidID(get_shoot_config());
    #endif

    // 更新自瞄状态
    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE)
    if (Data::auto_rune) {
        if (Data::state == 0 || Data::state == 1) pipeline->switch_rune_to_armor();
        else if (Data::state == 2 || Data::state == 3) pipeline->switch_armor_to_rune();
        else pipeline->switch_rune_to_armor();
    } else if (Data::manu_rune) {
        pipeline->switch_armor_to_rune();
    } else {
        pipeline->switch_rune_to_armor();
    }
    #endif

    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE)
    if (Data::rune_mode)       rotate_delay = rotate_delay_rune;
    else if (Data::state == 1) rotate_delay = rotate_delay_outpost;
    else                       rotate_delay = rotate_delay;
    #endif
}

void Control::shootspeed() {
    
    #ifdef TJURM_HERO
    
    float curr_speed = this->state_bytes_.input_data.curr_speed;
    float last_speed = speed_queue.back();

    if ((last_speed != curr_speed) && (curr_speed != 15.75f) && speed_write_flag) {
        speed_file << std::fixed << std::setprecision(10) << curr_speed << "     ";
        speed_file << std::setprecision(10) << target_yaw << " " << std::setprecision(10) << target_pitch <<std::endl;
    }
    
    if (curr_speed != 15.75f) speed_queue.push(curr_speed);

    float avg_speed = speed_queue.pop();
    avg_speed = std::clamp(avg_speed, 14.0f, 16.0f);

    operate_bytes_.output_data.avg_speed = avg_speed;
    shoot_speed = avg_speed;
    operate_bytes_.output_data.food = 0x01;

    rm::message("shoot speed", avg_speed);

    #endif
}

void Control::send_thread() {
    auto garage = Garage::get_instance();
    auto pipeline = Pipeline::get_instance();

    init_send();
    
    std::mutex mutex;
    while(true) {
        if(!send_flag_) {
            std::unique_lock<std::mutex> lock(mutex);
            send_cv_.wait(lock, [this]{return send_flag_;});
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(Data::send_wait_time));

        this->message();            // 统一终端发消息
        this->state();              // 根据串口更新状态
        this->shootspeed();         // 英雄弹速寄存器

        // 根据目标id判断是否需要自瞄
        if(Data::target_id == rm::ARMOR_ID_UNKNOWN) {
            #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_DRONSE)
            continue;
            #endif

            #ifdef TJURM_HERO
            send_single(get_yaw(), get_pitch(), false);
            continue;
            #endif

            #ifdef TJURM_SENTRY
            float camsense_x = this->state_bytes_.input_data.target_pose[0];
            float camsense_y = this->state_bytes_.input_data.target_pose[1];
            float camsense_z = this->state_bytes_.input_data.target_pose[2];
            if(abs(camsense_x) < 1e-2 && abs(camsense_y) < 1e-2) continue;

            getFlyDelay(target_yaw, target_pitch, shoot_speed, camsense_x, camsense_y, camsense_z);
            send_single(target_yaw, target_pitch, false);
            continue;

            #endif
        }

        // 迭代法求解击打 yaw, pitch
        auto objptr = garage->getObj(Data::target_id);
        objptr->getTarget(pose, 0.0, 0.0, 0.0);
        for(int i = 0; i < iteration_num; i++) {
            fly_delay = getFlyDelay(target_yaw, target_pitch, shoot_speed, pose(0, 0), pose(1, 0), pose(2, 0));
            fire = objptr->getTarget(pose, fly_delay, rotate_delay, shoot_delay);
        }
        rm::message("target pitch b", target_pitch);
        Data::target_dist = sqrt(pow(pose(0, 0), 2) + pow(pose(1, 0), 2) + pow(pose(2, 0), 2));
        
        // 如果返回坐标为0, 确定控制信号
        if ((std::abs(pose[0]) < 1e-2) && (std::abs(pose[1]) < 1e-2)) {
            #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_DRONSE)
            continue;
            #endif

            #ifdef TJURM_HERO
            send_single(get_yaw(), get_pitch(), false);
            continue;
            #endif

            #ifdef TJURM_SENTRY
            float camsense_x = this->state_bytes_.input_data.target_pose[0];
            float camsense_y = this->state_bytes_.input_data.target_pose[1];
            float camsense_z = this->state_bytes_.input_data.target_pose[2];
            if(abs(camsense_x) < 1e-2 && abs(camsense_y) < 1e-2) continue;

            getFlyDelay(target_yaw, target_pitch, shoot_speed, camsense_x, camsense_y, camsense_z);
            send_single(target_yaw, target_pitch, false);
            continue;

            #endif
        }

        // 控制发弹
        bool start_delay_flag = (getDoubleOfS(start_autoaim, getTime()) > start_fire_delay);
        bool autoaim_flag = get_autoaim();

        fire = (fire && start_delay_flag && autoaim_flag && Data::auto_fire);
        send_single(target_yaw, target_pitch, fire, Data::target_id);
    }
}