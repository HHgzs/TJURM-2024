#ifndef RM2024_THREADS_SERIAL_H_
#define RM2024_THREADS_SERIAL_H_
#include <mutex>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include "data_manager/base.h"
#include "data_manager/param.h"
#include "threads/control/structure.h"

#define SOF 0xA5

class Control {
public:
    static std::shared_ptr<Control> get_instance() {
        static std::shared_ptr<Control> instance(new Control());
        return instance;
    }
    double get_yaw() {
        return static_cast<double>(this->state_bytes_.input_data.curr_yaw);
    }
    double get_pitch() {
        return static_cast<double>(this->state_bytes_.input_data.curr_pitch);
    }
    double get_roll() {
        #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO) || defined(TJURM_SENTRY)
        return 0.0;
        #endif
        #ifdef TJURM_DRONSE
        return static_cast<double>(this->state_bytes_.input_data.curr_roll);
        #endif
    }
    rm::ArmorColor get_enemy() {
        return static_cast<rm::ArmorColor>(this->state_bytes_.input_data.enemy_color);
    }
    uint8_t get_state() {
        #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO) || defined(TJURM_DRONSE)
        return static_cast<uint8_t>(this->state_bytes_.input_data.state);
        #endif

        return 0x00;
    }
    double get_yaw_omega() {
        #if defined(TJURM_INFANTRY)
        return static_cast<double>(this->state_bytes_.input_data.curr_omega);
        #endif
        return 0.0;
    }

    rm::ArmorColor get_self() {
        rm::ArmorColor enemy_color = static_cast<rm::ArmorColor>(this->state_bytes_.input_data.enemy_color);
        if (enemy_color == rm::ARMOR_COLOR_BLUE) return rm::ARMOR_COLOR_RED;
        else if (enemy_color == rm::ARMOR_COLOR_RED) return rm::ARMOR_COLOR_BLUE;
    }

    char get_shoot_config() {
        #ifdef TJURM_SENTRY
        return static_cast<char>(this->state_bytes_.input_data.shoot_config);
        #endif
        return 0xFF;
    }

    bool get_autoaim() {
        #ifdef TJURM_SENTRY
        return true;
        #endif

        #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO) || defined(TJURM_DRONSE)
        return static_cast<bool>(this->state_bytes_.input_data.autoaim);
        #endif
    }

    void send_thread();
    void receive_thread();

    void autoaim();

    void send_single(double yaw, double pitch, bool fire, rm::ArmorID id = rm::ARMOR_ID_UNKNOWN);
    void stop_send() { send_flag_ = false; }
    void start_send() {
        send_flag_ = true;
        send_cv_.notify_all();
    }

    void message();
    void state();
    void shootspeed();

private:
    Control() {
        
        #ifdef TJURM_HERO
        this->state_bytes_.input_data.curr_speed = 15.8f;
        #endif

        #ifdef TJURM_SENTRY
        this->state_bytes_.input_data.shoot_config = 0xFF;
        #endif
    }
    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;


public:
    std::deque<std::pair<TimePoint, StateBytes>> state_queue_;

    StateBytes              state_bytes_;             // 电控 -> 自瞄
    OperateBytes            operate_bytes_;           // 自瞄 -> 电控

    int                     file_descriptor_;
    std::string             port_name_;

private:
    bool                    send_flag_ = true;
    std::condition_variable send_cv_;

    rm::CycleQueue<double>  yaw_history_;

};

#endif