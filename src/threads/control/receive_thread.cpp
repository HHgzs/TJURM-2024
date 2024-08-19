#include "threads/control.h"
#include "threads/control/crc.h"

#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>

using namespace rm;

void Control::receive_thread() {
    rm::initSerialHead(file_descriptor_, sizeof(StateBytes), SOF);
    char buffer[400];
    StateBytes state_buffer;


    int status = 0;
    TimePoint tp = getTime();
    while (true) {
        double dt = getDoubleOfS(tp, getTime());
        if(dt > 1.0 && Data::timeout_flag) {
            rm::message("Control error: timeout", rm::MSG_ERROR);
            exit(-1);
        }

        if(status != 0) {
            rm::message("Control error: " + std::to_string(status), rm::MSG_ERROR);
            if (access(port_name_.c_str(), F_OK) < 0) {
                init_serial();
                status = 0;
            } else {
                status = (int)rm::restartSerialPort(file_descriptor_, port_name_);
            }
        }

        memset(buffer, 0, sizeof(buffer));
        status = (int)rm::readFromSerialPort((uint8_t*)buffer, sizeof(StateBytes) + 1, file_descriptor_);
        if (status) {
            rm::message("Control error: " + std::to_string(status), rm::MSG_ERROR);
            continue;
        }
        
        if ((unsigned char)buffer[0] != SOF) {
            status = (int)rm::initSerialHead(file_descriptor_, sizeof(StateBytes), SOF);
            continue;
        }

        if (!verify_crc16_check_sum((uint8_t*)buffer, sizeof(StateBytes))) {
            rm::message("Control error: crc16 error", rm::MSG_ERROR);
            continue;
        }

        if (buffer[sizeof(StateBytes)] == '\n') {
            memcpy(&state_buffer, buffer, sizeof(StateBytes));  
        } else {
            rm::message("Control error: data error", rm::MSG_ERROR);
            continue;
        }

        if (Data::state_delay_flag) {
            std::pair<TimePoint, StateBytes> p = std::make_pair(getTime(), state_buffer);
            state_queue_.push_back(p);
            if (state_queue_.size() > Data::state_queue_size) {
                state_queue_.pop_front();
            }

            auto it = state_queue_.begin();
            for (; it != state_queue_.end(); ++it) {
                if (getDoubleOfS(it->first, getTime()) < Data::state_delay_time) {
                    this->state_bytes_ = it->second;
                    break;
                }
            }
            if (it == state_queue_.end()) {
                rm::message("Control error: state delay error", rm::MSG_ERROR);
                this->state_bytes_ = state_buffer;
            }

        } else {
            this->state_bytes_ = state_buffer;
        }

        Data::yaw   = get_yaw();
        Data::pitch = get_pitch();
        Data::roll  = get_roll();

        #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE)
        Data::yaw_omega = get_yaw_omega();
        #endif

        tp = getTime();
    }
}