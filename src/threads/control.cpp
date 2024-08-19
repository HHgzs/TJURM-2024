#include "garage/garage.h"
#include "threads/control.h"
#include "threads/control/crc.h"

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <cstring>
#include <unistd.h>

using namespace rm;

void Control::send_single(double yaw, double pitch, bool fire, rm::ArmorID id) {
    if (!Data::serial_flag) return;
    
    operate_bytes_.frame_header.sof = SOF;
    operate_bytes_.output_data.shoot_yaw = static_cast<float>(yaw);
    operate_bytes_.output_data.shoot_pitch = static_cast<float>(pitch);
    operate_bytes_.output_data.fire = fire;

    #if defined(TJURM_INFANTRY)
    operate_bytes_.output_data.target_id = static_cast<uint8_t>(id);
    #endif

    append_crc16_check_sum((uint8_t*)&operate_bytes_, sizeof(OperateBytes));
    int status = (int)rm::writeToSerialPort((uint8_t*)&operate_bytes_, sizeof(operate_bytes_), file_descriptor_);
    if (status) {
        rm::message("Control error: " + std::to_string(status), rm::MSG_ERROR);
        if (access(port_name_.c_str(), F_OK) < 0) {
            init_serial();
            status = 0;
        } else {
            status = (int)rm::restartSerialPort(file_descriptor_, port_name_);
        }
    }
}

void Control::autoaim() {
    std::thread send_thread(&Control::send_thread, this);
    send_thread.detach();

    if (Data::serial_flag) {
        std::thread receive_thread(&Control::receive_thread, this);
        receive_thread.detach();
    }
}
