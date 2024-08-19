#include <string>
#include <vector>
#include <thread>
#include <iostream>
#include <fstream>
#include <chrono>
#include "data_manager/base.h"
#include "data_manager/param.h"
#include "threads/pipeline.h"
#include "threads/control.h"
#include "garage/garage.h"

void init_debug() {
    auto param = Param::get_instance();
    Data::auto_fire = (*param)["Debug"]["System"]["AutoFire"];
    Data::auto_enemy = (*param)["Debug"]["System"]["AutoEnemy"];
    Data::auto_rune = (*param)["Debug"]["System"]["AutoRune"];
    Data::auto_capture = (*param)["Debug"]["System"]["AutoCapture"];

    Data::plus_pnp = (*param)["Debug"]["PlusPnP"]["Enable"];

    Data::serial_flag = (*param)["Debug"]["Control"]["Serial"];
    Data::timeout_flag = (*param)["Debug"]["Control"]["Timeout"];
    Data::manu_capture = (*param)["Debug"]["Control"]["ManuCapture"];
    Data::manu_fire = (*param)["Debug"]["Control"]["ManuFire"];

    Data::manu_rune = (*param)["Debug"]["Control"]["ManuRune"];
    Data::big_rune = (*param)["Debug"]["Control"]["BigRune"];

    Data::ui_flag = (*param)["Debug"]["ImageThread"]["UI"];
    Data::imwrite_flag = (*param)["Debug"]["ImageThread"]["Imwrite"];
    Data::image_flag = (bool)(Data::imwrite_flag || Data::imshow_flag);
    Data::binary_flag = (*param)["Debug"]["ImageThread"]["Binary"];
    Data::histogram_flag = (*param)["Debug"]["ImageThread"]["Histogram"];

    Data::reprojection_flag = (*param)["Debug"]["Display"]["Reprojection"];
    Data::pipeline_delay_flag = (*param)["Debug"]["Display"]["PipelineDelay"];
    Data::point_skip_flag = (*param)["Debug"]["Display"]["PointSkip"];

    Data::state_delay_flag = (*param)["Debug"]["StateDelay"]["Enable"];
    Data::state_delay_time = (*param)["Debug"]["StateDelay"]["TimeS"];
    Data::state_queue_size = (*param)["Debug"]["StateDelay"]["QueueSize"];
    Data::send_wait_time = (*param)["Debug"]["StateDelay"]["SendWait"];
}

bool init_camera() {
    auto param = Param::get_instance();
    auto control = Control::get_instance();

    // 获取相机参数矩阵json
    nlohmann::json camlens;
    std::string camlen_path = (*param)["Camera"]["CamLensDir"];
    try {
        std::ifstream camlens_json(camlen_path);
        camlens_json >> camlens;
        camlens_json.close();
    } catch (std::exception& e) {
        std::string err_str = "Failed to load CamLens json: " + std::string(e.what());
        rm::message(err_str, rm::MSG_ERROR);
        return false;
    }

    // 获取相机数量
    int camera_num;
    bool flag_camera = rm::getDaHengCameraNum(camera_num);
    Data::camera.clear();
    Data::camera.resize(camera_num + 1, nullptr);
    if(!flag_camera) {
        rm::message("Failed to get camera number", rm::MSG_ERROR);
        return false;
    }
    rm::message("get camera number "+ std::to_string(camera_num), rm::MSG_NOTE);
    // 初始化单相机
    if(camera_num == 1) {
        Data::camera_index = 1;
        Data::camera_base = 1;
        Data::camera_far = 1;

        double exp = (*param)["Camera"]["Base"]["ExposureTime"];
        double gain = (*param)["Camera"]["Base"]["Gain"];
        double rate = (*param)["Camera"]["Base"]["FrameRate"];
        std::string camera_type = (*param)["Camera"]["Base"]["CameraType"];
        std::string lens_type = (*param)["Camera"]["Base"]["LensType"];
        std::vector<double> camera_offset = (*param)["Car"]["CameraOffset"]["Base"];

        Data::camera[1] = new rm::Camera();
        flag_camera = rm::openDaHeng(
            Data::camera[1], 1, &Data::yaw, &Data::pitch, &Data::roll,
            false, exp, gain, rate);

        if(!flag_camera) {
            rm::message("Failed to open camera", rm::MSG_ERROR);
            return false;
        }

        Param::from_json(camlens[camera_type][lens_type]["Intrinsic"], Data::camera[1]->intrinsic_matrix);
        Param::from_json(camlens[camera_type][lens_type]["Distortion"], Data::camera[1]->distortion_coeffs);
        rm::tf_rotate_pnp2head(Data::camera[1]->Rotate_pnp2head, camera_offset[3], camera_offset[4], 0.0);
        rm::tf_trans_pnp2head(Data::camera[1]->Trans_pnp2head, camera_offset[0], camera_offset[1], camera_offset[2], camera_offset[3], camera_offset[4], 0.0);
        rm::mallocYoloCameraBuffer(&Data::camera[1]->rgb_host_buffer, &Data::camera[1]->rgb_device_buffer, Data::camera[1]->width, Data::camera[1]->height);



    // 初始化双相机
    } else if (camera_num == 2) {
        double exp_base = (*param)["Camera"]["Base"]["ExposureTime"];
        double gain_base = (*param)["Camera"]["Base"]["Gain"];
        double rate_base = (*param)["Camera"]["Base"]["FrameRate"];
        int width_base = (*param)["Camera"]["Base"]["Width"];

        double exp_far = (*param)["Camera"]["Far"]["ExposureTime"];
        double gain_far = (*param)["Camera"]["Far"]["Gain"];
        double rate_far = (*param)["Camera"]["Far"]["FrameRate"];
        int width_far = (*param)["Camera"]["Far"]["Width"];

        std::string camera_type_base = (*param)["Camera"]["Base"]["CameraType"];
        std::string lens_type_base = (*param)["Camera"]["Base"]["LensType"];

        std::string camera_type_far = (*param)["Camera"]["Far"]["CameraType"];
        std::string lens_type_far = (*param)["Camera"]["Far"]["LensType"];

        std::vector<double> camera_offset_base = (*param)["Car"]["CameraOffset"]["Base"];
        std::vector<double> camera_offset_far = (*param)["Car"]["CameraOffset"]["Far"];


        for(int i = 1; i <= camera_num; i++) {
            rm::message("begin open camera "+ std::to_string(i), rm::MSG_NOTE);
            Data::camera[i] = new rm::Camera();
            flag_camera = rm::openDaHeng(Data::camera[i], i, &Data::yaw, &Data::pitch, &Data::roll);

            if(!flag_camera) {
                rm::message("Failed to open camera: " + std::to_string(i), rm::MSG_ERROR);
                return false;
            }
            rm::message("camera width "+ std::to_string(Data::camera[i]->width), rm::MSG_NOTE);
            if (Data::camera[i]->width == width_base) {
                Data::camera_base = i;
                Data::camera_index = i;
                flag_camera = setDaHengArgs(Data::camera[i], exp_base, gain_base, rate_base);
                if(!flag_camera) {
                    rm::message("Failed to set camera args: " + std::to_string(i), rm::MSG_ERROR);
                    return false;
                }

                Param::from_json(camlens[camera_type_base][lens_type_base]["Intrinsic"], Data::camera[i]->intrinsic_matrix);
                Param::from_json(camlens[camera_type_base][lens_type_base]["Distortion"], Data::camera[i]->distortion_coeffs);
                rm::tf_rotate_pnp2head(Data::camera[i]->Rotate_pnp2head, camera_offset_base[3], camera_offset_base[4], 0.0);
                rm::tf_trans_pnp2head(Data::camera[i]->Trans_pnp2head, camera_offset_base[0], camera_offset_base[1], camera_offset_base[2], camera_offset_base[3], camera_offset_base[4], 0.0);
                rm::mallocYoloCameraBuffer(&Data::camera[i]->rgb_host_buffer, &Data::camera[i]->rgb_device_buffer, Data::camera[i]->width, Data::camera[i]->height);

            } else if (Data::camera[i]->width == width_far) {
                Data::camera_far = i;
                flag_camera = setDaHengArgs(Data::camera[i], exp_far, gain_far, rate_far);
                if(!flag_camera) {
                    rm::message("Failed to set camera args: " + std::to_string(i), rm::MSG_ERROR);
                    return false;
                }

                Param::from_json(camlens[camera_type_far][lens_type_far]["Intrinsic"], Data::camera[i]->intrinsic_matrix);
                Param::from_json(camlens[camera_type_far][lens_type_far]["Distortion"], Data::camera[i]->distortion_coeffs);
                rm::tf_rotate_pnp2head(Data::camera[i]->Rotate_pnp2head, camera_offset_far[3], camera_offset_far[4], 0.0);
                rm::tf_trans_pnp2head(Data::camera[i]->Trans_pnp2head, camera_offset_far[0], camera_offset_far[1], camera_offset_far[2], camera_offset_far[3], camera_offset_far[4], 0.0);
                rm::mallocYoloCameraBuffer(&Data::camera[i]->rgb_host_buffer, &Data::camera[i]->rgb_device_buffer, Data::camera[i]->width, Data::camera[i]->height);

            } else {
                rm::message("Invalid camera width: " + std::to_string(Data::camera[i]->width), rm::MSG_ERROR);
                return false;
            }
        }

    } else {
        rm::message("Invalid camera number: " + std::to_string(camera_num), rm::MSG_ERROR);
        return false;
    }
    return true;
}

bool deinit_camera() {
    for(int i = 1; i < Data::camera.size(); i++) {
        if(Data::camera[i] == nullptr) continue;
        
        if (Data::camera[i]->rgb_host_buffer != nullptr || Data::camera[i]->rgb_device_buffer != nullptr) {
            rm::freeYoloCameraBuffer(Data::camera[i]->rgb_host_buffer, Data::camera[i]->rgb_device_buffer);
            Data::camera[i]->rgb_host_buffer = nullptr;
            Data::camera[i]->rgb_device_buffer = nullptr;
        }

        delete Data::camera[i];
        Data::camera[i] = nullptr;
        rm::closeDaHeng();
    }
    rm::message("Camera deinit success", rm::MSG_WARNING);
    return true;
}


void init_serial() {
    int status;
    std::vector<std::string> port_list;
    auto control = Control::get_instance();

    while(true) {

        #if defined(TJURM_HERO)
        status = (int)rm::getSerialPortList(port_list, rm::SERIAL_TYPE_TTY_ACM);
        #endif

        #if defined(TJURM_BALANCE) || defined(TJURM_INFANTRY) || defined(TJURM_DRONSE) || defined(TJURM_SENTRY)
        status = (int)rm::getSerialPortList(port_list, rm::SERIAL_TYPE_TTY_USB);
        #endif

        if (status != 0 || port_list.empty()) {
            rm::message("Control port list failed", rm::MSG_ERROR);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            port_list.clear();
            continue;
        }

        control->port_name_ = port_list[0];
        status = (int)rm::openSerialPort(control->file_descriptor_, control->port_name_);
        if (status != 0) {
            rm::message("Control port open failed", rm::MSG_ERROR);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            port_list.clear();
            continue;
        }
        if(status == 0) {
            break;
        }
    }
}


void init_attack() {
    #ifdef TJURM_SENTRY
    Data::attack = new rm::Filtrate();
    #endif

    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_HERO) || defined(TJURM_DRONSE)
    Data::attack = new rm::DeadLocker();
    #endif 
}