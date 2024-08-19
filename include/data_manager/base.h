#ifndef RM2024_DATA_MANAGER_BASE_H_
#define RM2024_DATA_MANAGER_BASE_H_

#include <opencv2/opencv.hpp>
#include <openrm.h>
#include <Eigen/Dense>
#include <cstdint>

namespace Data {

extern rm::ArmorColor enemy_color;
extern rm::ArmorColor self_color;
extern rm::ArmorID self_id;
extern rm::ArmorID target_id;
extern double target_omega;
extern double target_dist;

extern std::vector<rm::Camera*> camera;
extern int camera_index;
extern int camera_base, camera_far;

extern uint8_t state;
extern float yaw;
extern float pitch;
extern float roll;
extern float yaw_omega;

extern rm::AttackInterface* attack;

extern bool auto_fire;
extern bool auto_enemy;
extern bool auto_rune;
extern bool auto_capture;
extern bool plus_pnp;

extern bool serial_flag;
extern bool timeout_flag;
extern bool manu_capture;
extern bool manu_fire;

extern bool manu_rune;
extern bool big_rune;

extern bool armor_mode;
extern bool rune_mode;
extern bool defence_mode;
extern bool record_mode;

extern bool image_flag;
extern bool ui_flag;
extern bool imshow_flag;
extern bool imwrite_flag;
extern bool binary_flag;
extern bool histogram_flag;


extern bool reprojection_flag;
extern bool pipeline_delay_flag;
extern bool point_skip_flag;

extern bool state_delay_flag;
extern double state_delay_time;
extern int state_queue_size;
extern int send_wait_time;

};

void init_debug();
bool init_camera();
bool deinit_camera();
void init_serial();
void init_attack();

#endif