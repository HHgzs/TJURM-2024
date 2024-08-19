#include "data_manager/base.h"

// 颜色
rm::ArmorColor Data::self_color;
rm::ArmorColor Data::enemy_color;

// 本机编号
rm::ArmorID Data::self_id;

// 当前核心打击目标
rm::ArmorID Data::target_id;
double Data::target_omega;
double Data::target_dist;

// 相机的参数记录
std::vector<rm::Camera*> Data::camera;
int Data::camera_index;
int Data::camera_base, Data::camera_far;

// 击打目标
rm::AttackInterface* Data::attack;

// 串口数据
uint8_t Data::state;
float Data::yaw   = 0.0;
float Data::pitch = 0.0;
float Data::roll  = 0.0;
float Data::yaw_omega = 0.0;

// debug模式
bool Data::auto_fire;

bool Data::auto_enemy;
bool Data::auto_rune;
bool Data::auto_capture;
bool Data::plus_pnp;

bool Data::serial_flag;
bool Data::timeout_flag;
bool Data::manu_capture;
bool Data::manu_fire;

bool Data::manu_rune;
bool Data::big_rune;

bool Data::armor_mode;
bool Data::rune_mode;
bool Data::defence_mode;
bool Data::record_mode;

bool Data::image_flag;
bool Data::ui_flag;
bool Data::imshow_flag = false;
bool Data::imwrite_flag;
bool Data::binary_flag;
bool Data::histogram_flag;

bool Data::reprojection_flag;
bool Data::pipeline_delay_flag;
bool Data::point_skip_flag;

bool Data::state_delay_flag;
double Data::state_delay_time;
int Data::state_queue_size;
int Data::send_wait_time;