#ifndef RM2024_THREADS_SERIAL_STRUCTURE_H_
#define RM2024_THREADS_SERIAL_STRUCTURE_H_
#include <cstdint>

struct FrameHeader {
    uint8_t     sof;
    uint8_t     crc8;
} __attribute__((packed));

struct FrameTailer {
    uint16_t    crc16;
} __attribute__((packed));


#ifdef TJURM_INFANTRY
struct InputData {
    float       curr_yaw;
    float       curr_pitch;
    float       curr_omega;
    uint8_t     state;
    uint8_t     autoaim;
    uint8_t     enemy_color;
} __attribute__((packed));

struct OutputData {
    float       shoot_yaw;
    float       shoot_pitch;
    uint8_t     fire;
    uint8_t     target_id;
} __attribute__((packed));
#endif

#ifdef TJURM_BALANCE
struct InputData {
    float       curr_yaw;
    float       curr_pitch;
    uint8_t     state;
    uint8_t     autoaim;
    uint8_t     enemy_color;
} __attribute__((packed));

struct OutputData {
    uint8_t     fire;
    float       shoot_yaw;
    float       shoot_pitch;
} __attribute__((packed));
#endif

#ifdef TJURM_HERO
struct InputData {
    float       curr_yaw;
    float       curr_pitch;
    uint8_t     state;
    uint8_t     autoaim;
    uint8_t     enemy_color;
    float       curr_speed;
} __attribute__((packed));

struct OutputData {
    uint8_t     fire;
    float       shoot_yaw;
    float       shoot_pitch;
    float       avg_speed;
    uint8_t     food;
} __attribute__((packed));
#endif


#ifdef TJURM_DRONSE
struct InputData {
    float       curr_yaw;
    float       curr_pitch;
    float       curr_roll;
    uint8_t     state;
    uint8_t     autoaim;
    uint8_t     enemy_color;
} __attribute__((packed));

struct OutputData {
    uint8_t     fire;
    float       shoot_yaw;
    float       shoot_pitch;
} __attribute__((packed));
#endif



#ifdef TJURM_SENTRY
struct InputData {
    uint64_t    config;
    float       target_pose[3];
    float       curr_yaw;
    float       curr_pitch;
    uint8_t     enemy_color;
    uint8_t     shoot_config;
} __attribute__((packed));

struct OutputData {
    float       shoot_yaw;           // 经过自瞄预测的yaw角度
    float       shoot_pitch;         // 经过自瞄预测的pitch角度
    uint8_t     fire;                // 是否发弹
} __attribute__((packed));
#endif


struct StateBytes {                     // 电控传给自瞄系统的云台数据
    FrameHeader frame_header;
    InputData   input_data;
    FrameTailer frame_tailer;   
} __attribute__((packed));

struct OperateBytes {                   // 自瞄返回给电控的控制数据
    FrameHeader frame_header;
    OutputData  output_data;
    FrameTailer frame_tailer;
} __attribute__((packed));


#endif