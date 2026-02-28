#pragma once

#include <FlexCAN_T4.h>
#include <map>
#include <math.h>
#include <string.h>
#include <Arduino.h>

// =====================
// RoboStride definitions
// =====================

#define Set_mode      'j' // 设置控制模式
#define Set_parameter 'p' // 设置参数

// 各种控制模式
#define move_control_mode        0 // 运控模式
#define PosPP_control_mode       1 // 位置模式（PP）
#define Speed_control_mode       2 // 速度模式
#define Elect_control_mode       3 // 电流模式
#define Set_Zero_mode            4 // 零点模式
#define PosCSP_control_mode      5 // 位置模式（CSP）

// ROS2 / CAN timeout
#define ROBOSTRIDE_ROS2_TIMEOUT  1000 // ms
#define ROBOSTRIDE_CAN_TIMEOUT   100  // ms

#define SC_MAX 23.0f
#define SC_MIN 0.0f
#define SV_MAX 20.0f
#define SV_MIN -20.0f
#define SCIQ_MIN -23.0f

// 通信地址
#define Communication_Type_Get_ID                0x00  // 获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl         0x01  // 运控模式控制指令
#define Communication_Type_MotorRequest          0x02  // 反馈电机运行状态
#define Communication_Type_MotorEnable           0x03  // 电机使能运行
#define Communication_Type_MotorStop             0x04  // 电机停止运行
#define Communication_Type_SetPosZero            0x06  // 设置电机机械零位
#define Communication_Type_Can_ID                0x07  // 更改当前电机CAN_ID
#define Communication_Type_Control_Mode          0x12  // 设置电机模式
#define Communication_Type_GetSingleParameter    0x11  // 读取单个参数
#define Communication_Type_SetSingleParameter    0x12  // 设定单个参数
#define Communication_Type_ErrorFeedback         0x15  // 故障反馈帧

static const uint16_t Index_List[] = {
    0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018,
    0x7019, 0x701A, 0x701B, 0x701C, 0x701D, 0x701E, 0x701F, 0x7020, 0x7021
};

struct ReceiveResult{
    uint8_t communication_type;
    uint16_t extra_data;
    uint8_t host_id;
    uint8_t data[8];
    uint8_t len;
};

struct RobostrideFeedback {
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temp_mos;
    float temp_rotor;
    uint8_t status;
};

enum class ActuatorType {
    ROBSTRIDE_00 = 0,
    ROBSTRIDE_01 = 1,
    ROBSTRIDE_02 = 2,
    ROBSTRIDE_03 = 3,
    ROBSTRIDE_04 = 4,
    ROBSTRIDE_05 = 5,
    ROBSTRIDE_06 = 6
};

struct ActuatorOperation {
    double position;   // rad
    double velocity;   // rad/s
    double torque;     // Nm
    double kp;
    double kd;
};

struct data_read_write_one {
    uint16_t index;
    float data;
};

typedef struct{
    int   set_motor_mode;
    float set_current;
    float set_speed;
    float set_Torque;
    float set_angle;
    float set_limit_cur;
    float set_Kp;
    float set_Ki;
    float set_Kd;
    float set_iq;
    float set_id;
    float set_acc;
} Motor_Set;

struct data_read_write{
    data_read_write_one run_mode       = {Index_List[0], 0};   // 0..5
    data_read_write_one iq_ref         = {Index_List[1], 0};   // -23..23A
    data_read_write_one spd_ref        = {Index_List[2], 0};   // -30..30rad/s
    data_read_write_one imit_torque    = {Index_List[3], 0};   // 0..12Nm
    data_read_write_one cur_kp         = {Index_List[4], 0};   // default 0.125
    data_read_write_one cur_ki         = {Index_List[5], 0};   // default 0.0158
    data_read_write_one cur_filt_gain  = {Index_List[6], 0.1}; // 0..1
    data_read_write_one loc_ref        = {Index_List[7], 0};   // rad
    data_read_write_one limit_spd      = {Index_List[8], 0};   // 0..30rad/s
    data_read_write_one limit_cur      = {Index_List[9], 0};   // 0..23A

    // read-only
    data_read_write_one mechPos        = {Index_List[10], 0};
    data_read_write_one iqf            = {Index_List[11], 0};
    data_read_write_one mechVel        = {Index_List[12], 0};
    data_read_write_one VBUS           = {Index_List[13], 0};
    data_read_write_one rotation       = {Index_List[14], 0};

    data_read_write_one loc_kp        = {Index_List[15], 0};
    data_read_write_one spd_kp        = {Index_List[16], 0};
    data_read_write_one spd_ki        = {Index_List[17], 0};
    data_read_write_one spd_filt_gain = {Index_List[18], 0};
};

static const std::map<ActuatorType, ActuatorOperation> ACTUATOR_OPERATION_MAPPING = {
    { ActuatorType::ROBSTRIDE_00, { 4 * M_PI, 50, 17,   500.0,  5.0   } },
    { ActuatorType::ROBSTRIDE_01, { 4 * M_PI, 44, 17,   500.0,  5.0   } },
    { ActuatorType::ROBSTRIDE_02, { 4 * M_PI, 44, 17,   500.0,  5.0   } },
    { ActuatorType::ROBSTRIDE_03, { 4 * M_PI, 50, 60,  5000.0, 100.0  } },
    { ActuatorType::ROBSTRIDE_04, { 4 * M_PI, 15, 120, 5000.0, 100.0  } },
    { ActuatorType::ROBSTRIDE_05, { 4 * M_PI, 33, 17,   500.0,  5.0   } },
    { ActuatorType::ROBSTRIDE_06, { 4 * M_PI, 20, 60,  5000.0, 100.0  } },
};

// =====================
// Control ref (Damiao-like)
// =====================
// len=0/1: disable
// len=2: velocity  (data[1]=vel)
// len=3: motion    (data[1]=vel, data[2]=pos)
// len=4: gain set  (data[0]=cur_kp, data[1]=cur_ki, data[2]=motion_kp, data[3]=motion_kd)
struct RoboStrideRef {
    int mode = 0; // 0:disable, 2:velocity, 3:motion
    float position_rad = 0.f;
    float velocity_rad_s = 0.f;

    float torque_nm = 0.f; // motion torque
    float vel_kp = 1.f;        // motion kp
    float vel_ki = 0.1f;        // motion ki
    float pos_kp = 20.f;        // motion kp
    // float pos_kd = 0.f;        // motion kd
};

FCTP_CLASS class RoboStride {
    uint8_t master_id_, motor_id_;
    int actuator_type_;
    FlexCAN_T4<_bus, _rxSize, _txSize> *can_;

    data_read_write_one params_;

    uint8_t error_code_ = 0;
    uint8_t pattern_ = 0;

    Motor_Set Motor_Set_All_ = {0};

    unsigned long last_recv_ros2_ts_ms_ = 0;

    // Track multi-turn position state similar to Damiao implementation
    int32_t position_turns_ = 0;
    float accumulated_position_rad_ = 0.f;
    float offset_position_rad_ = 0.f;
    float prev_raw_position_rad_ = 0.f;
    bool first_feedback_received_ = true;

public:
    RobostrideFeedback feedback = {0};
    RoboStrideRef ref;
    bool connect = false;
    data_read_write drw;

    RoboStride(FlexCAN_T4<_bus, _rxSize, _txSize> *can, uint8_t master_id, uint8_t motor_id, int actuator_type) :
        master_id_(master_id),
        motor_id_(motor_id),
        actuator_type_(actuator_type),
        can_(can)
    {}
    ~RoboStride(){}

    // ===========
    // Requested APIs
    // ===========

    void init(float current_max, float torque_max){
        // Try to read current run_mode (non-fatal)
        // 運転モード
        Get_RobStrite_Motor_parameter(0x7005);
        Get_RobStrite_Motor_parameter(0x7010); // cur_kp
        Get_RobStrite_Motor_parameter(0x7011); // cur_ki
        Get_RobStrite_Motor_parameter(0x7014); // cur_filt_gain

        // ---- velocity loop ----
        Get_RobStrite_Motor_parameter(0x701F); // spd_kp
        Get_RobStrite_Motor_parameter(0x7020); // spd_ki
        Get_RobStrite_Motor_parameter(0x7021); // spd_filt_gain

        // ---- position loop ----
        Get_RobStrite_Motor_parameter(0x701E); // loc_kp

        delayMicroseconds(1000);

        // Ensure motor is disabled at startup for safety
        // Disenable_Motor(0);
        // delayMicroseconds(1000);

        // Disenable_Motor(0);
        // delayMicroseconds(1000);

        Set_RobStrite_Motor_parameter(drw.run_mode.index, Speed_control_mode, Set_mode);
        delayMicroseconds(1000);

        Get_RobStrite_Motor_parameter(drw.run_mode.index);
        delayMicroseconds(1000);

        enable_motor();

        // Example: set some limits (kept from your original)
        Set_RobStrite_Motor_parameter(0X7018, 27.0f, Set_parameter);
        delayMicroseconds(1000);

        Set_RobStrite_Motor_parameter(0X7026, Motor_Set_All_.set_acc, Set_parameter);
        delayMicroseconds(1000);
        Serial.printf("Switched to Speed Control Mode, Now: %d\n", (int)drw.run_mode.data);

        Set_RobStrite_Motor_parameter(0X700B, torque_max, Set_parameter);//torque
        delayMicroseconds(1000);

        Set_RobStrite_Motor_parameter(0X7018, current_max, Set_parameter);//curr
        delayMicroseconds(1000);

        Set_RobStrite_Motor_parameter(0X7022, 1000, Set_parameter);//vel
        delayMicroseconds(1000);

        // Optional: enable once to get feedback frames flowing (depends on device behavior)
        enable_motor();
        delayMicroseconds(1000);

        // Seed timestamp so we don't instantly timeout
        last_recv_ros2_ts_ms_ = millis();

        // One feedback update attempt (non-fatal)
        receive_status_frame();
    }

    void setPacketFrame(const float *data, const int len){
        switch(len){
            case 0:
            case 1:
                ref.mode = 0; // disable
                last_recv_ros2_ts_ms_ = millis();
                break;

            case 2:
                ref.mode = 2; // velocity
                last_recv_ros2_ts_ms_ = millis();
                ref.velocity_rad_s = data[1];
                break;

            case 3:
                ref.mode = 3; // motion
                last_recv_ros2_ts_ms_ = millis();
                ref.velocity_rad_s = data[1];
                ref.position_rad   = data[2];
                break;

            case 4:
                // param/gain update packet
                last_recv_ros2_ts_ms_ = millis();
                ref.vel_kp = data[0];
                ref.vel_ki = data[1];
                ref.pos_kp     = data[2];
                // ref.pos_kd     = data[3];
                break;

            default:
                return;
        }
    }

    void writeCanFrame(){
        // timeout => disable
        int mode = ref.mode;
        if(millis() - last_recv_ros2_ts_ms_ > ROBOSTRIDE_ROS2_TIMEOUT){
            mode = 0;
        }
        if(ref.vel_kp != drw.spd_kp.data || ref.vel_ki != drw.spd_ki.data || ref.pos_kp != drw.loc_kp.data){
            // Disenable_Motor(0);
            // delayMicroseconds(500);

            Set_RobStrite_Motor_parameter(drw.spd_kp.index, ref.vel_kp, Set_parameter);
            delayMicroseconds(500);

            Get_RobStrite_Motor_parameter(drw.spd_kp.index);
            delayMicroseconds(500);

            Set_RobStrite_Motor_parameter(drw.spd_ki.index, ref.vel_ki, Set_parameter);
            delayMicroseconds(500);

            Get_RobStrite_Motor_parameter(drw.spd_ki.index);
            delayMicroseconds(500);

            Set_RobStrite_Motor_parameter(drw.loc_kp.index, ref.pos_kp, Set_parameter);
            delayMicroseconds(500);

            Get_RobStrite_Motor_parameter(drw.loc_kp.index);
            delayMicroseconds(500);

            // enable_motor();
            Serial.println("-----[cmd] gain updated-----");
        }

        // ---- control ----
        switch(mode){
            case 0: {
                send_velocity_mode_command(0);
                // Disenable_Motor(0);
                receive_status_frame(); // best effort
                break;
            }

            case 2: {
                // velocity mode (internal changes: disable->set mode->enable)
                send_velocity_mode_command(ref.velocity_rad_s);
                break;
            }

            case 3: {
                // motion mode (internal changes: disable->set mode->enable)
                // send_motion_command()
                break;
            }

            default:
                break;
        }

        // connect state (Damiao-like: "ROS2 is alive")
        connect = (millis() - last_recv_ros2_ts_ms_) < ROBOSTRIDE_CAN_TIMEOUT;
    }

private:
    // ==============
    // CAN receive/parsing
    // ==============
    bool receive(ReceiveResult &result, double timeout_ms){
    const unsigned long start_time = millis();
    CAN_message_t r_msg;

    // timeout_ms == 0 のときは「1回だけ読みに行く」扱いにしたいので、
    // ループを最低1回は回す
    while ((millis() - start_time) < (unsigned long)timeout_ms){
        if (can_->read(r_msg)){
            // ---- Filter 1: only extended frames (RoboStride is extended) ----
            if (!r_msg.flags.extended) {
                continue; // e.g., Damiao standard frames
            }

            // ---- Filter 2: only frames for this motor_id_ (lowest 8 bits) ----
            const uint32_t can_id = r_msg.id;
            const uint8_t  dst_motor_id = (uint8_t)(can_id & 0xFF);
            if (dst_motor_id != motor_id_) {
                continue;
            }

            // ---- Parse fields (match your current layout) ----
            result.communication_type = (uint8_t)((can_id >> 24) & 0x1F); // 5bit type
            result.extra_data         = (uint16_t)((can_id >> 8) & 0xFFFF);
            result.host_id            = (uint8_t)(can_id & 0xFF);         // NOTE: this is motor_id in your naming

            // Decode error/pattern from ID (as you already did)
            error_code_ = (uint8_t)((can_id >> 16) & 0x3F);
            pattern_    = (uint8_t)((can_id >> 22) & 0x03);

            memcpy(result.data, r_msg.buf, r_msg.len);
            result.len = r_msg.len;

            return true;
        }
    } 

    return false;
}

    bool receive_status_frame(){
        ReceiveResult result;

        // 5msは短いことがあるので、必要なら 10〜20ms に増やしてもOK
        const unsigned long timeout_ms = 10;
        const unsigned long start_time = millis();

        while ((millis() - start_time) < timeout_ms) {
            if (!receive(result, 10)) {
                continue;
            }

            // 必須：8バイト前提のフレームしかパースしない
            if (result.len < 8) {
                continue;
            }

            // ---- 0x02: motor feedback ----
            if (result.communication_type == Communication_Type_MotorRequest) {
                const uint16_t pos_u16  = (uint16_t)((result.data[0] << 8) | result.data[1]);
                const uint16_t vel_u16  = (uint16_t)((result.data[2] << 8) | result.data[3]);
                const uint16_t tor_u16  = (uint16_t)((result.data[4] << 8) | result.data[5]);
                const uint16_t temp_u16 = (uint16_t)((result.data[6] << 8) | result.data[7]);

                const auto &op = ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type_));

                // ✅ 16bit (0..65535) マップで復元（PDF仕様に合わせる）
                const float current_raw_pos = uint_to_float(pos_u16, -(float)op.position, (float)op.position, 16);
                const float half_range = (float)op.position;
                const float full_range = half_range * 2.0f;

                if (first_feedback_received_) {
                    position_turns_ = 0;
                    offset_position_rad_ = current_raw_pos;
                    first_feedback_received_ = false;
                } else if (half_range > 0.0f) {
                    const float diff = current_raw_pos - prev_raw_position_rad_;
                    if (diff < -half_range) {
                        position_turns_++;
                    } else if (diff > half_range) {
                        position_turns_--;
                    }
                }

                accumulated_position_rad_ = (static_cast<float>(position_turns_) * full_range) + current_raw_pos;
                prev_raw_position_rad_ = current_raw_pos;
                feedback.position_rad = accumulated_position_rad_ - offset_position_rad_;

                feedback.velocity_rad_s = uint_to_float(vel_u16, -(float)op.velocity, (float)op.velocity, 16);
                feedback.torque_nm      = uint_to_float(tor_u16, -(float)op.torque,   (float)op.torque,   16);

                // Temp*10 → 0.1倍
                feedback.temp_mos = (float)temp_u16 * 0.1f;

                return true;
            }

            // ---- 0x11: parameter read response ----
            if (result.communication_type == Communication_Type_GetSingleParameter) {
                // index is little-endian in data[0..1]
                const uint16_t idx = (uint16_t)((result.data[1] << 8) | result.data[0]);

                params_.index = idx;

                // run_mode (0x7005) is 1 byte at data[4]
                if (idx == Index_List[0]) {
                    drw.run_mode.data = (float)uint8_t(result.data[4]);
                    params_.data = drw.run_mode.data;
                    return true;
                }

                // float parameters: data[4..7] is float (little-endian)
                const float v = Byte_to_float(result.data);
                params_.data = v;

                // update cache if idx matches
                const int N = (int)(sizeof(Index_List) / sizeof(Index_List[0]));
                for (int i = 0; i < N; i++) {
                    if (idx == Index_List[i]) {
                        switch(i){
                            case 1:  drw.iq_ref.data        = v; break;
                            case 2:  drw.spd_ref.data       = v; break;
                            case 3:  drw.imit_torque.data   = v; break;
                            case 4:  drw.cur_kp.data = v; break;
                            case 5:  drw.cur_ki.data = v; break;
                            case 6:  drw.cur_filt_gain.data = v; break;
                            case 7:  drw.loc_ref.data       = v; break;
                            case 8:  drw.limit_spd.data     = v; break;
                            case 9:  drw.limit_cur.data     = v; break;
                            case 10: drw.mechPos.data       = v; break;
                            case 11: drw.iqf.data           = v; break;
                            case 12: drw.mechVel.data       = v; break;
                            case 13: drw.VBUS.data          = v; break;
                            case 14: drw.rotation.data = v; break;
                            case 15: drw.loc_kp.data = v; break;
                            case 16: drw.spd_kp.data = v; break;
                            case 17: drw.spd_ki.data = v; break;
                            case 18: drw.spd_filt_gain.data = v; break;

                            default: break;
                        }
                        break;
                    }
                }
                return true;
            }

            // ---- 0x15: error feedback (optional) ----
            if (result.communication_type == Communication_Type_ErrorFeedback) {
                // error_code_ / pattern_ は receive() 側で更新済み
                // 必要なら result.data の詳細もここで解釈
                return true;
            }

            // それ以外の通信タイプはこの関数の目的外なので読み捨て
        }

        return false;
    }

    // ==============
    // Parameter read/write
    // ==============
    bool Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode){
        CAN_message_t w_msg;
        w_msg.id = (Communication_Type_SetSingleParameter << 24) | (master_id_ << 8) | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 0x08;

        w_msg.buf[0] = (uint8_t)(Index & 0xFF);
        w_msg.buf[1] = (uint8_t)((Index >> 8) & 0xFF);
        w_msg.buf[2] = 0x00;
        w_msg.buf[3] = 0x00;

        if (Value_mode == Set_parameter){
            memcpy(&w_msg.buf[4], &Value, 4);
        }
        else if (Value_mode == Set_mode){
            w_msg.buf[4] = (uint8_t)Value;
            w_msg.buf[5] = 0x00;
            w_msg.buf[6] = 0x00;
            w_msg.buf[7] = 0x00;
        }

        if(can_->write(w_msg) != 1) {
            return false;
        }
        return receive_status_frame();
    }

    bool Get_RobStrite_Motor_parameter(uint16_t Index){
        CAN_message_t w_msg;
        w_msg.id = (Communication_Type_GetSingleParameter << 24) | (master_id_ << 8) | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 8;

        w_msg.buf[0] = (uint8_t)(Index & 0xFF);
        w_msg.buf[1] = (uint8_t)((Index >> 8) & 0xFF);
        w_msg.buf[2] = 0x00;
        w_msg.buf[3] = 0x00;
        w_msg.buf[4] = 0x00;
        w_msg.buf[5] = 0x00;
        w_msg.buf[6] = 0x00;
        w_msg.buf[7] = 0x00;

        if(can_->write(w_msg) != 1) {
            return false;
        }
        return receive_status_frame();
    }

    // ==============
    // Enable/Disable
    // ==============
    bool enable_motor(){
        CAN_message_t w_msg;
        w_msg.id = (Communication_Type_MotorEnable << 24) | (master_id_ << 8) | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 8;
        memset(w_msg.buf, 0, 8);

        if (can_->write(w_msg) != 1) {
            return false;
        }
        return receive_status_frame();
    }

    bool Disenable_Motor(uint8_t clear_error){
        CAN_message_t w_msg{};
        w_msg.id = (Communication_Type_MotorStop << 24) | (master_id_ << 8) | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 8;
        memset(w_msg.buf, 0, 8);

        w_msg.buf[0] = clear_error;

        if(can_->write(w_msg) != 1)
            return false;
        return receive_status_frame();
    }

    // ==============
    // Control commands
    // ==============
    bool send_motion_command(float torque,
                             float position_rad,
                             float velocity_rad_s,
                             float kp,
                             float kd)
    {
        // Ensure run_mode = move_control_mode (0) if required
        if(drw.run_mode.data != 0){
            Disenable_Motor(0);
            delayMicroseconds(1000);
            Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);
            delayMicroseconds(1000);
            Get_RobStrite_Motor_parameter(0x7005);
            delayMicroseconds(1000);
        }

        const auto &op = ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type_));

        CAN_message_t w_msg;
        w_msg.id = (Communication_Type_MotionControl << 24)
                 | (float_to_uint(torque, -(float)op.torque, (float)op.torque, 16) << 8)
                 | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 8;

        uint16_t pos  = float_to_uint(position_rad,  -(float)op.position, (float)op.position, 16);
        uint16_t vel  = float_to_uint(velocity_rad_s, -(float)op.velocity, (float)op.velocity, 16);
        uint16_t kp_u = float_to_uint(kp, 0.0f, (float)op.kp, 16);
        uint16_t kd_u = float_to_uint(kd, 0.0f, (float)op.kd, 16);

        w_msg.buf[0] = (pos >> 8);
        w_msg.buf[1] = (uint8_t)pos;
        w_msg.buf[2] = (vel >> 8);
        w_msg.buf[3] = (uint8_t)vel;
        w_msg.buf[4] = (kp_u >> 8);
        w_msg.buf[5] = (uint8_t)kp_u;
        w_msg.buf[6] = (kd_u >> 8);
        w_msg.buf[7] = (uint8_t)kd_u;

        if(can_->write(w_msg) != 1)
            return false;

        return receive_status_frame();
    }

    bool send_velocity_mode_command(float velocity_rad_s){
        // Switch to speed mode if needed
        // Get_RobStrite_Motor_parameter(drw.run_mode.index);
        // enable_motor();
        // if(drw.run_mode.data != Speed_control_mode){
        //     Disenable_Motor(0);
        //     delayMicroseconds(1000);

        //     Set_RobStrite_Motor_parameter(drw.run_mode.index, Speed_control_mode, Set_mode);
        //     delayMicroseconds(1000);

        //     Get_RobStrite_Motor_parameter(drw.run_mode.index);
        //     delayMicroseconds(1000);

        //     enable_motor();

        //     // Example: set some limits (kept from your original)
        //     Set_RobStrite_Motor_parameter(0X7018, 27.0f, Set_parameter);
        //     delayMicroseconds(1000);

        //     Set_RobStrite_Motor_parameter(0X7026, Motor_Set_All_.set_acc, Set_parameter);
        //     delayMicroseconds(1000);
        //     Serial.printf("Switched to Speed Control Mode, Now: %d\n", (int)drw.run_mode.data);
        // }

        // Write speed reference
        Set_RobStrite_Motor_parameter(0X700A, velocity_rad_s, Set_parameter);
        return true;
    }

    // Optional helper (not used by requested 3 functions, but kept)
    bool read_initial_position(float &position, const unsigned long timeout_ms=1000){
        unsigned long start_time = millis();
        CAN_message_t r_msg;
        bool success = false;
        float pos = 0.f;

        while(millis() - start_time < timeout_ms) {
            if(can_->read(r_msg) && (r_msg.len != 0)){
                uint8_t type = (r_msg.id >> 24) & 0xFF;
                uint8_t mid  = (r_msg.id >> 8)  & 0xFF;
                uint8_t eid  = r_msg.id & 0xFF;

                if (type == 0x02 && mid == 0x01 && eid == 0xFD){
                    uint16_t p_uint = (r_msg.buf[0] << 8) | r_msg.buf[1];
                    pos = uint_to_float(p_uint, -4 * M_PI, 4 * M_PI, 16);
                    success = true;
                    break;
                }
            }
        }

        if(success) position = pos;
        return success;
    }

    // ==============
    // Utility conversions
    // ==============
    float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits){
        float span = x_max - x_min;
        return ((float)x_int) * span / ((1 << bits) - 1) + x_min;
    }

    uint16_t float_to_uint(float x, float x_min, float x_max, int bits){
        if (x < x_min) x = x_min;
        if (x > x_max) x = x_max;
        float span = x_max - x_min;
        float offset = x - x_min;
        return static_cast<uint16_t>((offset * ((1 << bits) - 1)) / span);
    }

    float Byte_to_float(uint8_t* bytedata){
        uint32_t data = (uint32_t(bytedata[7]) << 24) |
                        (uint32_t(bytedata[6]) << 16) |
                        (uint32_t(bytedata[5]) << 8)  |
                        (uint32_t(bytedata[4]));
        float data_float;
        memcpy(&data_float, &data, sizeof(float));
        return data_float;
    }

    // ==============
    // (Optional) other modes kept as-is (if you still use them elsewhere)
    // ==============
public:
    bool RobStrite_Motor_PosPP_control(float Speed, float Acceleration, float Angle){
        // if(drw.run_mode.data != PosPP_control_mode){
        //     Disenable_Motor(0);
        //     delayMicroseconds(1000);
        //     Set_RobStrite_Motor_parameter(0X7005, PosPP_control_mode, Set_mode);
        //     delayMicroseconds(1000);
        //     Get_RobStrite_Motor_parameter(0x7005);
        //     delayMicroseconds(1000);
        //     enable_motor();
        //     delayMicroseconds(1000);
        // }

        Motor_Set_All_.set_speed = Speed;
        Motor_Set_All_.set_acc   = Acceleration;
        Motor_Set_All_.set_angle = Angle;

        Set_RobStrite_Motor_parameter(0X7025, Motor_Set_All_.set_speed, Set_parameter);
        delayMicroseconds(1000);

        Set_RobStrite_Motor_parameter(0X7026, Motor_Set_All_.set_acc, Set_parameter);
        delayMicroseconds(1000);

        Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All_.set_angle, Set_parameter);
        delayMicroseconds(1000);

        return true;
    }

    bool RobStrite_Motor_Current_control(float IqCommand, float IdCommand){
        // if(drw.run_mode.data != Elect_control_mode){
        //     Disenable_Motor(0);
        //     delayMicroseconds(1000);
        //     Set_RobStrite_Motor_parameter(0X7005, Elect_control_mode, Set_mode);
        //     delayMicroseconds(1000);
        //     Get_RobStrite_Motor_parameter(0x7005);
        //     delayMicroseconds(1000);
        //     enable_motor();
        //     delayMicroseconds(1000);
        // }

        Motor_Set_All_.set_iq = IqCommand;
        Motor_Set_All_.set_id = IdCommand;

        // NOTE: original code mixed float_to_uint output into "float"
        // Here keep behavior but safer would be to send raw float directly if protocol expects float.
        float iq_u = (float)float_to_uint(Motor_Set_All_.set_iq, SCIQ_MIN, SC_MAX, 16);
        Set_RobStrite_Motor_parameter(0X7006, iq_u, Set_parameter);
        delayMicroseconds(1000);

        Set_RobStrite_Motor_parameter(0X7007, Motor_Set_All_.set_id, Set_parameter);
        delayMicroseconds(1000);

        return true;
    }

    void RobStrite_Motor_Set_Zero_control(){
        Set_RobStrite_Motor_parameter(0X7005, Set_Zero_mode, Set_mode);
    }

    void Set_CAN_ID(uint8_t Set_CAN_ID){
        Disenable_Motor(0);

        CAN_message_t w_msg;
        w_msg.id = (Communication_Type_Can_ID << 24) | (Set_CAN_ID << 16) | (master_id_ << 8) | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 8;
        memset(w_msg.buf, 0, 8);

        can_->write(w_msg);
    }

    bool RobStrite_Motor_PosCSP_control(float Speed, float Angle){
        Motor_Set_All_.set_speed = Speed;
        Motor_Set_All_.set_angle = Angle;

        // if (drw.run_mode.data != PosCSP_control_mode){
        //     Disenable_Motor(0);
        //     delayMicroseconds(1000);
        //     Set_RobStrite_Motor_parameter(0X7005, PosCSP_control_mode, Set_mode);
        //     delayMicroseconds(1000);

        //     Get_RobStrite_Motor_parameter(0x7005);
        //     delayMicroseconds(1000);

        //     enable_motor();
        //     delayMicroseconds(1000);

        //     Motor_Set_All_.set_motor_mode = PosCSP_control_mode;
        // }

        const auto &op = ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type_));
        float sp_u = (float)float_to_uint(Motor_Set_All_.set_speed, -(float)op.velocity, (float)op.velocity, 16);

        Set_RobStrite_Motor_parameter(0X7017, sp_u, Set_parameter);
        delayMicroseconds(1000);

        Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All_.set_angle, Set_parameter);
        delayMicroseconds(1000);

        return true;
    }

    void Set_ZeroPos(){
        Disenable_Motor(0);

        if(drw.run_mode.data != Set_Zero_mode){
            Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);
            delayMicroseconds(1000);
            Get_RobStrite_Motor_parameter(0x7005);
            delayMicroseconds(1000);
        }

        CAN_message_t w_msg;
        w_msg.id = (Communication_Type_SetPosZero << 24) | (master_id_ << 8) | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 8;
        memset(w_msg.buf, 0, 8);
        w_msg.buf[0] = 1;

        can_->write(w_msg);
        enable_motor();
    }
};
