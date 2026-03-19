#pragma once

#include <FlexCAN_T4.h>
#include <map>
#include <math.h>
#include <string.h>
#include <Arduino.h>
#include "motor.h"

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
#define ROBOSTRIDE_RECONNECT_RETRY_MS 200 // ms
#define ROBOSTRIDE_RECONNECT_STEP_TIMEOUT_MS 100 // ms

// init delay
#define ROBOSTRIDE_INIT_DELAY_MS 100

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
#define Communication_Type_SetBaudRate           0x17  // 设置电机波特率（重启生效）

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

enum class ActuatorType {
    ROBSTRIDE_00 = 0,
    ROBSTRIDE_01 = 1,
    ROBSTRIDE_02 = 2,
    ROBSTRIDE_03 = 3,
    ROBSTRIDE_04 = 4,
    ROBSTRIDE_05 = 5,
    ROBSTRIDE_06 = 6
};

enum class RoboStrideBaudRate : uint8_t {
    BAUD_1M = 0x01,
    BAUD_500K = 0x02,
    BAUD_250K = 0x03,
    BAUD_125K = 0x04,
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

FCTP_CLASS class RoboStride : public MotorBase {
    enum ReconnectStep : uint8_t {
        RECONNECT_IDLE = 0,
        RECONNECT_SEND_DISABLE,
        RECONNECT_WAIT_DISABLE,
        RECONNECT_SEND_SET_MODE,
        RECONNECT_WAIT_SET_MODE,
        RECONNECT_SEND_GET_MODE,
        RECONNECT_WAIT_GET_MODE,
        RECONNECT_SEND_ENABLE,
        RECONNECT_WAIT_ENABLE,
        RECONNECT_RETRY_WAIT,
    };

    uint8_t master_id_, motor_id_;
    int actuator_type_;
    FlexCAN_T4<_bus, _rxSize, _txSize> *can_;

    data_read_write_one params_;

    uint8_t error_code_ = 0;
    uint8_t pattern_ = 0;

    Motor_Set Motor_Set_All_ = {0};

    unsigned long last_recv_can_ts_ms_ = 0;
    unsigned long last_recv_ros2_ts_ms_ = 0;
    uint8_t configured_run_mode_ = Speed_control_mode;

    // Track multi-turn position state similar to Damiao implementation
    int32_t position_turns_ = 0;
    float accumulated_position_rad_ = 0.f;
    float prev_raw_position_rad_ = 0.f;
    bool first_feedback_received_ = true;

    float mit_velocity_command_rad_s_ = 0.f; // For velocity control mode
    float mit_torque_command_nm_ = 0.f; // For torque control mode

    float pos_offset_ = 0.0f;
    bool prev_connect_can_ = false;
    bool reconnecting_ = false;
    ReconnectStep reconnect_step_ = RECONNECT_IDLE;
    unsigned long reconnect_step_started_ms_ = 0;
    unsigned long reconnect_retry_started_ms_ = 0;

public:
    MotorState &feedback;
    MotorRef &ref;
    data_read_write drw;

    RoboStride(FlexCAN_T4<_bus, _rxSize, _txSize> *can, uint8_t master_id, uint8_t motor_id, int actuator_type, float pos_offset) :
        master_id_(master_id),
        motor_id_(motor_id),
        actuator_type_(actuator_type),
        can_(can),
        feedback(motor_state),
        ref(motor_ref),
        pos_offset_(pos_offset)
    {
        ref.position_rad = 0.0f;
        ref.velocity_rad_s = 0.0f;
        ref.torque_nm = 0.0f;
        ref.kp_vel = 2.0f;
        ref.ki_vel = 0.021f;
        ref.kp_pos = 10.0f;
        ref.kd_pos = 0.0f;
    }
    ~RoboStride(){}

    // ===========
    // Requested APIs
    // ===========

    bool Set_Baud_Rate(RoboStrideBaudRate baud_rate, bool wait_response = false){
        CAN_message_t w_msg{};
        w_msg.id = (Communication_Type_SetBaudRate << 24) | (master_id_ << 8) | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 8;

        // RoboStride private protocol: fixed key sequence + baud command byte.
        w_msg.buf[0] = 0x01;
        w_msg.buf[1] = 0x02;
        w_msg.buf[2] = 0x03;
        w_msg.buf[3] = 0x04;
        w_msg.buf[4] = 0x05;
        w_msg.buf[5] = 0x06;
        w_msg.buf[6] = static_cast<uint8_t>(baud_rate);
        w_msg.buf[7] = 0x00;

        if(can_->write(w_msg) != 1){
            return false;
        }
        if(!wait_response){
            return true;
        }
        return receive_status_frame();
    }

    bool init(float speed_limit, float acc_limit, uint8_t init_run_mode, bool set_gain, bool check_gain){
        Serial.printf("[RoboStride:init] start (motor_id=%u)\n", motor_id_);
        auto log_step = [&](const char* step, bool ok) -> bool {
            Serial.print("[RoboStride:init] ");
            Serial.print(step);
            Serial.println(ok ? " OK" : " FAIL");
            return ok;
        };
        auto set_param = [&](uint16_t idx, float value, char set_mode=Set_parameter) -> bool {
            if(!log_step("Set parameter", Set_RobStrite_Motor_parameter(idx, value, set_mode))) {
                Serial.printf("[RoboStride:init] Failed to set parameter 0x%04X\n", idx);
                return false;
            }
            delay(ROBOSTRIDE_INIT_DELAY_MS);
            return true;
        };
        auto check_param = [&](data_read_write_one &eeprom, float expected) -> bool {
            if(!log_step("Get parameter", Get_RobStrite_Motor_parameter(eeprom.index))) {
                Serial.printf("[RoboStride:init] Failed to get parameter 0x%04X\n", eeprom.index);
                return false;
            }
            if (fabsf(eeprom.data - expected) > 1e-3f) {
                Serial.printf("[RoboStride:init] Parameter mismatch for 0x%04X: expected=%.5f actual=%.5f\n", eeprom.index, expected, eeprom.data);
                return false;
            }
            Serial.printf("[RoboStride:init] Parameter match for 0x%04X: %.5f\n", eeprom.index, eeprom.data);
            return true;
        };
        auto enableMotor = [&]() -> bool {
            if(!log_step("Enable motor", enable_motor())) return false;
            delay(ROBOSTRIDE_INIT_DELAY_MS);
            return true;
        };
        auto disableMotor = [&]() -> bool {
            if(!log_step("Disable motor", Disenable_Motor(0))) return false;
            delay(ROBOSTRIDE_INIT_DELAY_MS);
            return true;
        };
        configured_run_mode_ = init_run_mode;
        
        disableMotor();

        if(!set_param(drw.run_mode.index, (float)configured_run_mode_, Set_mode)) return false;
        if(!check_param(drw.run_mode, (float)configured_run_mode_)) return false;


        if(set_gain){
            // Speed Kp Gain
            if(!set_param(drw.spd_kp.index, ref.kp_vel)) return false;
            if(!set_param(drw.spd_ki.index, ref.ki_vel)) return false;
            if(!set_param(drw.loc_kp.index, ref.kp_pos)) return false;
            // if(!set_param(drw.acc_limit.index, acc_limit)) return false;
            // if(!set_param(drw.speed_limit.index, speed_limit)) return false;
        }

        if(check_gain){
            if(!check_param(drw.spd_kp, ref.kp_vel)) return false;
            if(!check_param(drw.spd_ki, ref.ki_vel)) return false;
            if(!check_param(drw.loc_kp, ref.kp_pos)) return false;
            // if(!check_param(drw.acc_limit, acc_limit)) return false;
            // if(!check_param(drw.speed_limit, speed_limit)) return false;
        }

        enableMotor();

        // Seed timestamp so we don't instantly timeout
        last_recv_ros2_ts_ms_ = millis();

        // if(!receive_status_frame()) return false;
        Serial.println("[RoboStride:init] success");
        return true;
    }

    void setPacketFrame(const float *data, const int len){
        switch(len){
            case 0:
            // case 1:
                ref.mode = 0; // disable
                last_recv_ros2_ts_ms_ = millis();
                break;
            case 1:    
            case 2:
            case 3:
                ref.mode = (configured_run_mode_ == PosPP_control_mode) ? 3 : 2; // position or velocity based on configured mode
                last_recv_ros2_ts_ms_ = millis();
                if(configured_run_mode_ == PosPP_control_mode){
                    ref.position_rad = data[len-1];
                }else{
                    ref.velocity_rad_s = data[len-1];
                }
                break;

            case 4:
                // param/gain update packet
                last_recv_ros2_ts_ms_ = millis();
                ref.kp_vel = data[0];
                ref.ki_vel = data[1];
                ref.kp_pos = data[2];
                // ref.pos_kd     = data[3];
                break;

            default:
                return;
        }
    }

    bool setCanFrame(const CAN_message_t &r_msg){
        ReceiveResult result;
        if(!decodeCanFrame(r_msg, result)){
            return false;
        }
        if(!applyReceiveResult(result)){
            return false;
        }
        last_recv_can_ts_ms_ = millis();
        if (reconnecting_) {
            handleReconnectResponse(result);
        }
        connect = connect_can;
        return true;
    }

    void writeCanFrame(){
        connect_can = (millis() - last_recv_can_ts_ms_) < ROBOSTRIDE_CAN_TIMEOUT;
        connect_ros2 = (millis() - last_recv_ros2_ts_ms_) < ROBOSTRIDE_ROS2_TIMEOUT;
        connect = connect_can;
        if (prev_connect_can_ && !connect_can && !reconnecting_) {
            startReconnect();
        }
        prev_connect_can_ = connect_can;

        if (reconnecting_) {
            processReconnect();
            return;
        }

        // timeout => disable
        int mode = ref.mode;
        if(!connect_ros2){
            mode = 0;
        }

        // Gain sync: optimistic local update after non-blocking write.
        // if (ref.kp_vel != drw.spd_kp.data) {
        //     Set_RobStrite_Motor_parameter(drw.spd_kp.index, ref.kp_vel, Set_parameter, false);
        //     drw.spd_kp.data = ref.kp_vel;
        //     return;
        // }
        // if (ref.ki_vel != drw.spd_ki.data) {
        //     Set_RobStrite_Motor_parameter(drw.spd_ki.index, ref.ki_vel, Set_parameter, false);
        //     drw.spd_ki.data = ref.ki_vel;
        //     return;
        // }
        // if (ref.kp_pos != drw.loc_kp.data) {
        //     Set_RobStrite_Motor_parameter(drw.loc_kp.index, ref.kp_pos, Set_parameter, false);
        //     drw.loc_kp.data = ref.kp_pos;
        //     return;
        // }

        // ---- control ----
        switch(mode){
            case 0: {
                if (configured_run_mode_ == PosPP_control_mode) {
                    // Hold current angle in PosPP mode.
                    send_pospp_mode_command(0.0f + pos_offset_, false);
                } else {
                    send_velocity_mode_command(0, false);
                }
                break;
            }

            case 2: {
                if (configured_run_mode_ == Speed_control_mode) {
                    send_velocity_mode_command(ref.velocity_rad_s, false);
                }
                break;
            }

            case 3: {
                if (configured_run_mode_ == PosPP_control_mode) {
                    // Position control in PosPP mode: command target angle only.
                    // send_motion_command(
                    //     mit_torque_command_nm_,
                    //     ref.position_rad,
                    //     mit_velocity_command_rad_s_,
                    //     ref.kp_pos,
                    //     ref.kd_pos,
                    //     false
                    // );
                    send_pospp_mode_command(ref.position_rad + pos_offset_, false);
                }
                break;
            }

            default:
                break;
        }
    }

private:
    void startReconnect() {
        reconnecting_ = true;
        reconnect_step_ = RECONNECT_SEND_DISABLE;
        reconnect_step_started_ms_ = millis();
        reconnect_retry_started_ms_ = 0;
    }

    void enterReconnectRetry() {
        reconnect_step_ = RECONNECT_RETRY_WAIT;
        reconnect_retry_started_ms_ = millis();
    }

    void finishReconnect() {
        reconnecting_ = false;
        reconnect_step_ = RECONNECT_IDLE;
        reconnect_step_started_ms_ = 0;
        reconnect_retry_started_ms_ = 0;
    }

    void processReconnect() {
        const unsigned long now_ms = millis();
        switch (reconnect_step_) {
            case RECONNECT_SEND_DISABLE:
                if (Disenable_Motor(0, false)) {
                    reconnect_step_ = RECONNECT_WAIT_DISABLE;
                    reconnect_step_started_ms_ = now_ms;
                } else {
                    enterReconnectRetry();
                }
                return;

            case RECONNECT_WAIT_DISABLE:
                if ((now_ms - reconnect_step_started_ms_) > ROBOSTRIDE_RECONNECT_STEP_TIMEOUT_MS) {
                    enterReconnectRetry();
                }
                return;

            case RECONNECT_SEND_SET_MODE:
                if (Set_RobStrite_Motor_parameter(drw.run_mode.index, (float)configured_run_mode_, Set_mode, false)) {
                    reconnect_step_ = RECONNECT_WAIT_SET_MODE;
                    reconnect_step_started_ms_ = now_ms;
                } else {
                    enterReconnectRetry();
                }
                return;

            case RECONNECT_WAIT_SET_MODE:
                if ((now_ms - reconnect_step_started_ms_) > ROBOSTRIDE_RECONNECT_STEP_TIMEOUT_MS) {
                    enterReconnectRetry();
                }
                return;

            case RECONNECT_SEND_GET_MODE:
                if (Get_RobStrite_Motor_parameter(drw.run_mode.index, false)) {
                    reconnect_step_ = RECONNECT_WAIT_GET_MODE;
                    reconnect_step_started_ms_ = now_ms;
                } else {
                    enterReconnectRetry();
                }
                return;

            case RECONNECT_WAIT_GET_MODE:
                if ((now_ms - reconnect_step_started_ms_) > ROBOSTRIDE_RECONNECT_STEP_TIMEOUT_MS) {
                    enterReconnectRetry();
                }
                return;

            case RECONNECT_SEND_ENABLE:
                if (enable_motor(false)) {
                    reconnect_step_ = RECONNECT_WAIT_ENABLE;
                    reconnect_step_started_ms_ = now_ms;
                } else {
                    enterReconnectRetry();
                }
                return;

            case RECONNECT_WAIT_ENABLE:
                if ((now_ms - reconnect_step_started_ms_) > ROBOSTRIDE_RECONNECT_STEP_TIMEOUT_MS) {
                    enterReconnectRetry();
                }
                return;

            case RECONNECT_RETRY_WAIT:
                if ((now_ms - reconnect_retry_started_ms_) >= ROBOSTRIDE_RECONNECT_RETRY_MS) {
                    reconnect_step_ = RECONNECT_SEND_DISABLE;
                    reconnect_step_started_ms_ = now_ms;
                }
                return;

            case RECONNECT_IDLE:
            default:
                finishReconnect();
                return;
        }
    }

    void handleReconnectResponse(const ReceiveResult &result) {
        switch (reconnect_step_) {
            case RECONNECT_WAIT_DISABLE:
                reconnect_step_ = RECONNECT_SEND_SET_MODE;
                reconnect_step_started_ms_ = millis();
                return;

            case RECONNECT_WAIT_SET_MODE:
                reconnect_step_ = RECONNECT_SEND_GET_MODE;
                reconnect_step_started_ms_ = millis();
                return;

            case RECONNECT_WAIT_GET_MODE:
                if (result.communication_type == Communication_Type_GetSingleParameter &&
                    params_.index == drw.run_mode.index &&
                    static_cast<uint8_t>(drw.run_mode.data) == configured_run_mode_) {
                    reconnect_step_ = RECONNECT_SEND_ENABLE;
                    reconnect_step_started_ms_ = millis();
                    return;
                }
                enterReconnectRetry();
                return;

            case RECONNECT_WAIT_ENABLE:
                finishReconnect();
                return;

            default:
                return;
        }
    }

    bool decodeCanFrame(const CAN_message_t &r_msg, ReceiveResult &result){
        if (!r_msg.flags.extended) {
            return false;
        }

        const uint32_t can_id = r_msg.id;
        const uint8_t comm_type = (uint8_t)((can_id >> 24) & 0x1F);
        const uint8_t id_src = (uint8_t)((can_id >> 8) & 0xFF);   // bit15..8
        const uint8_t id_dst = (uint8_t)(can_id & 0xFF);          // bit7..0
        const uint16_t data_area2 = (uint16_t)((can_id >> 8) & 0xFFFF);

        // Receive side accepts only reply-direction frames.
        // reply: bit15..8 = motor CAN_ID, bit7..0 = host/master CAN_ID
        bool id_ok = false;
        switch (comm_type) {
            case Communication_Type_MotorRequest:        // type 2
            case Communication_Type_GetSingleParameter:  // type 17 reply
            case Communication_Type_ErrorFeedback:       // type 21
                id_ok = (id_src == motor_id_) && (id_dst == master_id_);
                break;
            case Communication_Type_Get_ID:              // type 0 reply
                // Type 0 reply uses 0xFE in bit7..0 per manual.
                id_ok = (id_src == motor_id_) && ((id_dst == 0xFE) || (id_dst == master_id_));
                break;
            default:
                return false;
        }
        if (!id_ok) {
            return false;
        }

        result.communication_type = comm_type;
        result.extra_data = data_area2;
        result.host_id = id_dst;

        error_code_ = (uint8_t)((can_id >> 16) & 0x3F);
        pattern_    = (uint8_t)((can_id >> 22) & 0x03);

        memcpy(result.data, r_msg.buf, r_msg.len);
        result.len = r_msg.len;
        return true;
    }

    bool applyReceiveResult(const ReceiveResult &result){
        // This implementation only parses 8-byte RoboStride frames.
        if (result.len < 8) {
            return false;
        }

        if (result.communication_type == Communication_Type_MotorRequest) {
            const uint16_t pos_u16  = (uint16_t)((result.data[0] << 8) | result.data[1]);
            const uint16_t vel_u16  = (uint16_t)((result.data[2] << 8) | result.data[3]);
            const uint16_t tor_u16  = (uint16_t)((result.data[4] << 8) | result.data[5]);
            const uint16_t temp_u16 = (uint16_t)((result.data[6] << 8) | result.data[7]);

            const auto &op = ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type_));

            const float current_raw_pos = uint_to_float(pos_u16, -(float)op.position, (float)op.position, 16);
            const float half_range = (float)op.position;
            const float full_range = half_range * 2.0f;

            if (first_feedback_received_) {
                position_turns_ = 0;
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
            feedback.position_rad = accumulated_position_rad_ - pos_offset_;
            feedback.velocity_rad_s = uint_to_float(vel_u16, -(float)op.velocity, (float)op.velocity, 16);
            feedback.torque_nm = uint_to_float(tor_u16, -(float)op.torque, (float)op.torque, 16);
            feedback.temp_mos = (float)temp_u16 * 0.1f;
            return true;
        }

        if (result.communication_type == Communication_Type_GetSingleParameter) {
            const uint16_t idx = (uint16_t)((result.data[1] << 8) | result.data[0]);
            params_.index = idx;

            if (idx == Index_List[0]) {
                drw.run_mode.data = (float)uint8_t(result.data[4]);
                params_.data = drw.run_mode.data;
                return true;
            }

            const float v = Byte_to_float(result.data);
            params_.data = v;

            const int N = (int)(sizeof(Index_List) / sizeof(Index_List[0]));
            for (int i = 0; i < N; i++) {
                if (idx == Index_List[i]) {
                    switch(i){
                        case 1:  drw.iq_ref.data = v; break;
                        case 2:  drw.spd_ref.data = v; break;
                        case 3:  drw.imit_torque.data = v; break;
                        case 4:  drw.cur_kp.data = v; break;
                        case 5:  drw.cur_ki.data = v; break;
                        case 6:  drw.cur_filt_gain.data = v; break;
                        case 7:  drw.loc_ref.data = v; break;
                        case 8:  drw.limit_spd.data = v; break;
                        case 9:  drw.limit_cur.data = v; break;
                        case 10: drw.mechPos.data = v; break;
                        case 11: drw.iqf.data = v; break;
                        case 12: drw.mechVel.data = v; break;
                        case 13: drw.VBUS.data = v; break;
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

        if (result.communication_type == Communication_Type_ErrorFeedback) {
            return true;
        }

        return false;
    }

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
            // debug
            Serial.printf("[RoboStride] Received CAN frame: ID=0x%08X, DLC=%u, Data=", r_msg.id, r_msg.len);
            for(int i = 0; i < r_msg.len; i++){
                Serial.printf("%02X ", r_msg.buf[i]);
            }
            Serial.println();
            // decode
            if (decodeCanFrame(r_msg, result)) {
                return true;
            }
        }
    } 

    return false;
}

    bool receive_status_frame(){
        ReceiveResult result;
        if(!receive(result, 10)){
            Serial.println("[RoboStride] receive_status_frame timeout");
        }
        if (applyReceiveResult(result)) {
            return true;
        }
        return false;
    }

    // ==============
    // Parameter read/write
    // ==============
    bool Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode, bool wait_response = true){
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
        if(!wait_response) {
            return true;
        }
        return receive_status_frame();
    }

    bool Get_RobStrite_Motor_parameter(uint16_t Index, bool wait_response = true){
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
        if(!wait_response) {
            return true;
        }
        return receive_status_frame();
    }

    // ==============
    // Enable/Disable
    // ==============
    bool enable_motor(bool wait_response = true){
        CAN_message_t w_msg;
        w_msg.id = (Communication_Type_MotorEnable << 24) | (master_id_ << 8) | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 8;
        memset(w_msg.buf, 0, 8);

        if (can_->write(w_msg) != 1) {
            return false;
        }
        if(!wait_response) {
            return true;
        }
        return receive_status_frame();
    }

    bool Disenable_Motor(uint8_t clear_error, bool wait_response = true){
        CAN_message_t w_msg{};
        w_msg.id = (Communication_Type_MotorStop << 24) | (master_id_ << 8) | motor_id_;
        w_msg.flags.extended = 1;
        w_msg.len = 8;
        memset(w_msg.buf, 0, 8);

        w_msg.buf[0] = clear_error;

        if(can_->write(w_msg) != 1)
            return false;
        if(!wait_response) {
            return true;
        }
        return receive_status_frame();
    }

    // ==============
    // Control commands
    // ==============
    bool send_motion_command(float torque,
                             float position_rad,
                             float velocity_rad_s,
                             float kp,
                             float kd,
                             bool wait_response = true)
    {
        // Ensure run_mode = move_control_mode (0) if required
        // if(drw.run_mode.data != 0){
        //     Disenable_Motor(0, wait_response);
        //     delayMicroseconds(1000);
        //     Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode, wait_response);
        //     delayMicroseconds(1000);
        //     Get_RobStrite_Motor_parameter(0x7005, wait_response);
        //     delayMicroseconds(1000);
        // }

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

        if(!wait_response) {
            return true;
        }
        return receive_status_frame();
    }

    bool send_velocity_mode_command(float velocity_rad_s, bool wait_response = true){
        // Write speed reference
        Set_RobStrite_Motor_parameter(0X700A, velocity_rad_s, Set_parameter, wait_response);
        return true;
    }

    bool send_pospp_mode_command(float position_rad, bool wait_response = true){
        Set_RobStrite_Motor_parameter(0X7016, position_rad, Set_parameter, wait_response);
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

    float Byte_to_float(const uint8_t* bytedata){
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
