#pragma once

#include <FlexCAN_T4.h>

#define DAMIAO_ROS2_TIMEOUT 1000 // ms
#define DAMIAO_CAN_TIMEOUT 100 // ms
#define DAMIAO_HOMING_TIMEOUT 3000 // ms

struct DamiaoFeedBack {
  float position_rad;      // ユーザーに見せる位置 (マルチターン - オフセット)
  float velocity_rad_s;
  float torque_nm;
  int8_t temp_mos;
  int8_t temp_rotor;
  uint8_t status;
};

struct DamiaoRef{
    int mode = 0; // 0: disable, 2: velocity, 3: position (external loop)
    float position_rad = 0.f;
    float velocity_rad_s = 0.f;
    float velocity_limit_rad_s = 0.f;
    float kp_asr = 0.f;
    float ki_asr = 0.f;
    float kp_apr = 5.0f; 
    float ki_apr = 0.f;
};

struct DamiaoStatus{
    static constexpr uint8_t DISABLED = 0x00;
    static constexpr uint8_t ENABLED = 0x01;
    static constexpr uint8_t OVERLOAD = 0x0E;
};

FCTP_CLASS class Damiao {
    struct DamiaoFlash{
        uint32_t CTRL_MODE;
        float P_MAX;
        float V_MAX;
        float T_MAX;
        float KI_ASR; 
        float KP_ASR; 
        float KP_APR; 
        float KI_APR;
    };

    struct DamiaoFlashAddr{
        static constexpr uint8_t CTRL_MODE = 0x0A;
        static constexpr uint8_t PMAX = 0x15;
        static constexpr uint8_t VMAX = 0x16;
        static constexpr uint8_t TMAX = 0x17;
        static constexpr uint8_t KP_ASR = 0x19;
        static constexpr uint8_t KI_ASR = 0x1A;
        static constexpr uint8_t KP_APR = 0x1B;
        static constexpr uint8_t KI_APR = 0x1C;
    };

    struct DamiaoControlMode{
        static constexpr uint8_t MIT = 0x01;
        static constexpr uint8_t POSITION = 0x02;
        static constexpr uint8_t VELOCITY = 0x03;
    };

    struct DamiaoCtrlCmd{
        static constexpr uint8_t TORQUE_ENABLE = 0xFC;
        static constexpr uint8_t TORQUE_DISABLE = 0xFD;
    };

    FlexCAN_T4<_bus, _rxSize, _txSize> *can_;
    uint8_t master_id_;
    uint8_t slave_id_;
    unsigned long last_write_ts_ms_ = 0; 
    unsigned long last_recv_ros2_ts_ms_ = 0;
    unsigned long last_read_feedback_ts_ms_ = 0;
    
    // --- マルチターン・積算位置用の変数 ---
    int32_t turns = 0;           // ★追加: 回転数カウンタ
    float accumulated_pos = 0.f; // (turns * range) + raw_pos
    float offset_pos = 0.f;      // 原点設定時の accumulated_pos
    float prev_raw_pos = 0.f;    // 1ステップ前の生位置
    bool is_first_feedback = true; 
    bool no_can_recv = true;

    public:
        // ホーミング用変数
        bool homing_done = true; 
        float homing_vel = 0.f;
        unsigned long stop_timer = 0;

        DamiaoFlash flash;
        DamiaoFeedBack feedback;
        DamiaoRef ref;
        bool connect = false;
        
        // ホーミング判定用のトルク(電流相当)閾値
        float homing_torque_limit = 0.5f;
        bool limit_state = false;
        int homing_pin = -1;
        
        Damiao(FlexCAN_T4<_bus, _rxSize, _txSize> *can, uint8_t master_id, uint8_t slave_id) 
            : can_(can), master_id_(master_id), slave_id_(slave_id) {}
        ~Damiao(){}

        bool init(){
            if(!(readFlash<uint32_t>(DamiaoFlashAddr::CTRL_MODE, flash.CTRL_MODE) &&
                readFlash<float>(DamiaoFlashAddr::PMAX, flash.P_MAX) &&
                readFlash<float>(DamiaoFlashAddr::VMAX, flash.V_MAX) &&
                readFlash<float>(DamiaoFlashAddr::TMAX, flash.T_MAX) &&
                readFlash<float>(DamiaoFlashAddr::KP_ASR, flash.KP_ASR) &&
                readFlash<float>(DamiaoFlashAddr::KI_ASR, flash.KI_ASR) &&
                readFlash<float>(DamiaoFlashAddr::KP_APR, flash.KP_APR) &&
                readFlash<float>(DamiaoFlashAddr::KI_APR, flash.KI_APR) )
            ){
                return false;
            }
            ref.ki_apr = flash.KI_APR;
            ref.kp_apr = flash.KP_APR;
            ref.ki_asr = flash.KI_ASR;
            ref.kp_asr = flash.KP_ASR;
            return true;
        }

        void initHoming(int pin, float vel){
            homing_pin = pin;
            homing_vel = vel;
            homing_done = false;
            stop_timer = millis();
        }

        void setPacketFrame(const float *data, const int len){
            switch(len){
                case 0:
                case 1:
                    ref.mode = 0;
                    last_recv_ros2_ts_ms_ = millis();
                    break;
                case 2: 
                    ref.mode = 2;
                    last_recv_ros2_ts_ms_ = millis();
                    ref.velocity_rad_s = data[1];
                    break;
                case 3: 
                    ref.mode = 3;
                    last_recv_ros2_ts_ms_ = millis();
                    ref.velocity_limit_rad_s = data[1];
                    ref.position_rad = data[2];
                    break;
                case 4: 
                    last_recv_ros2_ts_ms_ = millis();
                    ref.kp_asr = data[0];
                    ref.ki_asr = data[1];
                    ref.kp_apr = data[2];
                    ref.ki_apr = data[3];
                    break;
                default:
                    return;
            }
        }

        void writeCanFrame(){
            readFeedback(); // まず位置更新

            // ▼▼▼ ホーミング処理 ▼▼▼
            if (!homing_done) {
                if(feedback.status != DamiaoStatus::ENABLED){
                    writeCtrlCmd(DamiaoCtrlCmd::TORQUE_ENABLE);
                }
                if(flash.CTRL_MODE != DamiaoControlMode::VELOCITY){
                    writeFlash<uint32_t>(DamiaoFlashAddr::CTRL_MODE, DamiaoControlMode::VELOCITY, flash.CTRL_MODE);
                }

                bool is_limit_hit = false;
                if(homing_pin != -1) {
                    if(limit_state == true) is_limit_hit = true;
                } else {
                    if(abs(feedback.torque_nm) >= homing_torque_limit) is_limit_hit = true;
                }

                if(is_limit_hit){
                    // 現在の積算位置をゼロ点とする
                    offset_pos = accumulated_pos;
                    writeVelocityControl(0.f); 
                    homing_done = true;
                    ref.position_rad = 0.f; 
                    ref.velocity_rad_s = 0.f;
                } else {
                    writeVelocityControl(homing_vel);
                }
                return;
            }

            // ▼▼▼ 通常制御 ▼▼▼
            int mode = ref.mode;
            if(millis() - last_recv_ros2_ts_ms_ > DAMIAO_ROS2_TIMEOUT){
                mode = 0;
            }

            switch(mode){
                case 0: // Disable
                    if(feedback.status != DamiaoStatus::DISABLED){
                        writeCtrlCmd(DamiaoCtrlCmd::TORQUE_DISABLE);
                    }
                    writeVelocityControl(0.f);
                    break;

                case 2: // Velocity
                    if(feedback.status != DamiaoStatus::ENABLED){
                        writeCtrlCmd(DamiaoCtrlCmd::TORQUE_ENABLE);
                    }
                    if(flash.CTRL_MODE != DamiaoControlMode::VELOCITY){
                        writeFlash<uint32_t>(DamiaoFlashAddr::CTRL_MODE, DamiaoControlMode::VELOCITY, flash.CTRL_MODE);
                    }
                    writeVelocityControl(ref.velocity_rad_s);
                    break;

                case 3: // Position (External)
                    if(feedback.status != DamiaoStatus::ENABLED){
                        writeCtrlCmd(DamiaoCtrlCmd::TORQUE_ENABLE);
                    }
                    if(flash.CTRL_MODE != DamiaoControlMode::VELOCITY){
                        writeFlash<uint32_t>(DamiaoFlashAddr::CTRL_MODE, DamiaoControlMode::VELOCITY, flash.CTRL_MODE);
                    }
                    {
                        // 原点出し後の相対位置(0スタート)へ向かって制御
                        float pos_error = ref.position_rad - feedback.position_rad;
                        ref.velocity_rad_s = pos_error * ref.kp_apr;

                        if (ref.velocity_rad_s > ref.velocity_limit_rad_s && ref.velocity_limit_rad_s > 0) 
                            ref.velocity_rad_s = ref.velocity_limit_rad_s;
                        if (ref.velocity_rad_s < -ref.velocity_limit_rad_s && ref.velocity_limit_rad_s > 0) 
                            ref.velocity_rad_s = -ref.velocity_limit_rad_s;

                        writeVelocityControl(ref.velocity_rad_s);
                    }
                    break;
                default:
                    break;
            }

            if(ref.kp_asr != flash.KP_ASR){
                writeFlash<float>(DamiaoFlashAddr::KP_ASR, ref.kp_asr, flash.KP_ASR);
            } else if(ref.ki_asr != flash.KI_ASR){
                writeFlash<float>(DamiaoFlashAddr::KI_ASR, ref.ki_asr, flash.KI_ASR);
            }
            connect = (millis() - last_recv_ros2_ts_ms_) < DAMIAO_CAN_TIMEOUT;
        }

    private:
        bool writePositionControl(const float velocity_rad_s, const float position_rad){
            CAN_message_t w_msg;
            w_msg.id = 0x100 + slave_id_;
            w_msg.len = 8;
            memcpy(w_msg.buf, (uint8_t*)&position_rad, sizeof(float));
            memcpy(w_msg.buf + 4, (uint8_t*)&velocity_rad_s, sizeof(float));
            return can_->write(w_msg) == 1;
        }
        bool writeVelocityControl(float velocity_rad_s){
            CAN_message_t w_msg;
            w_msg.id = 0x200 + slave_id_;
            w_msg.len = 4;
            memcpy(w_msg.buf, (uint8_t*)&velocity_rad_s, sizeof(float));
            return can_->write(w_msg) == 1;
        }

        bool writeCtrlCmd(const uint8_t cmd){
            CAN_message_t w_msg;
            w_msg.id = slave_id_;
            w_msg.len = 8;
            for (int i = 0; i < 7; i++) { w_msg.buf[i] = 0xff;}
            w_msg.buf[7] = cmd;
            return can_->write(w_msg) == 1;
        }

        bool readFeedback(const unsigned long timeout_ms = 1000){
            CAN_message_t r_msg;
            bool success = false;
            unsigned long start_time = millis();
            
            while(millis() - start_time < timeout_ms) {
                if(can_->read(r_msg)) {
                    if (r_msg.id == master_id_ && r_msg.len == 8 && (r_msg.buf[0] & 0x0F) == slave_id_) {
                        success = true;
                        break;
                    }
                }
            }

            if(success){
                last_read_feedback_ts_ms_ = millis();
                feedback.status = (r_msg.buf[0] & 0xF0) >> 4;
                
                uint16_t pos_raw_int = (uint16_t)(r_msg.buf[1] << 8 | r_msg.buf[2]);
                uint16_t vel_raw_int = (uint16_t)((r_msg.buf[3] << 4) | (r_msg.buf[4] >> 4));
                uint16_t tor_raw_int = (uint16_t)(((r_msg.buf[4] & 0x0F) << 8) | r_msg.buf[5]);
                
                // 生の角度 (-P_MAX ~ P_MAX)
                float current_raw_pos = uint_to_float(pos_raw_int, -flash.P_MAX, flash.P_MAX, 16);
                
                feedback.velocity_rad_s = uint_to_float(vel_raw_int, -flash.V_MAX, flash.V_MAX, 12);
                feedback.torque_nm = uint_to_float(tor_raw_int, -flash.T_MAX, flash.T_MAX, 12);
                feedback.temp_mos = (int8_t)r_msg.buf[6];
                feedback.temp_rotor = (int8_t)r_msg.buf[7];

                // ★積算処理のロジック変更 (Turns + Raw)
                float full_range = 2.0f * flash.P_MAX;

                if (is_first_feedback) {
                    prev_raw_pos = current_raw_pos;
                    turns = 0;
                    // 初回は turns=0 として計算
                    accumulated_pos = current_raw_pos; 
                    is_first_feedback = false;
                    
                    if(no_can_recv) {
                        offset_pos = accumulated_pos; 
                        no_can_recv = false;
                    }
                } else {
                    float diff = current_raw_pos - prev_raw_pos;
                    
                    // 回転数のカウントアップ・ダウン判定
                    // 閾値を P_MAX (範囲の半分) に設定し、それを超える急激な変化はラップアラウンドとみなす
                    if (diff < -flash.P_MAX) {
                        // 正方向に境界を跨いだ (例: 3.1 -> -3.1 => diff = -6.2)
                        turns++;
                    } else if (diff > flash.P_MAX) {
                        // 負方向に境界を跨いだ (例: -3.1 -> 3.1 => diff = 6.2)
                        turns--;
                    }
                    
                    // 最終位置 = (回転数 * 1周の範囲) + 生の位置
                    accumulated_pos = (turns * full_range) + current_raw_pos;
                    
                    prev_raw_pos = current_raw_pos;
                }

                // 外部出力用 (原点オフセット考慮)
                feedback.position_rad = accumulated_pos - offset_pos;
            }
            return success;
        }

        template<typename T>
        bool writeFlash(const uint8_t &addr, const T w_data, T &r_data, const unsigned long timeout_ms=1000){
            if((millis() - last_write_ts_ms_) < 30) return false;
            CAN_message_t w_msg, r_msg;
            w_msg.id = 0x7ff;
            w_msg.len = 8;
            w_msg.buf[0] = slave_id_ & 0xff;
            w_msg.buf[1] = 0x00;
            w_msg.buf[2] = 0x55;
            w_msg.buf[3] = addr;
            memcpy(w_msg.buf + 4, (uint8_t*)&w_data, sizeof(T));
            if(can_->write(w_msg) != 1) return false;
            last_write_ts_ms_ = millis();
            unsigned long start_time = last_write_ts_ms_;
            bool success = false;
            while(millis() - start_time < timeout_ms) {
                if(can_->read(r_msg) && (r_msg.len != 8 || r_msg.buf[0] != slave_id_ || r_msg.buf[3] != 0x55)){
                    success = true;
                    break;
                }
            }
            if(success) memcpy((uint8_t*)&r_data, r_msg.buf + 4, sizeof(T));
            return success;
        }

        template<typename T>
        bool readFlash(const uint8_t &addr, T &r_data, const unsigned long timeout_ms=1000){
            CAN_message_t w_msg, r_msg;
            w_msg.id = 0x7ff;
            w_msg.len = 4;
            w_msg.buf[0] = slave_id_ & 0xff;
            w_msg.buf[1] = 0x00;
            w_msg.buf[2] = 0x33;
            w_msg.buf[3] = addr;
            if(can_->write(w_msg) != 1) return false;
            unsigned long start_time = millis();
            bool success = false;
            while(millis() - start_time < timeout_ms) {
                if(can_->read(r_msg) && (r_msg.len != 8 || r_msg.buf[0] != slave_id_ || r_msg.buf[3] != 0x33)){
                    success = true;
                    break;
                }
            }
            if(success) memcpy((uint8_t*)&r_data, r_msg.buf + 4, sizeof(T));
            return success;
        }

        float uint_to_float(uint16_t x, float min, float max, int bits) {
            float normalized = (float)x / (float)((1 << bits) - 1);
            return normalized * (max - min) + min;
        }
};