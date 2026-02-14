#pragma once

// CAN プロトコル仕様
// control信号 / MIT / velocity / position
// 送信 -> 受信 (トルクや位置など)

// Flash読み書き
// 送信 -> 受信 (対応するアドレスのデータ)

#include <FlexCAN_T4.h>
// #include <homing.h>
#include "motor.h"

#define DAMIAO_ROS2_TIMEOUT 1000 // ms
#define DAMIAO_CAN_TIMEOUT 100 // ms
#define DAMIAO_GEAR_RATIO (3591. / 187.)

struct DamiaoStatus{
    static constexpr uint8_t DISABLED = 0x00;
    static constexpr uint8_t ENABLED = 0x01;
    static constexpr uint8_t OVERLOAD = 0x0E;
};

FCTP_CLASS class Damiao : public MotorBase {
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

    // 受信待ち関数
    volatile uint8_t *wait_data = nullptr;
    volatile uint8_t wait_data_cmd = 0;
    volatile uint8_t wait_data_addr = 0;
    volatile size_t wait_data_len = 0;

    public:
        DamiaoFlash flash;
        MotorState &feedback;
        MotorRef &ref;
        bool initialized = false;
        
        // ホーミング判定用のトルク(電流相当)閾値
        // Homing homing;
        
        Damiao(FlexCAN_T4<_bus, _rxSize, _txSize> *can, uint8_t master_id, uint8_t slave_id) 
            : can_(can),
              master_id_(master_id),
              slave_id_(slave_id),
              feedback(motor_state),
              ref(motor_ref) {
            ref.kp_vel = 1.0f;
            ref.ki_vel = 0.1f;
            ref.kp_pos = 5.0f;
            ref.kd_pos = 0.1f;
        }
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
            ref.kp_vel = flash.KI_ASR;
            ref.ki_vel = flash.KP_ASR;
            initialized = true;
            return true;
        }

        void setPacketFrame(const float *data, const int len){
            switch(len){
                case 0:
                    ref.mode = 0;
                    last_recv_ros2_ts_ms_ = millis();
                    break;
                // 固定にした
                case 1: 
                case 2:
                case 3:
                    ref.mode = 2; // velocity mode
                    last_recv_ros2_ts_ms_ = millis();
                    ref.velocity_rad_s = data[len-1];
                    break;
                // case 3: 
                //     ref.mode = 3;
                //     last_recv_ros2_ts_ms_ = millis();
                //     ref.velocity_limit_rad_s = data[1];
                //     ref.position_rad = data[2];
                //     break;
                case 4: 
                    last_recv_ros2_ts_ms_ = millis();
                    ref.kp_vel = data[0];
                    ref.ki_vel = data[1];
                    ref.kp_pos = data[2];
                    ref.kd_pos = data[3];
                    break;
                default:
                    return;
            }
        }

        void writeCanFrame(){
            connect_can = (millis() - last_read_feedback_ts_ms_) < DAMIAO_CAN_TIMEOUT;
            connect_ros2 = (millis() - last_recv_ros2_ts_ms_) < DAMIAO_ROS2_TIMEOUT;
            connect = connect_can;
            // パラメタ更新処理
            if(ref.kp_vel != flash.KP_ASR){
                writeFlash<float>(DamiaoFlashAddr::KP_ASR, ref.kp_vel, flash.KP_ASR);
                return;
            }
            if(ref.ki_vel != flash.KI_ASR){
                writeFlash<float>(DamiaoFlashAddr::KI_ASR, ref.ki_vel, flash.KI_ASR);
                return;
            }

            // ローカルにコピー
            MotorRef damiao_ref = ref;
            // auto homing_state = homing.get(feedback.torque_nm);
            // switch(homing_state.state){
            //     case HOMING_STATE_RUNNING:
            //         // ホーミング中
            //         damiao_ref.mode = 2; // 強制的に速度制御へ
            //         damiao_ref.velocity_rad_s = homing_state.ref_vel;
            //         break;
            //     case HOMING_STATE_SENSOR_OFFSET:
            //         offset_pos = accumulated_pos;
            //     case HOMING_STATE_DONE:
            //     default:
            //         if(!connect_ros2)
            //             damiao_ref.mode = 0;
            //         break;
            // }
            if(!connect_ros2)
                damiao_ref.mode = 0;

            switch(damiao_ref.mode){
                case 0: // Disable
                    if(feedback.status != DamiaoStatus::DISABLED){
                        writeCtrlCmd(DamiaoCtrlCmd::TORQUE_DISABLE);
                        return;
                    }
                    writeVelocityControl(0.f);
                    return;

                case 2: // Velocity
                    if(feedback.status != DamiaoStatus::ENABLED){
                        writeCtrlCmd(DamiaoCtrlCmd::TORQUE_ENABLE);
                        return;
                    }
                    if(flash.CTRL_MODE != DamiaoControlMode::VELOCITY){
                        writeFlash<uint32_t>(DamiaoFlashAddr::CTRL_MODE, DamiaoControlMode::VELOCITY, flash.CTRL_MODE);
                        return;
                    }
                    writeVelocityControl(damiao_ref.velocity_rad_s);
                    return;

                case 3: // Position (External)
                    if(feedback.status != DamiaoStatus::ENABLED){
                        writeCtrlCmd(DamiaoCtrlCmd::TORQUE_ENABLE);
                        return;
                    }
                    if(flash.CTRL_MODE != DamiaoControlMode::VELOCITY){
                        writeFlash<uint32_t>(DamiaoFlashAddr::CTRL_MODE, DamiaoControlMode::VELOCITY, flash.CTRL_MODE);
                        return;
                    }
                    {
                        // 原点出し後の相対位置(0スタート)へ向かって制御
                        float pos_error = damiao_ref.position_rad - feedback.position_rad;
                        ref.velocity_rad_s = pos_error * ref.kp_pos - ref.kd_pos * feedback.velocity_rad_s;

                        // if (ref.velocity_rad_s > ref.velocity_limit_rad_s && ref.velocity_limit_rad_s > 0) 
                        //     ref.velocity_rad_s = ref.velocity_limit_rad_s;
                        // if (ref.velocity_rad_s < -ref.velocity_limit_rad_s && ref.velocity_limit_rad_s > 0) 
                        //     ref.velocity_rad_s = -ref.velocity_limit_rad_s;

                        writeVelocityControl(ref.velocity_rad_s);
                        return;
                    }
                default:
                    break;
            }
        }

        // CANフレーム受信処理
        bool setCanFrame(const CAN_message_t &r_msg){
            // CAN IDチェック
            if(!check_can_id(r_msg))
                return false;
            // 非同期Flash読み出し処理
            if (wait_data != nullptr && check_cmd_addr(r_msg, wait_data_cmd, wait_data_addr)) {
                uint8_t* p = (uint8_t*)wait_data;
                size_t n = wait_data_len;
                wait_data = nullptr;            // ★先にクリア
                memcpy(p, r_msg.buf + 4, n);
                return true;
            }

            last_read_feedback_ts_ms_ = millis();
            feedback.status = (r_msg.buf[0] & 0xF0) >> 4;
            
            uint16_t pos_raw_int = (uint16_t)(r_msg.buf[1] << 8 | r_msg.buf[2]);
            uint16_t vel_raw_int = (uint16_t)((r_msg.buf[3] << 4) | (r_msg.buf[4] >> 4));
            uint16_t tor_raw_int = (uint16_t)(((r_msg.buf[4] & 0x0F) << 8) | r_msg.buf[5]);
            
            // 生の角度 (-P_MAX ~ P_MAX)
            float current_raw_pos = uint_to_float(pos_raw_int, -flash.P_MAX, flash.P_MAX, 16);
            
            feedback.velocity_rad_s = uint_to_float(vel_raw_int, -flash.V_MAX, flash.V_MAX, 12);
            feedback.torque_nm = uint_to_float(tor_raw_int, -flash.T_MAX, flash.T_MAX, 12);
            feedback.temp_mos = (float)((int8_t)r_msg.buf[6]);
            feedback.temp_rotor = (float)((int8_t)r_msg.buf[7]);

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
            feedback.position_rad = (accumulated_pos - offset_pos) / DAMIAO_GEAR_RATIO;
            connect = connect_can;
            return true;
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
            w_msg.len = 8;
            memset(w_msg.buf, 0, 8);
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

        template<typename T>
        bool writeFlash(const uint8_t &addr, const T w_data, T &r_data, const unsigned long timeout_ms=1000){
            if((millis() - last_write_ts_ms_) < 100) return false;
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

            if(initialized){
                wait_data = (uint8_t*)&r_data;
                wait_data_cmd = 0x55;
                wait_data_addr = addr;
                wait_data_len = sizeof(T);
                return true; // 初期化後は割り込みCAN受信のため応答不要
            }

            // 受信待ち
            unsigned long start_time = last_write_ts_ms_;
            bool success = false;
            while(millis() - start_time < timeout_ms) {
                if(can_->read(r_msg) && check_can_id(r_msg) && check_cmd_addr(r_msg, 0x55, addr)){
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
            w_msg.len = 8;
            memset(w_msg.buf, 0, 8);
            w_msg.buf[0] = slave_id_ & 0xff;
            w_msg.buf[1] = 0x00;
            w_msg.buf[2] = 0x33;
            w_msg.buf[3] = addr;
            if(can_->write(w_msg) != 1) return false;
            if(initialized){
                wait_data = (uint8_t*)&r_data;
                wait_data_cmd = 0x33;
                wait_data_addr = addr;
                wait_data_len = sizeof(T);
                return true; // 初期化後は割り込みCAN受信のため応答不要
            }

            // 受信待ち
            last_write_ts_ms_ = millis();
            unsigned long start_time = last_write_ts_ms_;
            bool success = false;
            while(millis() - start_time < timeout_ms) {
                if(can_->read(r_msg) && check_can_id(r_msg) && check_cmd_addr(r_msg, 0x33, addr)){
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

        bool check_can_id(const CAN_message_t &r_msg){
            if(r_msg.id != master_id_) return false;
            if(r_msg.len != 8) return false;
            if ((r_msg.buf[0] & 0x0F) != (slave_id_ & 0x0F)) return false;
            return true;
        }
        bool check_cmd_addr(const CAN_message_t &r_msg, const uint8_t cmd, const uint8_t addr){
            return r_msg.buf[2] == cmd && r_msg.buf[3] == addr;
        }
};
