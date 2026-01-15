#pragma once

#include <FlexCAN_T4.h>

#define DAMIAO_ROS2_TIMEOUT 1000 // ms
#define DAMIAO_CAN_TIMEOUT 100 // ms
#define DAMIAO_HOMING_TIMEOUT 3000 // ms (当て止め判定時間)

struct DamiaoFeedBack {
  float position_rad;
  float velocity_rad_s;
  float torque_nm;
  int8_t temp_mos;
  int8_t temp_rotor;
  uint8_t status;
};

struct DamiaoRef{
    int mode = 0; // 0: disable, 1: current control, 2: velocity control, 3: position control, 4: kp, ki, kp, ki
    float position_rad = 0.f;
    float velocity_rad_s = 0.f;
    float kp_asr = 0.f;
    float ki_asr = 0.f;
    float kp_apr = 0.f;
    float ki_apr = 0.f;
};

struct DamiaoStatus{
    static constexpr uint8_t DISABLED = 0x00;
    static constexpr uint8_t ENABLED = 0x01;
    static constexpr uint8_t SENSOR_READ_ERROR = 0x05;
    static constexpr uint8_t MOTOR_PARAMETER_READ_ERROR = 0x06;
    static constexpr uint8_t OVER_PRESSURE = 0x08;
    static constexpr uint8_t UNDER_VOLTAGE = 0x09;
    static constexpr uint8_t OVER_CURRENT = 0x0A;
    static constexpr uint8_t MOS_OVER_TEMPERATURE = 0x0B;
    static constexpr uint8_t MOTOR_COIL_OVER_TEMPERATURE = 0x0C;
    static constexpr uint8_t COMMUNICATION_LOSS = 0x0D;
    static constexpr uint8_t OVERLOAD = 0x0E;
};

// CANの割り込み禁止！
FCTP_CLASS class Damiao {
    struct DamiaoFlash{
        uint32_t CTRL_MODE;
        float P_MAX;
        float V_MAX;
        float T_MAX;
        float KI_ASR; // Speed
        float KP_ASR; // Speed 
        float KP_APR; // Position
        float KI_APR; // Position
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
    unsigned long last_write_ts_ms_ = 0; // 同じCANバスで共通するのでstatic
    unsigned long last_recv_ros2_ts_ms_ = 0;
    unsigned long last_read_feedback_ts_ms_ = 0;
    float offset_pos = 0.f;
    bool no_can_recv = true;

    // ホーミング用変数
    bool homing_done = true;
    float homing_vel = 0.f;
    unsigned long stop_timer = 0;

    public:
        DamiaoFlash flash;
        DamiaoFeedBack feedback;
        DamiaoRef ref;
        bool connect = false;
        float init_pos = 0.f;

        // 外部からセットされるリミットスイッチ状態
        bool limit_state = false;
        int homing_pin = -1;
        
        Damiao(FlexCAN_T4<_bus, _rxSize, _txSize> *can, uint8_t master_id, uint8_t slave_id) : can_(can), master_id_(master_id), slave_id_(slave_id) {
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
            ref.ki_apr = flash.KI_APR;
            ref.kp_apr = flash.KP_APR;
            ref.ki_asr = flash.KI_ASR;
            ref.kp_asr = flash.KP_ASR;
            return true;
        }
        void initHoming(int pin, float vel){
            // pinMode(pin, INPUT);
            // while(digitalRead(pin) == HIGH){
            //     writeVelocityControl(vel);
            //     delay(10);
            // }
            // writeVelocityControl(0.f);

            homing_pin = pin;
            homing_vel = vel;
            homing_done = false;
            stop_timer = millis();
        }
        // ros2から受信したデータをセット
        void setPacketFrame(const float *data, const int len){
            switch(len){
                case 0:
                case 1:
                    ref.mode = len;//disable
                    last_recv_ros2_ts_ms_ = millis();
                    break;
                case 2:
                    ref.mode = len;
                    last_recv_ros2_ts_ms_ = millis();
                    ref.velocity_rad_s = data[1];
                    break;
                case 3:
                    ref.mode = len;
                    last_recv_ros2_ts_ms_ = millis();
                    ref.velocity_rad_s = data[1];
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

        // 現在のデータをCANに書き込む
        void writeCanFrame(){

            // ▼▼▼ ホーミング処理の割り込み ▼▼▼
            if (!homing_done) {
                // まずトルクONを確認
                if(feedback.status != DamiaoStatus::ENABLED){
                    writeCtrlCmd(DamiaoCtrlCmd::TORQUE_ENABLE);
                }
                // 速度制御モードへ
                if(flash.CTRL_MODE != DamiaoControlMode::VELOCITY){
                    writeFlash<uint32_t>(DamiaoFlashAddr::CTRL_MODE, DamiaoControlMode::VELOCITY, flash.CTRL_MODE);
                }

                bool is_limit_hit = false;
                if(homing_pin != -1) {
                    // パターンA: リミットスイッチ (limit_stateはmain.cppで更新される)
                    if(limit_state == true) {
                        is_limit_hit = true;
                    }
                } else {
                    // パターンB: 当て止め (速度が出ている間はタイマーリセット)
                    if(abs(feedback.velocity_rad_s) >= 0.1f) { // 閾値調整 (例えば0.1rad/s)
                        stop_timer = millis();
                    }
                    if((millis() - stop_timer) >= DAMIAO_HOMING_TIMEOUT){
                        is_limit_hit = true;
                    }
                }

                if(is_limit_hit){
                    // 原点設定完了
                    homing_done = true;
                    // 現在位置を基準(init_pos)にするためのオフセット計算
                    // feedback.position_radは生の絶対値が入っていると仮定して計算
                    // 注意: readFeedback内で offset_pos が引かれている場合、ここで再調整が必要
                    // ここでは「現在の生の位置」を「init_pos」にしたい
                    
                    // readFeedbackで引かれているoffsetを一度戻して計算
                    float raw_pos = feedback.position_rad + offset_pos;
                    offset_pos = raw_pos - init_pos;
                    
                    writeVelocityControl(0.f); // 停止
                } else {
                    // ホーミング継続
                    writeVelocityControl(homing_vel);
                }
                
                // フィードバック更新 (これをしないと速度判定などができない)
                readFeedback();
                return; // 通常制御には行かない
            }
            // ▲▲▲ ホーミング処理ここまで ▲▲▲


            int mode = ref.mode;
            if(millis() - last_recv_ros2_ts_ms_ > DAMIAO_ROS2_TIMEOUT){
                mode = 0; // timeout
            }
            switch(mode){
                case 0: //torque off
                    if(feedback.status != DamiaoStatus::DISABLED){
                        writeCtrlCmd(DamiaoCtrlCmd::TORQUE_DISABLE);
                    }
                    writeVelocityControl(0.f); // stop velocity control
                    if(!readFeedback()){
                        Serial.println("Error Read Feed Back");
                    }
                    break;
                // case 1: current contorl
                case 2:
                    // トルクがオフならオンにする
                    if(feedback.status != DamiaoStatus::ENABLED){
                        writeCtrlCmd(DamiaoCtrlCmd::TORQUE_ENABLE);
                    }
                    // モードが速度制御じゃなかったら変更
                    if(flash.CTRL_MODE != DamiaoControlMode::VELOCITY){
                        writeFlash<uint32_t>(DamiaoFlashAddr::CTRL_MODE, DamiaoControlMode::VELOCITY, flash.CTRL_MODE);
                    }
                    writeVelocityControl(ref.velocity_rad_s);
                    readFeedback();
                    break;
                case 3: // position control
                    if(feedback.status != DamiaoStatus::ENABLED){
                        writeCtrlCmd(DamiaoCtrlCmd::TORQUE_ENABLE);
                    }
                    if(flash.CTRL_MODE != DamiaoControlMode::POSITION){ // position control
                        writeFlash<uint32_t>(DamiaoFlashAddr::CTRL_MODE, DamiaoControlMode::POSITION, flash.CTRL_MODE);
                    }
                    writePositionControl(ref.velocity_rad_s, ref.position_rad);
                    readFeedback();
                    break;
                default:
                    break;
            }
            // check pid gain
            if(ref.kp_asr != flash.KP_ASR){
                writeFlash<float>(DamiaoFlashAddr::KP_ASR, ref.kp_asr, flash.KP_ASR);
            }else if(ref.ki_asr != flash.KI_ASR){
                writeFlash<float>(DamiaoFlashAddr::KI_ASR, ref.ki_asr, flash.KI_ASR);
            }else if(ref.kp_apr != flash.KP_APR){
                writeFlash<float>(DamiaoFlashAddr::KP_APR, ref.kp_apr, flash.KP_APR);
            }else if(ref.ki_apr != flash.KI_APR){
                writeFlash<float>(DamiaoFlashAddr::KI_APR, ref.ki_apr, flash.KI_APR);
            }
            // check can connection
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
            unsigned long start_time = millis();

            while (millis() - start_time < timeout_ms) {
                if (!can_->read(r_msg)) {
                    continue;
                }

                // ---- Strict frame shape check ----
                if (r_msg.len != 8) {
                    continue;
                }

                // Optional: If you want to ignore extended frames (RoboStride is extended)
                // This protects you when mixing buses.
                if (r_msg.flags.extended) {
                    continue;
                }

                // ---- Header check: low nibble is slave id ----
                const uint8_t sid = (r_msg.buf[0] & 0x0F);
                if (sid != slave_id_) {
                    continue;
                }

                // ---- Passed: parse ----
                last_read_feedback_ts_ms_ = millis();

                feedback.status = (r_msg.buf[0] & 0xF0) >> 4;

                const uint16_t pos_raw = (uint16_t)((r_msg.buf[1] << 8) | r_msg.buf[2]);
                const uint16_t vel_raw = (uint16_t)((r_msg.buf[3] << 4) | (r_msg.buf[4] >> 4));
                const uint16_t tor_raw = (uint16_t)(((r_msg.buf[4] & 0x0F) << 8) | r_msg.buf[5]);

                feedback.position_rad   = uint_to_float(pos_raw, -flash.P_MAX, flash.P_MAX, 16);
                feedback.velocity_rad_s = uint_to_float(vel_raw, -flash.V_MAX, flash.V_MAX, 12);
                feedback.torque_nm      = uint_to_float(tor_raw, -flash.T_MAX, flash.T_MAX, 12);

                feedback.temp_mos   = (int8_t)r_msg.buf[6];
                feedback.temp_rotor = (int8_t)r_msg.buf[7];

                if (no_can_recv) {
                    offset_pos = feedback.position_rad - init_pos;
                    no_can_recv = false;
                }
                feedback.position_rad -= offset_pos;

                return true;
            }
            return false;
        }

        template<typename T>
        bool writeFlash(const uint8_t &addr, const T w_data, T &r_data, const unsigned long timeout_ms=1000){
            if((millis() - last_write_ts_ms_) < 30){
                return false; // wait for 30 ms
            }

            CAN_message_t w_msg, r_msg;
            w_msg.id = 0x7ff;
            w_msg.len = 8;
            w_msg.buf[0] = slave_id_ & 0xff;
            w_msg.buf[1] = 0x00;
            w_msg.buf[2] = 0x55;
            w_msg.buf[3] = addr;
            memcpy(w_msg.buf + 4, (uint8_t*)&w_data, sizeof(T));

            if(can_->write(w_msg) != 1) {
                return false;
            }
            
            last_write_ts_ms_ = millis();
            
            unsigned long start_time = last_write_ts_ms_;
            bool success = false;
            while(millis() - start_time < timeout_ms) {
                if(can_->read(r_msg) &&
                   (r_msg.len != 8 || r_msg.buf[0] != slave_id_ || r_msg.buf[3] != 0x55)){
                    success = true;
                    break;
                }
            }
            if(success){
                memcpy((uint8_t*)&r_data, r_msg.buf + 4, sizeof(T));
            }
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

            if(can_->write(w_msg) != 1) {
                return false;
            }

            unsigned long start_time = millis();
            bool success = false;
            while(millis() - start_time < timeout_ms) {
                if(can_->read(r_msg) &&
                   (r_msg.len != 8 || r_msg.buf[0] != slave_id_ || r_msg.buf[3] != 0x33)){
                    success = true;
                    break;
                }
            }
            if(success){
                memcpy((uint8_t*)&r_data, r_msg.buf + 4, sizeof(T));
            }
            return success;
        }

        float uint_to_float(uint16_t x, float min, float max, int bits) {
            float normalized = (float)x / (float)((1 << bits) - 1);
            return normalized * (max - min) + min;
        }
};