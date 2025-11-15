#pragma once

#include "rclcpp/rclcpp.hpp"
#include <inttypes.h>

// SOEMのコンテキストと構造体をinclude
#include "soem/soem.h"

// SOEMのグローバル変数をクラスメンバーとして定義
class CoreHardware : public rclcpp::Node{
public:
    CoreHardware();
    ~CoreHardware();

private:
    // --- SOEM関連のメンバー変数 ---
    static constexpr int NSEC_PER_SEC = 1000000000;
    static constexpr int EC_TIMEOUTMON = 500;
    // static constexpr uint8_t EC_TIMEOUTRET = 50; // SOEMの定義に合わせる

    uint8_t IOmap_[4096];
    int expectedWKC_ = 0;
    int wkc_ = 0;
    bool mapping_done_ = false;
    bool dorun_ = false;
    bool in_op_ = false;
    int dowkccheck_ = 0;
    int current_group_ = 0;
    int cycle_ = 0;
    int64_t cycle_time_ns_ = 1000000; // 1ms (サンプルの1000000ns)
    
    ecx_contextt ctx_;
    
    float p_gain_ = 0.01f;
    float i_gain_ = 0.00002f;
    int64_t sync_offset_ns_ = 500000; // 500us

    // --- ROS 2関連のメンバー変数 ---
    rclcpp::TimerBase::SharedPtr timer_;
    std::string if_name_;
    
    // --- メンバ関数 ---
    uint32_t counter_ = 0;
    
    // 周期実行されるコールバック (ecatthread相当)
    void cyclic_task();

    // PI制御によるDC同期 (ec_sync相当)
    void ec_sync(int64_t reftime, int64_t cycletime, int64_t *offsettime);

    // EtherCATネットワークの起動 (ecatbringup相当)
    bool ecat_bringup();

    // スレーブのエラーチェック (ecatcheck相当の簡易版、または別スレッドで実行)
    void ecat_check_state();
};