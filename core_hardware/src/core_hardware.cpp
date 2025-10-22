#include <core_hardware/core_hardware.hpp>
#include <chrono>

using namespace std::chrono_literals;

CoreHardware::CoreHardware()
    : rclcpp::Node("core_hardware"){
    // パラメータの宣言と取得
    this->declare_parameter("if_name", "eth0");
    this->declare_parameter("cycle_time_us", 1000); // デフォルト1000us = 1ms

    this->get_parameter("if_name", if_name_);
    int cycle_time_us;
    this->get_parameter("cycle_time_us", cycle_time_us);
    cycle_time_ns_ = (int64_t)cycle_time_us * 1000;

    // -----------------------------------------------------
    // 1. EtherCATネットワークの起動 (ecatbringup相当)
    if (!ecat_bringup()) {
        RCLCPP_FATAL(this->get_logger(), "EtherCAT network failed to start.");
        // ノードをシャットダウンするか、リカバリを試みる
        return; 
    }
    
    // -----------------------------------------------------
    // 2. 周期実行タイマーの設定 (ecatthread相当)
    // リアルタイム性が要求される場合は、このタイマーをRT Executorで実行する必要がある
    RCLCPP_INFO(this->get_logger(), "Starting cyclic task with %d us period.", cycle_time_us);
    
    // タイマー周期は設定サイクルタイムに合わせる
    auto period = std::chrono::nanoseconds(cycle_time_ns_);
    timer_ = this->create_wall_timer(
        period, 
        std::bind(&CoreHardware::cyclic_task, this)
    );
    
    // -----------------------------------------------------
    // 3. エラーチェックのスレッド起動（元のecatcheck相当）
    // ROS 2の仕組みで別ノードやサービスとして実行する方がROS的だが、ここでは簡単に別スレッドとして起動する
    // Note: ROS 2ノード内でのスレッド管理は注意が必要
    std::thread check_thread(&CoreHardware::ecat_check_state, this);
    check_thread.detach(); // デタッチしてノードの実行とは別に動かす
}

CoreHardware::~CoreHardware(){
    RCLCPP_INFO(this->get_logger(), "Shutting down EtherCAT master...");
    ecx_close(&ctx_);
}

// --- 周期制御タスク (ecatthread相当) ---
void CoreHardware::cyclic_task(){
    static int64_t toff = 0; // DC同期オフセット
    
    if (!mapping_done_) {
        // マッピング完了を待つか、エラーをログに出す
        return; 
    }

    if (dorun_) 
    {
        cycle_++;
        // データの受信
        wkc_ = ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);

        if (wkc_ != expectedWKC_) {
             dowkccheck_++;
        } else {
             dowkccheck_ = 0;
        }

        if (ctx_.slavelist[0].hasdc && (wkc_ > 0))
        {
            // DC同期のためのtoff計算
            ec_sync(ctx_.DCtime, cycle_time_ns_, &toff);
            // タイマーを使っているため、オフセット(toff)を直接タイマーに適用するのは難しい
            // ROS 2ではハードウェアタイマーやRTOSを使うことが多い
        }

        // メッセージング処理
        ecx_mbxhandler(&ctx_, 0, 4);

        // データの送信 (次のサイクルの出力)
        ecx_send_processdata(&ctx_);
    }
}

// --- PI制御によるDC同期 (ec_sync相当) ---
void CoreHardware::ec_sync(int64_t reftime, int64_t cycletime, int64_t *offsettime){
    static int64_t integral = 0;
    int64_t delta;
    
    delta = (reftime - sync_offset_ns_) % cycletime;
    if (delta > (cycletime / 2)) {
        delta = delta - cycletime;
    }
    
    // timeerrorはグローバル変数ではなくメンバー変数として扱う
    // timeerror_ = -delta; 
    
    integral += (-delta); // timeerrorの代わり
    *offsettime = (int64_t)(((-delta) * p_gain_) + (integral * i_gain_));
}

bool CoreHardware::ecat_bringup(){
    RCLCPP_INFO(this->get_logger(), "EtherCAT Startup on interface: %s", if_name_.c_str());

    // ecx_initの成功を確認
    if (!ecx_init(&ctx_, const_cast<char *>(if_name_.c_str())))
    {
        RCLCPP_FATAL(this->get_logger(), "ecx_init failed. Check interface name and permissions.");
        return false;
    }

    if (ctx_.slavecount > 0)
    {
        // ネットワーク構成の初期化とI/Oマッピング
        ecx_config_init(&ctx_);
        ec_groupt *group = &ctx_.grouplist[0];
        ecx_config_map_group(&ctx_, IOmap_, 0);

        expectedWKC_ = (group->outputsWKC * 2) + group->inputsWKC;
        RCLCPP_INFO(this->get_logger(), "%d slaves found and configured. Expected WKC: %d", ctx_.slavecount, expectedWKC_);

        // DC設定 (Distributed Clocks)
        mapping_done_ = true;
        ecx_configdc(&ctx_);

        // CoEスレーブをサイクリックMBXハンドラに追加
        for (int si = 1; si <= ctx_.slavecount; si++)
        {
            if (ctx_.slavelist[si].CoEdetails > 0)
            {
                ecx_slavembxcyclic(&ctx_, si);
                RCLCPP_INFO(this->get_logger(), "Slave %d added to cyclic mailbox handler", si);
            }
        }

        // --- OP状態への遷移 ---
        
        // ネットワークがクロックに同期するのを待つ (サンプルでは1秒)
        dorun_ = true;
        osal_usleep(1000000); 

        // 状態をOPに設定し、待機
        ctx_.slavelist[0].state = EC_STATE_OPERATIONAL;
        ecx_writestate(&ctx_, 0);
        ecx_statecheck(&ctx_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

        if (ctx_.slavelist[0].state != EC_STATE_OPERATIONAL)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach OPERATIONAL state.");
            // 失敗したスレーブの詳細を表示
            for (int si = 1; si <= ctx_.slavecount; si++)
            {
                ec_slavet *slave = &ctx_.slavelist[si];
                if (slave->state != EC_STATE_OPERATIONAL)
                {
                    RCLCPP_ERROR(this->get_logger(), "Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s",
                                 si, slave->state, slave->ALstatuscode, ec_ALstatuscode2string(slave->ALstatuscode));
                }
            }
            // 失敗時はクリーンアップして終了する方が安全だが、ここではfalseを返す
            dorun_ = false;
            return false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "EtherCAT OP state reached.");
            in_op_ = true;
            return true;
        }
    }
    else
    {
        RCLCPP_FATAL(this->get_logger(), "No slaves found.");
        return false;
    }
}

void CoreHardware::ecat_check_state(){
    // このスレッドはノードが実行されている間ずっとループする
    while (rclcpp::ok()) 
    {
        // OP状態で、WKCエラーが連続した場合、またはスレーブの状態チェックが必要な場合
        if (in_op_ && ((dowkccheck_ > 2) || ctx_.grouplist[current_group_].docheckstate))
        {
            ctx_.grouplist[current_group_].docheckstate = FALSE;
            ecx_readstate(&ctx_);

            for (int slaveix = 1; slaveix <= ctx_.slavecount; slaveix++)
            {
                ec_slavet *slave = &ctx_.slavelist[slaveix];

                if ((slave->group == current_group_) && (slave->state != EC_STATE_OPERATIONAL))
                {
                    ctx_.grouplist[current_group_].docheckstate = TRUE;
                    // ... (元のサンプルコードにあったエラー処理/状態遷移ロジック) ...
                    
                    if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        RCLCPP_WARN(this->get_logger(), "Slave %d in SAFE_OP + ERROR, attempting ack.", slaveix);
                        slave->state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ecx_writestate(&ctx_, slaveix);
                    }
                    else if (slave->state == EC_STATE_SAFE_OP)
                    {
                        RCLCPP_WARN(this->get_logger(), "Slave %d in SAFE_OP, changing to OPERATIONAL.", slaveix);
                        slave->state = EC_STATE_OPERATIONAL;
                        if (slave->mbxhandlerstate == ECT_MBXH_LOST) slave->mbxhandlerstate = ECT_MBXH_CYCLIC;
                        ecx_writestate(&ctx_, slaveix);
                    }
                    else if (slave->state > EC_STATE_NONE)
                    {
                        // 再構成のロジックなど...
                    }
                    else if (!slave->islost)
                    {
                        // slave lost のロジック...
                    }
                }
                
                // slave lostからの復帰ロジック
                if (slave->islost)
                {
                    // ... (元のサンプルコードにあったリカバリロジック) ...
                    if (slave->state <= EC_STATE_INIT)
                    {
                        if (ecx_recover_slave(&ctx_, slaveix, EC_TIMEOUTMON))
                        {
                            slave->islost = FALSE;
                            RCLCPP_INFO(this->get_logger(), "Slave %d recovered.", slaveix);
                        }
                    }
                    else
                    {
                        slave->islost = FALSE;
                        RCLCPP_INFO(this->get_logger(), "Slave %d found.", slaveix);
                    }
                }
            }

            if (!ctx_.grouplist[current_group_].docheckstate)
                RCLCPP_INFO(this->get_logger(), "OK : all slaves resumed OPERATIONAL.");
            
            dowkccheck_ = 0;
        }
        
        // 10ms待機
        osal_usleep(10000); 
    }
}

// --- メイン関数 ---
int main(int argc, char *argv[]){
    // ROS 2初期化
    rclcpp::init(argc, argv);
    
    // シングルスレッドExecutorでノードを実行 (RT対応の場合は設定変更が必要)
    rclcpp::spin(std::make_shared<CoreHardware>());
    
    // ROS 2終了処理
    rclcpp::shutdown();
    return 0;
}