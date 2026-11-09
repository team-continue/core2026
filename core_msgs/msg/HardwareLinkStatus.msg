# /hardware/link_status: PC とハードウェアの間の通信状態
std_msgs/Header header
bool daemon_connected       # core_hardware と core_hardware_daemon (IPC) が接続中か
bool ethercat_connected     # daemon と upper 基板 (EtherCAT) が通信中か。daemon 未接続なら false
float32 ethercat_rx_age_ms  # 最後に EtherCAT を受信してからの時間 [ms]。不明なら NaN
bool bottom_connected       # upper 基板と bottom 基板 (CAN) が通信中か。EtherCAT 未接続なら false
