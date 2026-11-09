# upper 基板の CAN バス 1 本分の状態
string name           # バス名 (can3 / can2)
bool valid            # 取得できているか (false なら以下の値は無効)
bool bus_off          # bus-off 状態か
bool error_passive    # error passive 状態か
uint8 tx_error_count  # 送信エラーカウンタ
uint8 rx_error_count  # 受信エラーカウンタ
