# モータ 1 台分の状態。Teensy 層で取れた事実をそのまま出し、判定 (閾値) は health_monitor が行う
uint8 id              # PDO 上のモータ ID (0〜14)
bool supported        # 診断対象か (false なら以下の値は無効)
bool connected        # 通信が生きているか
bool enabled          # 出力が有効か
bool fault            # 故障状態か
uint8 error_code      # ドライバ固有のエラーコード (生値)
float32 temp          # 主温度 [℃]。取得できない場合は NaN
float32 temp_rotor    # ローター温度 [℃] (Damiao のみ)。取得できない場合は NaN
float32 voltage       # 電圧 [V]。取得できない場合は NaN
