#include "ecat_app.h"

#include <stdio.h>
#include "ecat_slv.h"
#include "esc_irq.h"
#include "pdo_override.h"
#include <Arduino.h>

/* Application variables */
_Objects    Obj;

void ecatapp_init(){
    
    static esc_cfg_t config = {
        .user_arg = "ax58100",
        .use_interrupt = 0,
        .watchdog_cnt = INT32_MAX,
        .set_defaults_hook = NULL,
        .pre_state_change_hook = NULL,
        .post_state_change_hook = NULL,
        .application_hook = NULL,
        .safeoutput_override = NULL,
        .pre_object_download_hook = NULL,
        .post_object_download_hook = NULL,
        .rxpdo_override = rxpdo_override,//rxpdo_override
        .txpdo_override = txpdo_override,//txpdo_override
        .esc_hw_interrupt_enable = NULL,//ESC_interrupt_enable
        .esc_hw_interrupt_disable = NULL,//ESC_interrupt_disable
        .esc_hw_eep_handler = NULL,
        .esc_check_dc_handler = NULL,
    };

    delay(1000);
    Serial.println("Hello Main");
    ecat_slv_init(&config);
}


void ecatapp_loop(){
    // polling ecat slave
    ecat_slv();

    // L Chika over Ether-CAT
    if ((Obj.test_value_rx % 200) < 100) {
        digitalWrite(LED_BUILTIN, LOW);   // 消灯
    } else {
        digitalWrite(LED_BUILTIN, HIGH);  // 点灯
    }
}