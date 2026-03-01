#include "ecat_app.h"

#include <stdio.h>
#include "ecat_slv.h"
#include "esc_irq.h"
#include "pdo_override.h"
#include <Arduino.h>

/* Application variables */
_Objects    Obj;

void ecatapp_setUint8(const uint8_t id, const uint8_t *data, const uint8_t len){
    
}

void ecatapp_setFloat(const uint8_t id, const float *data, const uint8_t len){
    ecatapp_setUint8(id, (uint8_t*)data, len*sizeof(float));
}

bool irq_flag = false;
bool sync0_flag = false;

#define ESC_updateALevent() ESC_read (ESCREG_LOCALTIME, (void *) &ESCvar.Time, sizeof (ESCvar.Time));

// for DIG_process callback
void cb_get_inputs(){

}

void cb_set_outputs(){

}

// Handle ESC interrupt
void irq_interrupt_callback(void){
    irq_flag = true;
    // Serial.println("irq interrupt");
}

// DC SYNC0 interrupt
void sync0_interrupt_callback(void){
    sync0_flag = true;
    Serial.println("sync0 interrupt");
}

uint16_t check_dc_handler (void){
    // minimum watchdog value is 1 ms, in nanoseconds
    #define MIN_WATCHDOG_VALUE_NS      1000000

    /* Indicate we run DC */
    ESCvar.dcsync = 1;
    /* Fetch the sync counter limit (SDO10F1) */
    ESCvar.synccounterlimit = Obj.Error_Settings.SyncErrorCounterLimit;

    uint32_t sync0cycleTime = ESC_enable_DC();
    Obj.Sync_Manager_2_Parameters.CycleTime = sync0cycleTime;
    Serial.print("Sync0 Cycle Time: ");
    Serial.println(sync0cycleTime);
    // Obj.Sync_Manager_3_Parameters.CycleTime = sync0cycleTime;
    // calculate watchdog value as 2 x SYNC0 cycle time
    int watchdog_value = 2 * sync0cycleTime;
    if (watchdog_value < MIN_WATCHDOG_VALUE_NS) {
        watchdog_value = MIN_WATCHDOG_VALUE_NS;
    }
    APP_setwatchdog(watchdog_value);
    Serial.println("enable DC");

    return 0;
}

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
        .esc_hw_interrupt_enable = ESC_interrupt_enable,//ESC_interrupt_enable
        .esc_hw_interrupt_disable = ESC_interrupt_disable,//ESC_interrupt_disable
        .esc_hw_eep_handler = NULL,
        .esc_check_dc_handler = NULL,//check_dc_handler
    };

    delay(1000);
    Serial.println("Hello Main");
    ecat_slv_init(&config);
}


void ecatapp_loop(){
    // Simple
    ecat_slv();
    return;
     // stack in mixed mode
    if (sync0_flag) {
        ESC_updateALevent();        
        DIG_process (DIG_PROCESS_APP_HOOK_FLAG | DIG_PROCESS_INPUTS_FLAG);
        sync0_flag = false;
    }
    if (irq_flag) {
        ESC_updateALevent();
        if (ESCvar.dcsync) {
            DIG_process (DIG_PROCESS_OUTPUTS_FLAG);    
        } else {
            DIG_process (DIG_PROCESS_OUTPUTS_FLAG | DIG_PROCESS_APP_HOOK_FLAG | DIG_PROCESS_INPUTS_FLAG);
        }
        irq_flag = false;
    } else {
        // ecat_slv_worker(ESCREG_ALEVENT_CONTROL | ESCREG_ALEVENT_SMCHANGE
        //                 | ESCREG_ALEVENT_SM0 | ESCREG_ALEVENT_SM1);
        ecat_slv_poll();
        DIG_process(DIG_PROCESS_WD_FLAG);
    }
}