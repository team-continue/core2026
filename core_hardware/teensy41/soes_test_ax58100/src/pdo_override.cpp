#include <string.h>
#include "spi_hw.h"

#include "esc.h"
#include "ecat_slv.h"
#include "utypes.h"

#include "pdo_override.h"

#define ESC_CMD_READ        0x02
#define ESC_CMD_READ_WAIT_STATE 0x03
#define ESC_CMD_WRITE       0x04

#define READ_END_BYTE       0xFF

#define ADDR_SIZE           2               // 2 bytes mode: addr < 3 | cmd
#define WAIT_SIZE           1               // Wait State byte, single
#define READ_END_SIZE     1               // Read end byte

#define et1100 1

#define PDO_RX_TR_SIZE (ADDR_SIZE + WAIT_SIZE + sizeof(Obj.test_value_rx))
#define PDO_TX_TR_SIZE (ADDR_SIZE + sizeof(Obj.test_value_tx))

#define PD0_BUFFER_SIZE (PDO_RX_TR_SIZE > PDO_TX_TR_SIZE ? PDO_RX_TR_SIZE : PDO_TX_TR_SIZE)
#define RX_PADDING_SIZE     (8 - ADDR_SIZE - WAIT_SIZE)
#define TX_PADDING_SIZE     (8 - ADDR_SIZE)   // COE_pdo(Un)pack wants 

// 送受信バッファ
uint8_t tx_buffer[TX_PADDING_SIZE + PD0_BUFFER_SIZE] __attribute__((aligned (8)));
uint8_t rx_buffer[RX_PADDING_SIZE + PD0_BUFFER_SIZE] __attribute__((aligned (8)));
uint8_t *rx_buffer_spi = &rx_buffer[RX_PADDING_SIZE];
uint8_t *rx_buffer_coe = &rx_buffer[RX_PADDING_SIZE + ADDR_SIZE + WAIT_SIZE];
uint8_t *tx_buffer_spi = &tx_buffer[TX_PADDING_SIZE];
uint8_t *tx_buffer_coe = &tx_buffer[TX_PADDING_SIZE + ADDR_SIZE];

// master to slave
void rxpdo_override(void){
    memset(tx_buffer_spi, 0, PDO_RX_TR_SIZE);
    memset(rx_buffer_spi, 0, PDO_RX_TR_SIZE);
    // 読み込みアドレス設定
    uint16_t address = ESC_SM2_sma;
    tx_buffer_spi[0] = address >> 5;
    tx_buffer_spi[1] = ((address & 0x1F) << 3) | ESC_CMD_READ_WAIT_STATE;
    tx_buffer_spi[ADDR_SIZE] = READ_END_BYTE; // wait state byte
    tx_buffer_spi[PDO_RX_TR_SIZE - 1] = READ_END_BYTE; // read end byte

    // SPI 通信
    spi_select(et1100);
    spi_bidirectionally_transfer(et1100, rx_buffer_spi, tx_buffer_spi, PDO_RX_TR_SIZE);
    spi_unselect(et1100);

    // AL Event レジスタの更新
    ESCvar.ALevent = etohs((uint16_t)*rx_buffer_spi);

    // 受信データをCOE_pdoUnpackで展開
    if (MAX_MAPPINGS_SM2 > 0) {
        COE_pdoUnpack(rx_buffer_coe, ESCvar.sm2mappings, SMmap2);
    }

    // loop back
    // static unsigned long prev_print_ms = millis();
    // if (millis() - prev_print_ms > 1000){
    //     prev_print_ms = millis();
    //     Serial.print("RXPDO received: ");
    //     Serial.println(Obj.test_value_rx);
    // }

    Obj.test_value_tx = Obj.test_value_rx;

    // Serial.print("RXPDO received: ");
}

// slave to master 
void txpdo_override(void){
    Obj.test_value_tx++;

    memset(tx_buffer_spi, 0, PDO_TX_TR_SIZE);
    // 書き込みアドレス設定
    uint16_t address = ESC_SM3_sma;
    tx_buffer_spi[0] = address >> 5;
    tx_buffer_spi[1] = ((address & 0x1F) << 3) | ESC_CMD_WRITE;

    // txbufにデータを詰める
    if (MAX_MAPPINGS_SM3 > 0){
        COE_pdoPack (tx_buffer_coe, ESCvar.sm3mappings, SMmap3);
    }

    // SPI 通信
    spi_select(et1100);
    spi_write(et1100, tx_buffer_spi, PDO_TX_TR_SIZE); /* send address + data */
    spi_unselect(et1100);
}