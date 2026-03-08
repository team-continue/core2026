#include <stdarg.h>
#include <Arduino.h>


#define DXL_USB_VER           20170915

#define CMD_PORT              Serial      // USB
// #define DBG_PORT              Serial2     // UART1
#define DXL_PORT              Serial7
uint32_t dxl_baud=              1000000;

#define DXL_LED_TX            13

#define DXL_TX_BUFFER_LENGTH  32768

static uint8_t ser2_rx_extra[DXL_TX_BUFFER_LENGTH];
static uint8_t ser2_tx_extra[DXL_TX_BUFFER_LENGTH];


uint8_t tx_buffer[DXL_TX_BUFFER_LENGTH];


static int rx_led_count = 0;
static int tx_led_count = 0;

static int rx_led_update_time;
static int tx_led_update_time;

static uint32_t update_time[8];


static uint32_t rx_data_cnt = 0;
static uint32_t tx_data_cnt = 0;

static uint32_t rx_bandwidth = 0;
static uint32_t tx_bandwidth = 0;

uint32_t usb_baud;

void update_dxl();
void update_led();


void setup()
{
  CMD_PORT.begin(115200);
  DXL_PORT.begin(dxl_baud);

  DXL_PORT.addMemoryForRead(ser2_rx_extra, sizeof(ser2_rx_extra));
  DXL_PORT.addMemoryForWrite(ser2_tx_extra, sizeof(ser2_tx_extra));

  pinMode( DXL_LED_TX, OUTPUT );

  digitalWrite(DXL_LED_TX, HIGH);
}

void loop()
{
  update_dxl();
  update_led();

  if( CMD_PORT.baud() != dxl_baud )
  {
    dxl_baud = CMD_PORT.baud();
    DXL_PORT.begin(CMD_PORT.baud());
  }

  if( (millis()-update_time[1]) > 1000 )
  {
    update_time[1] = millis();

    tx_bandwidth = tx_data_cnt;
    rx_bandwidth = rx_data_cnt;

    tx_data_cnt = 0;
    rx_data_cnt = 0;
  }
}


void update_dxl()
{
  int length;
  int i;


  //-- USB -> DXL
  length = CMD_PORT.available();
  if( length > 0 ){
    for(i=0; i<length; i++ ){
      DXL_PORT.write(CMD_PORT.read());
      DXL_PORT.flush();
    }

    tx_led_count = 3;

    tx_data_cnt += length;
  }

  //-- DXL -> USB
  length = DXL_PORT.available();
  if( length > 0 )
  {
    if( length > DXL_TX_BUFFER_LENGTH )
    {
      length = DXL_TX_BUFFER_LENGTH;
    }
    for(i=0; i<length; i++ )
    {
      tx_buffer[i] = DXL_PORT.read();
    }
    CMD_PORT.write(tx_buffer, length);

    rx_led_count = 3;
    rx_data_cnt += length;
  }
}


void update_led()
{
  if( (millis()-tx_led_update_time) > 50 )
  {
    tx_led_update_time = millis();

    if( tx_led_count )
    {
      digitalWrite(DXL_LED_TX, !digitalRead(DXL_LED_TX));
      tx_led_count--;
    }
    else
    {
      digitalWrite(DXL_LED_TX, HIGH);
    }
  }

  if( (millis()-rx_led_update_time) > 50 )
  {
    rx_led_update_time = millis();

    if( rx_led_count )
    {
      rx_led_count--;
    }
    else
    {
    }
  }
}