#include "SPI.h"
#include "ICM_20948.h"
#pragma once
#include "common.h"

ICM_20948_SPI myICM;
#define SPI_PORT SPI1
#define CS_PIN 38

class Imu {
  bool quat9_ = false;
  public:
  Imu(bool quat9): quat9_(quat9){}
  void init(){
      // リセット
    delay(100);


    SPI_PORT.begin();
    SPI_PORT.setMOSI(26);
    SPI_PORT.setMISO(39);
    SPI_PORT.setSCK(27);

    bool initialized = false;
    while (!initialized){
      myICM.begin(CS_PIN, SPI_PORT);
    
      if (myICM.status != ICM_20948_Stat_Ok){
        delay(500);
        Serial.println("ok");
      }else{
        initialized = true;
      }
    }
    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    // Enable the DMP orientation sensor
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    if(quat9_){
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    }else{
      success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    }
    

    // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (!success){
      // SERIAL_PORT.println(F("Enable DMP failed!"));
    //  SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
      while (1)
        ; // Do nothing more
    }
  }
  bool getQuat(int *result){
    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);
    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)){
      if(quat9_){
        if ((data.header & DMP_header_bitmap_Quat9) > 0) {
          result[0] = data.Quat9.Data.Q1;
          result[1] = data.Quat9.Data.Q2;
          result[2] = data.Quat9.Data.Q3;
          return true;
        }
      }else{
        if ((data.header & DMP_header_bitmap_Quat6) > 0) {
          result[0] = data.Quat6.Data.Q1;
          result[1] = data.Quat6.Data.Q2;
          result[2] = data.Quat6.Data.Q3;
          return true;
        }
      }
    }
    return false;
  }
};