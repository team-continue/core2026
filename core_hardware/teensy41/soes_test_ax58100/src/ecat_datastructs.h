#pragma once

#include <map>
#include <vector>
#include <stdint.h>

struct EcatData{
    uint8_t len_max;
    uint32_t addr;
};

std::map<uint8_t, EcatData> ecat_data;

void ecat_dataInit(uint8_t slave_id){
    if(slave_id == 1){
        ecat_data = std::map<uint8_t, EcatData>{
            std::pair<uint8_t, EcatData>{0, {0, 0}}
        };
    }else{
        
    }

    uint32_t len_sum = 0;
    for(auto &data: ecat_data){
        data.second.addr = len_sum;
        len_sum += data.second.len_max;
    }
}