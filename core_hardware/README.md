# core_hardware

## eepromへの書き込み方

```
sudo ./vendor/soem/bin/eepromtool ifname 1 -w ./teensy41/upper/soes/soes-esi/eeprom.bin
```

## testコード

build
```
cd test
./build.sh
```

exec
```
sudo ./build/ecat_zero_check ifname
```