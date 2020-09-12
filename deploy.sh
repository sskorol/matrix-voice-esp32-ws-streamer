#!/bin/bash
FIRMWARE=.pio/build/esp32dev/firmware.bin

showhelp () {
  echo "----------------------------------------------------"
  echo "OTA-base installer (for the first deploy or crashes)"
  echo "----------------------------------------------------"
  echo ""
  echo "usage:"
  echo "./deploy.sh [RaspberryPi IP]"
  echo ""
  echo "example:"
  echo "./deploy.sh 192.168.0.100"
  echo ""
} 

if [ "$1" = "" ]; then
  showhelp
else
  if test -f "$FIRMWARE"; then
    echo ""
    cp .pio/build/esp32dev/firmware.bin .
    cp .pio/build/esp32dev/partitions.bin .
    cp ./ota/bootloader.bin .
    echo "Loading firmware: $FIRMWARE"
    echo ""
    tar cf - *.bin | ssh pi@$1 'tar xf - -C /tmp;sudo voice_esp32_reset;voice_esptool --chip esp32 --port /dev/ttyS0 --baud 1500000 --before default_reset --after hard_reset write_flash -u --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 /tmp/bootloader.bin 0x10000 /tmp/firmware.bin 0x8000 /tmp/partitions.bin'
    echo "done"
    echo ""
    echo "[SUCCESS] Please disconnect your MatrixVoice from the RaspberryPi and reconnect it alone for future OTA updates."
    echo ""
  else
    echo "[ERROR] Please build firmware first!"
    exit 1
  fi

fi

exit 0
