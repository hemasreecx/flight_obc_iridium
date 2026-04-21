#!/bin/bash


echo "Compiling C++ code..."
cd build
cmake ..
make
cd ..


PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1)

if [ -z "$PORT" ]; then
    echo "Warning: Pico serial port not found. Is it already in bootloader mode?"
else
    echo "Pico found on $PORT. Triggering 1200-baud hardware reset..."

    stty -f "$PORT" 1200


    sleep 2
fi


echo "Waiting for RPI-RP2 drive to mount..."


for i in {1..10}; do
    if [ -d "/Volumes/RPI-RP2" ]; then
        break
    fi
    sleep 1
done

if [ -d "/Volumes/RPI-RP2" ]; then
    echo "Drive found! Uploading firmware..."

    # Copy the compiled file
    cp build/obc_flight.uf2 /Volumes/RPI-RP2/

    echo "Upload Complete! Pico is rebooting to run your new code."
else
    echo "Error: RPI-RP2 drive did not mount. You may need to press the button manually this one last time."
    exit 1
fi