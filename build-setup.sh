#!/bin/bash

# Setup build environment

chmod 755 Tools/scripts/install-prereqs-ubuntu.sh
chmod 755 Tools/firmware-build/firmware-build.sh
Tools/scripts/install-prereqs-ubuntu.sh
echo ""
echo "DONE!"

ASSUME_YES=false
function maybe_prompt_user() {
    if $ASSUME_YES; then
        return 0
    else
        read -p "$1"
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            return 0
        else
            return 1
        fi
    fi
}

echo ""

if maybe_prompt_user "Install tools for building binaries for 64-bit linux autopilots (i.e. Navio2/Raspberry Pi4B) [N/y]?" ; then
    echo ""
    echo "installing tools for 64-bit linux-based autopilots....."
    if [ ! -d /opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64 ]; then
        sudo git clone --depth 1 https://github.com/raspberrypi/tools.git /opt/tools
    else
        echo "nothing to do - already installed........"
    fi

    exportline3="export PATH=/opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:\$PATH";
    grep -Fxq "$exportline3" ~/.profile 2>/dev/null || {
        if maybe_prompt_user "Add /opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin to your PATH [N/y]?" ; then
            echo $exportline3 >> ~/.profile
            eval $exportline3
        else
            echo "Skipping adding /opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin to PATH."
        fi
}
else
    echo ""
    echo "Skipping 64-bit linux autopilot tools install........"
fi

echo ""
echo "creating desktop script for building HeliPilot......."
if [ ! -e ~/Desktop/HeliPilot-Firmware ]; then
    cp Tools/firmware-build/firmware-build.sh ~/Desktop/ && mv ~/Desktop/firmware-build.sh ~/Desktop/HeliPilot-Firmware
else
    echo ""
    echo "nothing to do - file exists!"
fi

echo ""
echo "DONE!"
echo ""
echo ">>> PLEASE LOG OUT AND LOG BACK IN TO REGISTER the PATH BEFORE CONTINUING <<<"
echo ""

