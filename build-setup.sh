#!/bin/bash

# Setup build environment and tools to run SIM
chmod 755 Tools/scripts/install-prereqs-ubuntu.sh
chmod 755 Tools/firmware-build/firmware-build.sh

# run Ubuntu setup script
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
echo "creating desktop icon for building HeliPilot......."
if [ ! -e ~/.config/helipilot/firmware-build.sh ]; then
    # we try to create the directory structure for WSL on Windows, since it doesn't exist by default
    mkdir -m 755 ~/Desktop
    mkdir -m 755 ~/.config
    cp Tools/firmware-build/firmware-build.sh ~/.config/helipilot/firmware-build.sh
    chmod 755 ~/.config/helipilot/firmware-build.sh
    sudo mkdir /usr/share/icons/helipilot
    sudo cp -r Tools/firmware-build/icons/ /usr/share/icons/helipilot/
    touch ~/Desktop/Build-HeliPilot.desktop && chmod 755 ~/Desktop/Build-HeliPilot.desktop
    echo "[Desktop Entry]" >> ~/Desktop/Build-HeliPilot.desktop
    echo "Encoding=UTF-8" >> ~/Desktop/Build-HeliPilot.desktop
    echo "Name=Build-HeliPilot" >> ~/Desktop/Build-HeliPilot.desktop
    echo "Comment=HeliPilot Build Launcher" >> ~/Desktop/Build-HeliPilot.desktop
    echo "Exec=gnome-terminal -e .config/helipilot/firmware-build.sh" >> ~/Desktop/Build-HeliPilot.desktop
    echo "Icon=/usr/share/icons/helipilot/icons/systembuild.png" >> ~/Desktop/Build-HeliPilot.desktop
    echo "Type=Application" >> ~/Desktop/Build-HeliPilot.desktop
else
    echo ""
    echo "nothing to do - file exists!"
fi

echo ""
echo "creating desktop icon for running the HeliPilot simulator......."
if [ ! -e ~/.config/helipilot/sim ]; then
    # we try to create the directory structure for WSL on Windows, since it doesn't exist by default
    mkdir -m 755 ~/Desktop
    mkdir -m 755 ~/.config
    cp ./sim ~/.config/helipilot/sim && chmod 755 ~/.config/helipilot/sim
    touch ~/Desktop/Simulator.desktop && chmod 755 ~/Desktop/Simulator.desktop
    echo "[Desktop Entry]" >> ~/Desktop/Simulator.desktop
    echo "Encoding=UTF-8" >> ~/Desktop/Simulator.desktop
    echo "Name=Simulator" >> ~/Desktop/Simulator.desktop
    echo "Comment=HeliPilot Simulator Launcher" >> ~/Desktop/Simulator.desktop
    echo "Exec=gnome-terminal -e .config/helipilot/sim" >> ~/Desktop/Simulator.desktop
    echo "Icon=/usr/share/icons/helipilot/icons/simulator.png" >> ~/Desktop/Simulator.desktop
    echo "Type=Application" >> ~/Desktop/Simulator.desktop
else
    echo ""
    echo "nothing to do - file exists!"
fi

. ~/.profile
echo ""
echo "DONE!"

