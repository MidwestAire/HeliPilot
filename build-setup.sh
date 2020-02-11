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

# install Ubuntu Launchers for Desktop Systems
echo ""
echo "creating Application Launcher for HeliPilot-Build......."
if [ ! -e ~/.local/bin/firmware-build.sh ]; then
    # we try to create the directory structure for WSL on Windows, since it doesn't exist by default
    mkdir -m 755 ~/.local/bin
    cp Tools/firmware-build/firmware-build.sh ~/.local/bin/firmware-build.sh
    chmod 755 ~/.local/bin/firmware-build.sh
    sudo mkdir /usr/share/icons/helipilot
    sudo chmod 755 /usr/share/icons/helipilot
    sudo cp -r Tools/firmware-build/icons/ /usr/share/icons/helipilot/
    # create Application Launcher
    touch ~/.local/share/applications/HeliPilot-Build.desktop
    echo "[Desktop Entry]" >> ~/.local/share/applications/HeliPilot-Build.desktop
    echo "Encoding=UTF-8" >> ~/.local/share/applications/HeliPilot-Build.desktop
    echo "Name=HeliPilot-Build" >> ~/.local/share/applications/HeliPilot-Build.desktop
    echo "Comment=HeliPilot Build Launcher" >> ~/.local/share/applications/HeliPilot-Build.desktop
    echo "Exec=gnome-terminal -e .local/bin/firmware-build.sh" >> ~/.local/share/applications/HeliPilot-Build.desktop
    echo "Icon=/usr/share/icons/helipilot/icons/systembuild.png" >> ~/.local/share/applications/HeliPilot-Build.desktop
    echo "Type=Application" >> ~/.local/share/applications/HeliPilot-Build.desktop
    chmod 755 ~/.local/share/applications/HeliPilot-Build.desktop
    echo ""
    echo "Application Launcher Installed........"
else
    echo ""
    echo "nothing to do - application launcher already exists!"
fi

echo ""
echo "Firmware Build application launcher for the Dock is installed......"
if maybe_prompt_user "Do you want a desktop launcher too [N/y]?" ; then
    echo ""
    echo "creating desktop icon for building HeliPilot......."
    if [ ! -e ~/Desktop/HeliPilot-Build.desktop ]; then
        # we try to create the directory structure for WSL on Windows, since it doesn't exist by default
        mkdir -m 755 ~/Desktop
        # create Application Launcher
        touch ~/Desktop/HeliPilot-Build.desktop
        echo "[Desktop Entry]" >> ~/Desktop/HeliPilot-Build.desktop
        echo "Encoding=UTF-8" >> ~/Desktop/HeliPilot-Build.desktop
        echo "Name=HeliPilot-Build" >> ~/Desktop/HeliPilot-Build.desktop
        echo "Comment=HeliPilot Build Launcher" >> ~/Desktop/HeliPilot-Build.desktop
        echo "Exec=gnome-terminal -e .local/bin/firmware-build.sh" >> ~/Desktop/HeliPilot-Build.desktop
        echo "Icon=/usr/share/icons/helipilot/icons/systembuild.png" >> ~/Desktop/HeliPilot-Build.desktop
        echo "Type=Application" >> ~/Desktop/HeliPilot-Build.desktop
        chmod 755 ~/Desktop/HeliPilot-Build.desktop
    else
        echo ""
        echo "nothing to do - application launcher already exists!"
    fi
else
    echo ""
    echo "Skipping installation of desktop Launcher.........."
fi

echo ""
echo "creating Application Launcher for the HeliPilot simulator......."
if [ ! -e ~/.local/bin/sim ]; then
    # we try to create the directory structure for WSL on Windows, since it doesn't exist by default
    mkdir -m 755 ~/.local/bin
    cp ./sim ~/.local/bin/sim
    chmod 755 ~/.local/bin/sim
    # create Application Launcher
    touch ~/.local/share/applications/HeliPilot-Simulator.desktop
    echo "[Desktop Entry]" >> ~/.local/share/applications/HeliPilot-Simulator.desktop
    echo "Encoding=UTF-8" >> ~/.local/share/applications/HeliPilot-Simulator.desktop
    echo "Name=HeliPilot-Simulator" >> ~/.local/share/applications/HeliPilot-Simulator.desktop
    echo "Comment=HeliPilot Simulator Launcher" >> ~/.local/share/applications/HeliPilot-Simulator.desktop
    echo "Exec=gnome-terminal -e .local/bin/sim" >> ~/.local/share/applications/HeliPilot-Simulator.desktop
    echo "Icon=/usr/share/icons/helipilot/icons/simulator.png" >> ~/.local/share/applications/HeliPilot-Simulator.desktop
    echo "Type=Application" >> ~/.local/share/applications/HeliPilot-Simulator.desktop
    chmod 755 ~/.local/share/applications/HeliPilot-Simulator.desktop
else
    echo ""
    echo "nothing to do - application launcher already exists!"
fi

echo ""
echo "Simulator application launcher for the Dock is installed......"
if maybe_prompt_user "Do you want a desktop launcher too [N/y]?" ; then
    echo ""
    echo "creating desktop icon for running the HeliPilot simulator......."
    if [ ! -e ~/Desktop/HeliPilot-Simulator.desktop ]; then
        # we try to create the directory structure for WSL on Windows, since it doesn't exist by default
        mkdir -m 755 ~/Desktop
        # create Application Launcher
        touch ~/Desktop/HeliPilot-Simulator.desktop
        echo "[Desktop Entry]" >> ~/Desktop/HeliPilot-Simulator.desktop
        echo "Encoding=UTF-8" >> ~/Desktop/HeliPilot-Simulator.desktop
        echo "Name=HeliPilot-Simulator" >> ~/Desktop/HeliPilot-Simulator.desktop
        echo "Comment=HeliPilot Simulator Launcher" >> ~/Desktop/HeliPilot-Simulator.desktop
        echo "Exec=gnome-terminal -e .local/bin/sim" >> ~/Desktop/HeliPilot-Simulator.desktop
        echo "Icon=/usr/share/icons/helipilot/icons/simulator.png" >> ~/Desktop/HeliPilot-Simulator.desktop
        echo "Type=Application" >> ~/Desktop/HeliPilot-Simulator.desktop
        chmod 755 ~/Desktop/HeliPilot-Simulator.desktop
    else
        echo ""
        echo "nothing to do - application launcher already exists!"
    fi
else
    echo ""
    echo "Skipping installation of desktop Launcher.........."
fi

. ~/.profile
echo ""
echo "DONE!"
echo ""
echo "Please log out, then log back in to register your application paths and launchers"

