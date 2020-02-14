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

MACHINE_TYPE=$(uname -m)
if [ ${MACHINE_TYPE} == 'x86_64' ]; then
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
else
    echo ""
    echo "your processor architecture was detected as $MACHINE_TYPE"
    echo "skipping installation of 64-bit linux compilers........."
fi

# install Firmware Builder for Debian-based desktop systems
echo ""
echo "Installing the HeliPilot Firmware Builder and Dock Launcher....."
if [ -e ~/.local/bin/firmware-build.sh ]; then
    if maybe_prompt_user "Firmware Builder already installed. Upgrade to latest version [N/y]?" ; then
        cp Tools/firmware-build/firmware-build.sh ~/.local/bin/firmware-build.sh
        chmod 755 ~/.local/bin/firmware-build.sh
    else
        echo ""
        echo "Skipping Firmware Builder upgrade"
    fi
fi
    
if [ ! -e ~/.local/bin/firmware-build.sh ]; then
    mkdir -p ~/.local/bin
    sudo mkdir -p /usr/share/icons/helipilot
    sudo cp -r Tools/firmware-build/icons/ /usr/share/icons/helipilot/
    sudo chmod -R 755 /usr/share/icons/helipilot/icons
    sudo chmod 644 /usr/share/icons/helipilot/icons/*.png
    cp Tools/firmware-build/firmware-build.sh ~/.local/bin/firmware-build.sh
    chmod 755 ~/.local/bin/firmware-build.sh
fi

# create Application Launcher
if [ ! -e ~/.local/share/applications/HeliPilot-Build.desktop ]; then
    mkdir -p ~/.local/share/applications
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

# install Simulator for Debian-based desktop systems
echo ""
echo "Installing the HeliPilot Simulator and Dock Launcher....."
if [ -e ~/.local/bin/sim ]; then
    if maybe_prompt_user "Simulator already installed. Upgrade to latest version [N/y]?" ; then
        cp ./sim ~/.local/bin/sim
        chmod 755 ~/.local/bin/sim
    else
        echo ""
        echo "Skipping Simulator upgrade"
    fi
fi

if [ ! -e ~/.local/bin/sim ]; then
    cp ./sim ~/.local/bin/sim
    chmod 755 ~/.local/bin/sim
fi

# create Application Launcher
if [ ! -e ~/.local/share/applications/HeliPilot-Simulator.desktop ]; then
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

# install Desktop Suite Updater for Debian-based desktop systems
echo ""
echo "Installing the HeliPilot Desktop Suite Updater and Dock Launcher....."
if [ -e ~/.local/bin/HeliPilot-Update ]; then
    if maybe_prompt_user "Updater already installed. Upgrade to latest version [N/y]?" ; then
        cp Tools/HeliPilot-Updater/HeliPilot-Update ~/.local/bin/HeliPilot-Update
        chmod 755 ~/.local/bin/HeliPilot-Update
    else
        echo ""
        echo "Skipping Updater upgrade"
    fi
fi

if [ ! -e ~/.local/bin/HeliPilot-Update ]; then
    cp Tools/HeliPilot-Updater/HeliPilot-Update ~/.local/bin/HeliPilot-Update
    chmod 755 ~/.local/bin/HeliPilot-Update
    cp Tools/HeliPilot_desktop_version.txt ~/.config/helipilot/HeliPilot_desktop_version.txt
    chmod 644 ~/.config/helipilot/HeliPilot_desktop_version.txt
    sudo cp Tools/HeliPilot-Updater/update.png /usr/share/icons/helipilot/icons/update.png
    sudo chmod 644 /usr/share/icons/helipilot/icons/update.png
fi

# create Application Launcher
if [ ! -e ~/.local/share/applications/HeliPilot-Update.desktop ]; then
    touch ~/.local/share/applications/HeliPilot-Update.desktop
    echo "[Desktop Entry]" >> ~/.local/share/applications/HeliPilot-Update.desktop
    echo "Encoding=UTF-8" >> ~/.local/share/applications/HeliPilot-Update.desktop
    echo "Name=HeliPilot-Update" >> ~/.local/share/applications/HeliPilot-Update.desktop
    echo "Comment=HeliPilot Updater" >> ~/.local/share/applications/HeliPilot-Update.desktop
    echo "Exec=gnome-terminal -e .local/bin/HeliPilot-Update" >> ~/.local/share/applications/HeliPilot-Update.desktop
    echo "Icon=/usr/share/icons/helipilot/icons/update.png" >> ~/.local/share/applications/HeliPilot-Update.desktop
    echo "Type=Application" >> ~/.local/share/applications/HeliPilot-Update.desktop
    chmod 755 ~/.local/share/applications/HeliPilot-Update.desktop
else
    echo ""
    echo "nothing to do - application launcher already exists!"
fi

echo ""
echo "Simulator application launcher for the Dock is installed......"
if maybe_prompt_user "Do you want a desktop launcher too [N/y]?" ; then
    echo ""
    echo "creating desktop icon for running the HeliPilot updater......."
    if [ ! -e ~/Desktop/HeliPilot-Update.desktop ]; then
        # create Application Launcher
        touch ~/Desktop/HeliPilot-Update.desktop
        echo "[Desktop Entry]" >> ~/Desktop/HeliPilot-Update.desktop
        echo "Encoding=UTF-8" >> ~/Desktop/HeliPilot-Update.desktop
        echo "Name=HeliPilot-Update" >> ~/Desktop/HeliPilot-Update.desktop
        echo "Comment=HeliPilot Update Launcher" >> ~/Desktop/HeliPilot-Update.desktop
        echo "Exec=gnome-terminal -e .local/bin/HeliPilot-Update" >> ~/Desktop/HeliPilot-Update.desktop
        echo "Icon=/usr/share/icons/helipilot/icons/update.png" >> ~/Desktop/HeliPilot-Update.desktop
        echo "Type=Application" >> ~/Desktop/HeliPilot-Update.desktop
        chmod 755 ~/Desktop/HeliPilot-Update.desktop
    else
        echo ""
        echo "nothing to do - application launcher already exists!"
    fi
else
    echo ""
    echo "Skipping installation of desktop Launcher.........."
fi

# create Application Uninstaller
cp Tools/HeliPilot-uninstall ~/.local/bin/HeliPilot-uninstall
chmod 755 ~/.local/bin/HeliPilot-uninstall
if [ ! -e ~/.local/share/applications/HeliPilot-uninstall.desktop ]; then
    touch ~/.local/share/applications/HeliPilot-uninstall.desktop
    echo "[Desktop Entry]" >> ~/.local/share/applications/HeliPilot-uninstall.desktop
    echo "Encoding=UTF-8" >> ~/.local/share/applications/HeliPilot-uninstall.desktop
    echo "Name=HeliPilot-uninstall" >> ~/.local/share/applications/HeliPilot-uninstall.desktop
    echo "Comment=HeliPilot Uninstaller" >> ~/.local/share/applications/HeliPilot-uninstall.desktop
    echo "Exec=gnome-terminal -e .local/bin/HeliPilot-uninstall" >> ~/.local/share/applications/HeliPilot-uninstall.desktop
    echo "Icon=/usr/share/icons/helipilot/icons/uninstall.png" >> ~/.local/share/applications/HeliPilot-uninstall.desktop
    echo "Type=Application" >> ~/.local/share/applications/HeliPilot-uninstall.desktop
    chmod 755 ~/.local/share/applications/HeliPilot-uninstall.desktop
else
    echo ""
    echo "nothing to do - application uninstaller already exists!"
fi

# install HeliPilot-QGC
echo ""
echo "Installation of HeliPilot Desktop Suite is complete"
echo ""
if maybe_prompt_user "Do you you want to install the HeliPilot Ground Station [N/y]?" ; then
    echo ""
    mkdir tmp
    cd tmp
    wget https://github.com/MidwestAire/HeliPilot/releases/download/v20.02/HeliPilot-QGCv20_Linux.zip
    unzip HeliPilot-QGCv20_Linux.zip
    cd HeliPilot-QGCv20 && ./install.sh
    cd $root
    rm -rf tmp
else
    echo ""
    echo "Skipping Ground Station install.........."
fi

. ~/.profile
echo ""
echo "DONE!"
echo ""
echo "You should log out and log back in to register the paths for the program icons....."

