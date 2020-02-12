#!/bin/bash

function draw_logo() {
    clear
    echo -e "\e[1;34m 0  0  0000  0     0 \e[0m\e[1;31m  0 0   0  0      000   0000000 \e[0m"
    echo -e "\e[1;34m 0  0  0     0     0 \e[0m\e[1;31m  0  0  0  0     0   0     0 \e[0m"
    echo -e "\e[1;34m 0000  000   0     0 \e[0m\e[1;31m  0 0   0  0    0     0    0 \e[0m"
    echo -e "\e[1;34m 0  0  0     0     0 \e[0m\e[1;31m  0     0  0     0   0     0 \e[0m"
    echo -e "\e[1;34m 0  0  0000  0000  0 \e[0m\e[1;31m  0     0  0000   000      0 \e[0m"
    echo -e "\e[1;34m----------------------\e[0m\e[1;31m--------------------------- \e[0m"
    echo "building HeliPilot firmware.................."
    echo ""
}

ASSUME_YES=false
function maybe_prompt_user() {
    if $ASSUME_YES; then
        return 1
    else
        read -p "$1"
        if [[ $REPLY =~ ^[Nn]$ ]]; then
            return 1
        else
            return 0
        fi
    fi
}

draw_logo
# check that HeliPilot repo exists
if [ ! -e ~/github/HeliPilot/Tools/firmware-build/firmware-build.sh ]; then
    if [ ! -e ~/.config/helipilot/helipilot_path.txt ]; then
        echo ""
        echo -e "\e[1;31m The HeliPilot code repo was not found........... \e[0m"
        echo ""
        echo "Please run HeliPilot Setup to fix this. You can download the HeliPilot Setup"
        echo "program by downloading it from github at"
        echo "https://github.com/MidwestAire/HeliPilot/releases/tag/v21-beta"
        echo ""
        read -p "Program will not run - press Ctl-C to exit" die
    else
        helipilot_path=$( cat ~/.config/helipilot/helipilot_path.txt )
    fi
else
    helipilot_path="github/HeliPilot"
fi

# Setup build environment - initialize variables
# environment variables
now=$(date +"%m-%d-%Y")

#  must have a github account and token to auto-upload to github
github_owner=MidwestAire
github_token=********************************************
github_tag=v21-beta

draw_logo
echo ""
echo "for GitHub release uploads please edit ~/.config/helipilot/firmware-build.sh"
echo "and enter your GitHub account owner and upload token details......."
echo ""
echo "the GitHub upload tag is set to $github_tag"
echo "if this is ok press Enter, otherwise enter a new tag such as v20.02, etc.."
read -p "GitHub Tag: " new_tag
    if [ -z "$new_tag" ] ; then
        echo "GitHub tag not changed"
    else
        github_tag=$new_tag
    fi
    
echo ""
if maybe_prompt_user "Upload this build to GitHub releases [n/Y]" ; then
    github_upload=true
else
    github_upload=false
fi

########################################################################
#  build-spectic variables
# set this to where you want the binaries to go during the build
binary_location=~/Desktop/Firmware

draw_logo
echo ""
echo "the HeliPilot binaries will go to $binary_location"
echo "if this is ok press Enter, otherwise type in a different location"
read -p "Binary Destination: " new_location
    if [ -z "$new_location" ] ; then
        echo "Destination not changed"
    else
        binary_location=$new_location
    fi
BuildDir=$binary_location-$now
mkdir -m 755 $BuildDir
echo "build directory" $BuildDir "is writeable................."

build_version=$github_tag
github_branch=master

echo ""
echo "build is set to build from $github_branch branch"
echo "if this is ok press Enter, otherwise type in a different branch (i.e. HeliPilot-v19 or HeliPilot-v20)"
read -p "Branch: " new_branch
    if [ -z "$new_branch" ] ; then
        echo "Build branch set to $github_branch"
    else
        github_branch=$new_branch
        echo "Build branch is set to $github_branch"
    fi

# set to what type of binaries to build, true or false

echo ""
if maybe_prompt_user "Build for NuttX [n/Y]?" ; then
    NuttX_build=true
else
    NuttX_build=false
fi

echo ""
if maybe_prompt_user "Build for Linux [n/Y]?" ; then
    Linux_build=true
else
    Linux_build=false
fi

echo ""
if maybe_prompt_user "Build for ChibiOS [n/Y]?" ; then
    ChibiOS_build=true
else
    ChibiOS_build=false
fi

echo "$NuttX_build"
echo "$Linux_build"
echo "$ChibiOS_build"

########################################################################
# end of user configs, do not edit below this line..

draw_logo
build=~/$helipilot_path/Tools/firmware-build
build_path=~/$helipilot_path
# make sure control files and github upload is executable
chmod 744 $build_path/Tools/firmware-build/controls/* && chmod 744 $build_path/Tools/firmware-build/github-upload

cd $build_path
    git checkout $github_branch  && rm -rf build
    git submodule update --init --recursive

###########   Build Targets for NuttX #################
if [ "$NuttX_build" == true ]; then
    control=NuttX-FMUv2
    source $build/controls/PX4-fmuv2 -v $build_version -d $BuildDir -c $control -r $build_path
    if [ "$github_upload" == true ]; then
        source $build/github-upload github_api_token=$github_token owner=$github_owner repo=$github_repo tag=$github_tag filename=$BuildDir/$version-$control.px4
    fi

    control=NuttX-FMUv3
    source $build/controls/PX4-fmuv3 -v $build_version -d $BuildDir -c $control -r $build_path
    if [ "$github_upload" == true ]; then
        source $build/github-upload github_api_token=$github_token owner=$github_owner repo=$github_repo tag=$github_tag filename=$BuildDir/$version-$control.px4
    fi

    control=NuttX-Pixhawk3_Pro
    source $build/controls/PX4-fmuv4Pro -v $build_version -d $BuildDir -c $control -r $build_path
    if [ "$github_upload" == true ]; then
        source $build/github-upload github_api_token=$github_token owner=$github_owner repo=$github_repo tag=$github_tag filename=$BuildDir/$version-$control.px4
    fi
fi

###########   Build Targets for Linux #################
if [ "$Linux_build" == true ]; then
    control=Linux64-Navio2
    source $build/controls/Linux64-Navio2 -v $build_version -d $BuildDir -c $control -r $build_path
    if [ "$github_upload" == true ]; then
        source $build/github-upload github_api_token=$github_token owner=$github_owner repo=$github_repo tag=$github_tag filename=$BuildDir/$version-$control
    fi

    control=Linux64-Ubuntu_Server
    source $build/controls/Linux64-Ubuntu_Server -v $build_version -d $BuildDir -c $control -r $build_path
    if [ "$github_upload" == true ]; then
        source $build/github-upload github_api_token=$github_token owner=$github_owner repo=$github_repo tag=$github_tag filename=$BuildDir/$version-$control
    fi
fi

############ Build Targets for ChibiOS #################
if [ "$ChibiOS_build" == true ]; then
    control=-ChibiOS-FMUv3
    source $build/controls/ChibiOS-fmuv3 -v $build_version -d $BuildDir -c $control -r $build_path
    if [ "$github_upload" == true ]; then
        source $build/github-upload github_api_token=$github_token owner=$github_owner repo=$github_repo tag=$github_tag filename=$BuildDir/$version-$control.bin
    fi

    control=ChibiOS-CUAVv5
    source $build/controls/ChibiOS-CUAVv5 -v $build_version -d $BuildDir -c $control -r $build_path
    if [ "$github_upload" == true ]; then
        source $build/github-upload github_api_token=$github_token owner=$github_owner repo=$github_repo tag=$github_tag filename=$BuildDir/$version-$control.bin
    fi

    control=ChibiOS-Pixhawk4
    source $build/controls/ChibiOS-Pixhawk4 -v $build_version -d $BuildDir -c $control -r $build_path
    if [ "$github_upload" == true ]; then
        source $build/github-upload github_api_token=$github_token owner=$github_owner repo=$github_repo tag=$github_tag filename=$BuildDir/$version-$control.bin
    fi
fi

# clean up build directory
cd $build_path && rm -rf build
echo "Build Directory Cleaned!"
echo "DONE!"
