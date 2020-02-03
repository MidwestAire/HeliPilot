#!/bin/bash

# Setup build environment - initialize variables
# environment variables
build=~/github/HeliPilot/Tools/firmware-build
now=$(date +"%m-%d-%Y")

#  must have a github account and token to auto-upload to github
github_owner=MidwestAire
github_token=**************************************

#######################################################
#  build-spectic variables
# set this to where you want the binaries to go during the build
BuildDir=~/Documents/HeliPilot-Firmware-$now
build_version=HPv21-beta

# set this to the path of the root of your github repos
path=~/github

# set this to the name of the github repo
github_repo=HeliPilot

# what branch are we building from
github_branch=master

# set the github tag and upload to true to upload to github releases
github_tag=v21-beta
github_upload=false

# set to what type of binaries to build, true or false
NuttX_build=true
Linux_build=false
ChibiOS_build=false

#######################################################

build_path=$path/$github_repo
# make sure control files and github upload is executable
chmod 744 $build_path/Tools/firmware-build/controls/* && chmod 744 $build_path/Tools/firmware-build/github-upload

cd $build_path
    git checkout $github_branch  && rm -rf build
    git submodule update --init --recursive
mkdir $BuildDir && chmod 755 $BuildDir
echo "build directory" $BuildDir "is writeable................."
echo "building on repository" $github_repo "branch" $github_branch

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
