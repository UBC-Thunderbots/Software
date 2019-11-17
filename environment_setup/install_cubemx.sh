#!/bin/bash

# Any failure should terminate this script
set -e

# download version 5.4.0 (change the following two variables to change versions)
CUBE_VERSION="5.4.0"
CUBE_LINK="https://www.st.com/content/ccc/resource/technical/software/sw_development_suite/group0/5f/d0/cf/79/10/fb/4e/7e/STM32CubeMX_v5-4-0/files/stm32cubemx_v5.4.0.zip/jcr:content/translations/en.stm32cubemx_v5.4.0.zip"

CUBEMX_TMP_DIR="/tmp/cubemx"
CUBEMX_INSTALL_DIR="/opt/STM32CubeMX_$CUBE_VERSION"

if [ -d "$CUBEMX_INSTALL_DIR" ]; then
    echo "================================================================"
    echo " CubeMX $CUBE_VERSION already installed "
    echo "================================================================"
else
    echo "================================================================"
    echo "Installing CubeMX $CUBE_VERSION"
    echo "================================================================"

    # setup a clean directory to install cubemx in
    rm -rf $CUBEMX_TMP_DIR
    mkdir -p $CUBEMX_TMP_DIR
    cd $CUBEMX_TMP_DIR

    # create auto install xml
    AUTO_INSTALL_XML="\
        <?xml version='1.0' encoding='UTF-8' standalone='no'?>
        <AutomatedInstallation langpack='eng'>
            <com.st.microxplorer.install.MXHTMLHelloPanel id='readme'/>
            <com.st.microxplorer.install.MXLicensePanel id='licence.panel'/>
            <com.st.microxplorer.install.MXTargetPanel id='target.panel'>
            <installpath>$CUBEMX_INSTALL_DIR</installpath>
            </com.st.microxplorer.install.MXTargetPanel>
            <com.st.microxplorer.install.MXShortcutPanel id='shortcut.panel'/>
            <com.st.microxplorer.install.MXInstallPanel id='install.panel'/>
            <com.st.microxplorer.install.MXFinishPanel id='finish.panel'/>
        </AutomatedInstallation>"
    
    echo -n $AUTO_INSTALL_XML > auto-install.xml

    # create script to cd and run STM32CubeMX
    STM32_SCRIPT="cd /opt/STM32CubeMX_$CUBE_VERSION && ./STM32CubeMX && cd -"

    echo -n $STM32_SCRIPT > cuberunner.sh
    
    curl -O $CUBE_LINK
    unzip en.stm32cubemx_v5.4.0.zip
    
    sudo java -jar ./SetupSTM32CubeMX-5.4.0.exe auto-install.xml
    sudo cp ./cuberunner.sh /opt/STM32CubeMX_$CUBE_VERSION/cuberunner.sh

    cd $CUBEMX_INSTALL_DIR

    sudo chmod +x cuberunner.sh
    sudo ln -sfn /opt/STM32CubeMX_$CUBE_VERSION/cuberunner.sh /usr/local/bin/STM32CubeMX 

    echo "================================================================"
    echo "Done Installing CubeMX"
    echo "================================================================"
fi
