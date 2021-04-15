#!/bin/bash

# Any failure should terminate this script
set -e

# download version 6.2.0 (change the following three variables to change versions)
CUBE_VERSION="6.2.0"
CUBE_ZIP_FILENAME="en.stm32cubemx-lin_v6-2-0.zip"

# this link can be found here: https://www.st.com/en/development-tools/stm32cubemx.html
# use the guest download feature (i.e do NOT log in). ST will send out an email that has
# a link that doesn't need to be authenticated
CUBE_LINK="https://www.st.com/content/ccc/resource/technical/software/sw_development_suite/group0/53/ea/a1/95/91/ee/48/d5/stm32cubemx-lin_v6-2-0/files/stm32cubemx-lin_v6-2-0.zip/jcr:content/translations/en.stm32cubemx-lin_v6-2-0.zip"

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
    AUTO_INSTALL_XML="
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
    echo -n $'#!/bin/bash \n\n ' > cuberunner.sh
    echo -n "pushd /opt/STM32CubeMX_$CUBE_VERSION && ./STM32CubeMX && popd" >> cuberunner.sh
    
    curl -O $CUBE_LINK
    unzip $CUBE_ZIP_FILENAME
    
    sudo java -jar ./SetupSTM32CubeMX-$CUBE_VERSION auto-install.xml
    sudo cp ./cuberunner.sh /opt/STM32CubeMX_$CUBE_VERSION/cuberunner.sh

    sudo chmod 777 -R /opt/STM32CubeMX_$CUBE_VERSION

    cd $CUBEMX_INSTALL_DIR

    sudo chmod +x cuberunner.sh
    sudo ln -sfn /opt/STM32CubeMX_$CUBE_VERSION/cuberunner.sh /usr/local/bin/STM32CubeMX 

    echo "================================================================"
    echo "Done Installing CubeMX to $CUBEMX_INSTALL_DIR"
    echo "NOTE: if you are upgrading versions, you can delete older versions in the /opt directory"
    echo "================================================================"
fi
