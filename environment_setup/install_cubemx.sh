#!/bin/bash

echo "================================================================"
echo "Installing CubeMX V 5.4.0"
echo "================================================================"

# Any failure should terminate this script
set -e

CUBEMX_TMP_DIR="/tmp/cubemx"

# download vs v5.4.0
CUBE_LINK="https://www.st.com/content/ccc/resource/technical/software/sw_development_suite/group0/5f/d0/cf/79/10/fb/4e/7e/STM32CubeMX_v5-4-0/files/stm32cubemx_v5.4.0.zip/jcr:content/translations/en.stm32cubemx_v5.4.0.zip"

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
        <installpath>/opt/STM32CubeMX</installpath>
        </com.st.microxplorer.install.MXTargetPanel>
        <com.st.microxplorer.install.MXShortcutPanel id='shortcut.panel'/>
        <com.st.microxplorer.install.MXInstallPanel id='install.panel'/>
        <com.st.microxplorer.install.MXFinishPanel id='finish.panel'/>
    </AutomatedInstallation>"

echo -n $AUTO_INSTALL_XML > auto-install.xml

wget $CUBE_LINK
unzip en.stm32cubemx_v5.4.0.zip

sudo java -jar ./SetupSTM32CubeMX-5.4.0.exe auto-install.xml

echo "================================================================"
echo "Done Installing CubeMX"
echo "================================================================"
