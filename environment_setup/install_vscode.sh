#!/bin/bash

echo "================================================================"
echo "Installing VSCode"
echo "================================================================"

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

if [ -d "/opt/VSCode-linux-x64" ] 
then
    echo "Old VSCode installation detected, please delete /opt/VSCode-linux-x64 and re-run" 
    exit 1
fi

# permalink obtained from here: https://github.com/microsoft/vscode/issues/1084
download_permalink_linux64=http://go.microsoft.com/fwlink/?LinkID=620884
vscode_executable_path="/usr/local/bin/vscode"

echo "Downloading Stable VSCode"
wget -O /tmp/vscode-stable.tar.gz $download_permalink_linux64

echo "Unzipping to /opt folder"
tar -xvf /tmp/vscode-stable.tar.gz -C /opt

echo "Creating Desktop Entry"

VSCODE_DESKTOP_ENTRY='''
[Desktop Entry]
Name=Visual Studio Code
Comment=Programming Text Editor
Exec=/opt/VSCode-linux-x64/code
Icon=/opt/VSCode-linux-x64/resources/app/resources/linux/code.png
Terminal=false
Type=Application
Categories=Programming;
'''
echo "$VSCODE_DESKTOP_ENTRY" > ~/.local/share/applications/vscode.desktop

echo "Symlink VSCode"
sudo ln -s -f /opt/VSCode-linux-x64/code ${vscode_executable_path}

echo "================================================================"
echo "Done"
echo "================================================================"
