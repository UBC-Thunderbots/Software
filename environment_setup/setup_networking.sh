#!/bin/bash
# At the time of creating this script, we have team members with enp3s0f1,
# wlp3s0, wlp7s0, eth1, eth0 and a plethora of other network interfaces 
#
# This script will create two virtual interfaces tbots_wifi and tbots_eth
# and then bridge those with the appropriate physical interfaces on the users
# system.
#
# This script can be rerun to reconfigure the interfaces.
echo "================================================================"
echo " Networking Interface Setup"
echo "================================================================"

TBOTS_WIFI_INTERFACE=tbots_wifi
TBOTS_ETHERNET_INTERFACE=tbots_eth

echo "Removing tbots network interfaces, if they exist"
sudo ip link set dev $TBOTS_WIFI_INTERFACE down
sudo ip link set dev $TBOTS_ETHERNET_INTERFACE down
sudo brctl delbr $TBOTS_WIFI_INTERFACE
sudo brctl delbr $TBOTS_ETHERNET_INTERFACE

# get the "suggested" interface(s). The first interface is usually the correct
# one but we provide the option to select other interfaces, which is useful for VM users
SUGGESTED_INTERFACE=$(route | grep '^default' | grep -o '[^ ]*$')

PS3="Suggested interfaces(s):
$SUGGESTED_INTERFACE
Select wifi interface (input 0 to skip): "

echo "==================================================="
echo " Configure WiFi interface ($TBOTS_WIFI_INTERFACE)"
echo "==================================================="

select INTERFACE in $(ls /sys/class/net);
do
    case $INTERFACE in
        "")
            echo "WiFi interface not configured"
            break
            ;;
        *)
            echo "> Selected interface $INTERFACE ($REPLY)"
            sudo brctl addbr $TBOTS_WIFI_INTERFACE && \
            sudo brctl addif $TBOTS_WIFI_INTERFACE $INTERFACE && \
            sudo ip link set dev $TBOTS_WIFI_INTERFACE up && \
            echo "> WiFi Interface Configured"
            break
            ;;
    esac
done

PS3="Select ethernet interface (input 0 to skip): "

echo "==================================================="
echo " Configure Ethernet interface ($TBOTS_ETHERNET_INTERFACE)"
echo "==================================================="
select INTERFACE in $(ls -I $TBOTS_WIFI_INTERFACE /sys/class/net );
do
    case $INTERFACE in
        "")
            echo "Ethernet interface not configured"
            break
            ;;
        *)
            echo "> Selected interface $INTERFACE ($REPLY)"
            sudo brctl addbr $TBOTS_ETHERNET_INTERFACE && \
            sudo brctl addif $TBOTS_ETHERNET_INTERFACE $INTERFACE && \
            sudo ip link set dev $TBOTS_ETHERNET_INTERFACE up && \
            echo "> Ethernet Interface Configured"
            break
            ;;
    esac
done
