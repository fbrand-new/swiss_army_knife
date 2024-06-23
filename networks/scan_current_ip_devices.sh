#!/bin/bash

# The following line returns all the ip devices connected to the network to which you are connected with the specified interface. You need to provide an interface to make the script work.

if [ "$#" -ne 1 ]  
then
    echo "Please provide an interface. Run ifconfig to get your interface name"
    exit -1
else
    INTERFACE=$1
fi

sudo arp-scan --interface $INTERFACE --localnet
