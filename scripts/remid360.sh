#!/bin/bash

mid360_ifname="enp3s0"
host_ip="192.168.1.5"
mid360_ip="192.168.1.190"

sudo nmcli connection add \
        con-name "mid360" \
        type ethernet \
        ifname $mid360_ifname \
        ipv4.address $host_ip/32 \
        ipv4.method manual \
        ipv4.routes $mid360_ip/32