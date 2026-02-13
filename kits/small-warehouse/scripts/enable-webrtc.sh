#!/bin/bash

sudo modprobe dummy 
sudo ip link add dummy0 type dummy
sudo ip addr add $(curl -s http://169.254.169.254/latest/meta-data/public-ipv4)/32 dev dummy0 
sudo ip link set dummy0 up

sudo ufw allow 8211/tcp
sudo ufw allow 9000:10000/udp
sudo ufw reload