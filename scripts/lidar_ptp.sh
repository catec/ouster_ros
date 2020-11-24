#!/usr/bin/env bash

sudo systemctl daemon-reload
sudo systemctl restart ptp4l
sudo systemctl restart phc2sys
sudo systemctl restart phc2shm

# sudo apt install linuxptp chrony ethtool

# Modify /etc/linuxptp/ptp4l.conf with:
#  clockClass 128
#  boundary_clock_jbod 1
#  [eno1] <- append this, being eno1 the lidar network interface

# Create folder: sudo mkdir -p /etc/systemd/system/ptp4l.service.d
# Create /etc/systemd/system/ptp4l.service.d/override.conf with:
#  [Service]
#  ExecStart=
#  ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf

# Create folder: sudo mkdir -p /etc/systemd/system/phc2sys.service.d
# Create /etc/systemd/system/phc2sys.service.d/override.conf with:
#  [Service]
#  ExecStart=
#  ExecStart=/usr/sbin/phc2sys -w -s CLOCK_REALTIME -c eno1

# Modify /etc/systemd/system/phc2shm.service

#  [Unit]
#  Description=Synchronize PTP hardware clock (PHC) to NTP SHM
#  Documentation=man:phc2sys
#  After=ntpdate.service
#  Requires=ptp4l.service
#  After=ptp4l.service
#  [Service]
#  Type=simple
#  ExecStart=/usr/sbin/phc2sys -s eno1 -E ntpshm -w 
#  [Install]
#  WantedBy=multi-user.target
