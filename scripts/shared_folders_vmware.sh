#!/bin/bash

# @Hugo Costelha - 2019
# Scrit to enable shared folders in a virtual machine

if ! grep -q vmhgfs "/etc/fstab"; then
  echo "Enabling shared folders. You need to reboot."
  echo ".host:/ /mnt/hgfs fuse.vmhgfs-fuse allow_other 0 0" >> /etc/fstab
else
  echo "Shared folders already enabled. Configure them if you have not done so already."
fi
