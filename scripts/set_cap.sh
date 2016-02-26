#! /bin/sh
sudo setcap cap_net_raw,cap_net_admin=eip $1
getcap $1