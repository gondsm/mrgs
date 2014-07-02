#!/bin/bash
sudo olsrd -i $MRGS_INTERFACE -d 0 > /dev/null && echo olsrd launched.