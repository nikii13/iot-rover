#!/bin/sh
set -eu

printf '%s\n' 'Installing dependencies...'
sudo apt update
sudo apt install -y liblgpio-dev g++ make

printf '%s\n' 'Building encoder server...'
make clean
make

printf '%s\n' 'Installing/restarting encoderd...'
sudo make install

printf '%s\n' 'Service status:'
systemctl --no-pager --full status encoderd || true

printf '%s\n' ''
printf '%s\n' 'Socket:'
ls -l /tmp/encoder.sock

printf '%s\n' ''
printf '%s\n' 'Done. Lab programs are in ./lab'
