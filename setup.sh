#!/bin/bash
set -eu

mkdir -p catkin_ws/src

echo "Add Docker's official GPG key:"
sudo apt-get update
sudo apt-get install ca-certificates curl vim
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo "Add the repository to Apt sources:"
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get -y update

echo "Install Docker packages"
sudo apt-get -y install docker-ce docker-ce-cli containerd.io 

sudo systemctl enable --now docker

sudo usermod -aG docker $USER
newgrp docker

docker build -t ttc .

