#!/bin/bash
set -eu

sudo apt-get -y update && apt-get -y install ca-certificates curl vim git
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

git clone "https://$1@github.com/vanipro/rna" rna && mv rna/* . && sudo rm -rf rna

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get -y update && apt-get -y install docker-ce docker-ce-cli containerd.io

sudo systemctl enable --now docker
sudo usermod -aG docker $USER
newgrp docker

docker pull vanipro/rna:arm64
make up
