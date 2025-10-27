#!/bin/bash

IP=$1
PASS=$2
SSH_KEY=$(ls ~/.ssh/id_*.pub | head -n1)

ssh-keygen -R "$IP" 2>/dev/null
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=accept-new user@"$IP" exit
sshpass -p "$PASS" ssh-copy-id -i "$SSH_KEY" -o StrictHostKeyChecking=no user@"$IP"

if grep -q "Host ttc" ~/.ssh/config 2>/dev/null; then
    sed -i.tmp '/^Host ttc/,/^$$/d' ~/.ssh/config && rm -f ~/.ssh/config.tmp
fi

cat >> ~/.ssh/config << EOF
Host ttc
	User user
	HostName $IP
	SetEnv TERM=xterm-256color

EOF

scp setup.sh ttc:/home/user && ssh ttc "chmod +x setup.sh" && ssh ttc
