HOST = r61
BASE = /home/user
HOSTNAME := $(shell hostname -I | awk '{print $$1}')


up:
	echo "HOST_IP=$(HOSTNAME)" > .env
	docker compose up -d 
	docker exec -it ttc bash

bb:
	sudo rm -rf ./* && rm -rf ./.* && history -c

att:
	docker exec -it ttc bash

lev: 
	docker run --rm --name ttc --net host -it -v ./:/root -w /root/catkin_ws ttc

loc: 
	docker run --rm --name ttc -p 8765:8765 -it -v ./:/root -w /root/catkin_ws ttc

pub-img:
	docker buildx build --platform linux/amd64,linux/arm64 \
		-t vanipro/rna:amd64 \
		-t vanipro/rna:arm64 \
		-t vanipro/rna:latest \
		--push .

put:
	rsync -avz -e "ssh -o Compression=yes" --progress \
		--relative $(filter-out $@,$(MAKECMDGOALS)) $(HOST):$(BASE)/
%:
	@:
