HOST = r61
BASE = /home/user


up:
	echo "HOST_IP=$(hostname -I | awk '{print $1}')" > .env
	docker compose up -d 
	docker exec -it ttc bash

att:
	docker exec -it ttc bash

lev: 
	docker run --rm --name ttc --net host -it -v ./:/root -w /root/catkin_ws ttc

loc: 
	docker run --rm --name ttc -p 8765:8765 -it -v ./:/root -w /root/catkin_ws ttc

put:
	rsync -avz -e "ssh -o Compression=yes" --progress \
		--relative $(filter-out $@,$(MAKECMDGOALS)) $(HOST):$(BASE)/
%:
	@:
