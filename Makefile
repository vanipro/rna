dev:
	docker run --rm --name ttc --net host --device=/dev/ttyUSB0 --device=/dev/ttyACM0 --device=/dev/ttyACM1 -it -v ./:/root -w /root/catkin_ws ttc

lev: 
	docker run --rm --name ttc --net host -it -v ./:/root -w /root/catkin_ws ttc

loc: 
	docker run --rm --name ttc -p 8765:8765 -it -v ./:/root -w /root/catkin_ws ttc

att:
	docker exec -it ttc bash

bld:
	docker build -t ttc .

