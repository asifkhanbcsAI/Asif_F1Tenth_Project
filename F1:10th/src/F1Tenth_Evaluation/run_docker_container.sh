#!/bin/sh

container_name=f1tenth_gym_ros
create_container (){
        docker run --rm -it\
        --name ${container_name} \
        -h ${container_name} \
      	--env="DISPLAY"\
      	--env="QT_X11_NO_MITSHM=1" \
      	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="${XAUTHORITY}:/root/.Xauthority" \
        --privileged \
        --net=f1tenth_net \
 	      -v $PWD/../:/f1tenth_workshop/ \
        f1tenth_gym_ros
}

rm_container (){
	if [ "$(docker ps -aq -f name=${container_name})" ]
        then
		if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
		then
			docker stop ${container_name}
		fi
        	docker rm ${container_name}
        fi
}

if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
then
	echo "Container is Running. Starting new session."
	docker exec -it ${container_name} bash
else
	rm_container
	xhost + local:host
	create_container
	xhost - local:host
fi
