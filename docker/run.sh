export image=mppi-ca/base
export container=mppi-ca-base

cuda_args=()
volumes_args=()
environment_args=()

volumes_args+=( "-v $HOME/.Xauthority:/home/user/.Xauthority:ro" )
environment_args+=( "-v /dev/bus/usb:/dev/bus/usb " )




path="/home/user/mppi-ca/src" 
volumes_args+=( "-v $(pwd)/..:${path}:rw" )


docker run -it --privileged --net=host \
    --name $container \
    --device-cgroup-rule='c 189:* rmw' \
    ${environment_args[@]} \
    ${volumes_args[@]} \
    $image zsh