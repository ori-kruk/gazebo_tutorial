# README

[gazebo classic install page ](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)

## models
[gazebo classic models](http://models.gazebosim.org/)

[gimbal](http://models.gazebosim.org/gimbal_small_2d/model.tar.gz)
[iris](http://models.gazebosim.org/iris_with_standoffs_demo/model.tar.gz)


```
docker run -it --rm \
--name gz11 \
--hostname gz11 \
--privileged \
-v /home/user/projects/gazebo_tutorial:/workspaces/gazebo_tutorial \
--env="DISPLAY=$DISPLAY"  \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="$XAUTHORITY:$XAUTHORITY" \
--env="XAUTHORITY=$XAUTHORITY" \
--net=host \
 --device=/dev/dri:/dev/dri \
vsc-gazebo_tutorial-9884397052b6ea301371252f8b3859bec438e431d2da4707c6c3a7784cc0997f \
/bin/bash
```

## Hardware Acceleration in Docker
[5 Ways to Speedup Gazebo Simulations](https://www.blackcoffeerobotics.com/blog/5-ways-to-speedup-gazebo-simulations)
### intel system

```bash
apt-get -y install libgl1-mesa-glx libgl1-mesa-dri
```

```bash
docker run -it --privileged --net=host \    
--name test_image_container \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
--volume=/tmp/.X11-unix:/tmp/.X11-unix \
--device=/dev/dri:/dev/dri \
test_image:latest
```


---

# pub msg

```bash
gz topic -p "/gazebo/default/iris_demo/gimbal_tilt_cmd"  "gazebo.msgs.GzString" -m 'data: "1.0"'
/gazebo/default/iris_demo/gimbal_tilt_cmd
```

---

## Resource 
- [5 Ways to Speedup Gazebo Simulations](https://www.blackcoffeerobotics.com/blog/5-ways-to-speedup-gazebo-simulations)