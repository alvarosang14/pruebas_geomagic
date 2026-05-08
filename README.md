# pruebas_geomagic
Programa para probar el uso ihaptic de yarp -> device geomagic -> hd/hl phantom

Qt: sudo apt install qt6-base-dev qt6-tools-dev qt6-tools-dev-tools

# For Docker
xhost +local:root

# Ejecución de nodos

``` bash
yarpserver
yarpdev --device deviceBundler --attached_device geomagicdriver --wrapper_device hapticDevice_nws_ros2 --topic_name pepito
```

``` bash
ros2 run abb_egm_driver egm_driver --ros-args -p smooth_factor:=0.8
ros2 run geomagic_control haptic_to_cartesian --ros-args -p roll_N_sensor:=-1.57 -p yow_N_sensor:=-1.57
```

``` bash
ros2 run jr3_driver jr3_driver --ros-args -p channel:=/dev/ttyACM0
ros2 run geomagic_control cartesian_to_haptic
rqt
```
