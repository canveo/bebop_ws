## COMANDOS PARROT-SPHINX

### Arranque del servidor


```console
$ sudo systemctl start firmwared.service

$ sudo firmwared
```

### Lanzador del drone virtual (bebop2 power)
#### Se elimína la opsción de la cámara para optimizar recursos. Adicional se verifica el nombre del dispositivo de red con el comando 'iwconfig' y editarlo en el archivo  '/opt/parrot-sphinx/usr/share/sphinx/drones/bebop.drone' de ser necesario

```console
$ sphinx /opt/parrot-sudo sphinx/usr/share/sphinx/drones/bebop2.drone::with_front_cam=false
```
### Inicio del nodo bebop_autonomy

```console
$ roslaunch bebop_driver bebop_sphinx.launch 
```
+ el archivo bebop_sphinx.launch fue modificado desde la dirección /home/carlos/bebop_ws/src/bebop_autonomy/bebop_driver/launch, para el cual solo se modificó el argumento de la dirección ip utilizado por el drone virtual

```xml
 <arg name="ip" default="10.202.0.1" /> 
```
```xml
<?xml version="1.0"?>
<launch>
    <arg name="namespace" default="bebop" />
    <!-- arg name="ip" default="192.168.42.1" /-->
    <arg name="ip" default="10.202.0.1" /> <!-- This is for the sphinx drone -->
    <arg name="drone_type" default="bebop1" /> <!-- available drone types: bebop1, bebop2 -->
    <arg name="config_file" default="$(find bebop_driver)/config/defaults.yaml" />
    <arg name="camera_info_url" default="package://bebop_driver/data/$(arg drone_type)_camera_calib.yaml" />
    <group ns="$(arg namespace)">
        <node pkg="bebop_driver" name="bebop_driver" type="bebop_driver_node" output="screen">
            <param name="camera_info_url" value="$(arg camera_info_url)" />
            <param name="bebop_ip" value="$(arg ip)" />
            <rosparam command="load" file="$(arg config_file)" />
        </node>
        <include file="$(find bebop_description)/launch/description.launch" />
    </group>
</launch>

```



