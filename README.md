# IMU project

## Dependencies
* C++'11
* serial

## Configure this project

### Instal serial
"choose /path/to/serial"

```
cd /path/to/serial
git clone https://github.com/wjwwood/serial.git
make
make test
make install
```

### Get IMU Project
Clone the project from CDISE bitbucket:

`git clone ssh://git@cdise-bitbucket.skoltech.ru:7999/mr/imu.git`

We need to disable the certificate because server certificate verifications fails.

`git remote remove origin`

Create a new repository, either at the group or private.

`git remote add origin ssh://git@cdise-bitbucket.skoltech.ru:7999/mr/imu.git`

For compiling on catkin, first, we create a symbolic link at the ROS workspace:
```
roscd && cd ../src
ln -n /path/to/serial /path/to/your/project
cd ..
catkin_make 
```

Make changes
```
git add <files/dirs changed>
git commit -m  "<commit message>"
git push -u origin master
```

## Launch files
* imu.launch: record imu data only
* imu_rs.launch: record imu plus realsense 435 data 

## todo 
* Add udev rules for imu data sctreaming device (currently PORT = "/dev/ttyACM0")
* Think what value should be used if "-" or "@" or " " is recieved from imu (currently 0.0)