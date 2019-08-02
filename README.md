# cssd_workcell_manager (cssd_wm)

## acronym
RAWM = Robot Arm Workcell Manager

## Installing Dependencies
1. mysql server and cpp connect
```
sudo apt-get install mysql-server
sudo apt-get install libmysqlcppconn-dev=1.1.9-1
```
2. [xbee_interfaces](https://github.com/RMFHOPE/xbee_interface)

3. [rmf_msgs](null)

## Setup The Environment
### Simple Mysql DB Setup
```
# 1. enter mysql application
sudo mysql -u root -p

# 2. Create new user for cppconn
CREATE USER '$USER'@'localhost' IDENTIFIED BY '$PASSWORD';

# 3. create database
CREATE DATABASE inventory;

# 4. select database
USE inventory;

# 5. Create table
CREATE TABLE workcell (aruco_id INT, item VARCHAR(30) );

# 6. Populate table
exit
ros2 cd cssd_workcell_manager
./db_repopulation
```

### Package Complilation
```
colcon build --symlink-install --packages-select cssd_workcell_manager
```

## Running the node
1. Make sure that the `parameters.yaml` file in params folder is changed to the appropriate parameter.

```
ros2 run cssd_workcell_manager cssd_workcell_manager ,number_of_workcell>  __params:=/home/malcomneo/ros2_ws/src/CSSD_WM/cssd_workcell_manager/params/parameters.yaml
```
