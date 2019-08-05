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

### Package Complilation
```
colcon build --symlink-install --packages-select cssd_workcell_manager
```

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
cd ros2_ws/src/cssd_workcell_manager/db_repopulation
./db_repopulation
```

** For now username & password are 'malcomneo'. If user and password has been set differently, please change and rebuild the repopulation code as shown below.
```
# change the password
con = driver->connect("tcp://127.0.0.1:3306", "malcomneo", "malcomneo");

# compile
sudo g++ -Wall -I/usr/include/cppconn -o db_repopulation db_repopulate.cpp -L/usr/lib -lmysqlcppconn

```


## Running the node
1. Make sure that the `parameters.yaml` file in params folder is changed to the appropriate parameter (e.g password, inventory, etc mentioned above).

```
ros2 run cssd_workcell_manager cssd_workcell_manager <$NUM_OF_WORKCELL>  __params:={$ROS2_WS}/src/cssd_workcell_manager/cssd_workcell_manager/params/parameters.yaml
```

```
SAMPLE ROS2 TOPIC PUB
```


## What has changed
- change `main` in class to `task_execution_thread`
- change of topic names

## TODO
- Class member var name (e.g.`dispenser_name` to `dispenser_name_`)
- Change `sub_workcell` to a struct
- replace `RAWM` to a generic term `subworkcell`
- use `std::map` for `RAWM_pointer`, and change naming: `current_tasks_map_obj`
- tidy `task_execution_thread` function
- $NUM_OF_WORKCELL in rosparam
- add sample ros2 dispenser reqeuest pub in readme file
