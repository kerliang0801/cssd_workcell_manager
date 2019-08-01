# cssd_workcell_manager (cssd_wm)

## acronym
RAWM = Robot Arm Workcell Manager

## dependency
1. mysql server and cpp connect
```
sudo apt-get install mysql-server
sudo apt-get install libmysqlcppconn-dev=1.1.9-1
```
## creating database
1. create database
```
CREATE DATABASE inventory
```
2. select database
```
USE inventory
```
3. Create table
```
CREATE TABLE workcell (aruco_id INT, item VARCHAR(30) );
```
4. Populate table
```
cd cssd_workcell_manager
./db_repopulation
```
