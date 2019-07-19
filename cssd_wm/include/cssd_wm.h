#ifndef CSSD_WM_H // include guard
#define CSSD_WM_H


#include <stdlib.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>

#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include <vector>
#include <string>
	

#include "cssd_msgs/msg/dispenser_request.hpp"
#include "cssd_msgs/msg/dispenser_request_item.hpp"
#include "cssd_msgs/msg/inventory_check_request.hpp"
#include "cssd_msgs/msg/inventory_check_response.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class cssd_wm : public rclcpp::Node
{
public:
  cssd_wm(int no_of_workcell);

private:

	struct requests
	{
		std::string request_id;
		std::string transporter_type;
		std::vector<uint32_t> aruco_id;
	};


	class workcell
	{
	public:
		std::string name;
		std::vector<std::string> item_carried;
		std::vector<requests> queue;
	};

  rclcpp::Subscription<cssd_msgs::msg::InventoryCheckRequest>::SharedPtr CheckInventory_;
  rclcpp::Subscription<cssd_msgs::msg::DispenserRequest>::SharedPtr DispenserRequest_;
  rclcpp::Publisher<cssd_msgs::msg::InventoryCheckResponse>::SharedPtr InventoryCheckResponse_;


  sql::Driver *driver;
	sql::Connection *con;
	sql::Statement *stmt;
	sql::ResultSet *res;
  sql::PreparedStatement *pstmt;

	std::vector<workcell> RAM;
	std::vector<std::string> test;


  void request_callback(const cssd_msgs::msg::DispenserRequest::SharedPtr msg);
  void inventory_check(const cssd_msgs::msg::InventoryCheckRequest::SharedPtr msg);

};


#endif

// for (std::vector<std::string>::iterator it = inventory.begin() ; it != inventory.end(); ++it)
// {
  
//   RCLCPP_INFO(this->get_logger(), *it);
// }