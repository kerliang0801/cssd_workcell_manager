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


#include "rmf_msgs/msg/dispenser_request.hpp"
#include "rmf_msgs/msg/dispenser_result.hpp"
#include "rmf_msgs/msg/dispenser_request_item.hpp"
#include "rmf_msgs/msg/inventory_check_request.hpp"
#include "rmf_msgs/msg/inventory_check_response.hpp"
#include "rmf_msgs/msg/dispenser_state.hpp"
#include "xbee_interface/srv/r2_r.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

struct requests
{
	std::string request_id;
	std::vector<int32_t> aruco_id;
	std::vector<std::string> item_type;
};


// TODO: change to struct
struct sub_workcell
{
  //dispenser_mode: (0,idle), (1,busy), (2,offline)
	std::string name;
	int32_t dispenser_mode;
	std::string ongoing_compartment_id;
	//The compartment the subworkcell is dropping to now. Used in checking that the dropoff is actually made in subworkcell respond callback
	std::vector<std::string> item_carried_by_subworkcell;
	std::vector<requests> queue;
};



class CssdWorkcellManager : public rclcpp::Node
{
public:
  CssdWorkcellManager(int no_of_workcell);
  void task_execution_thread();

private:
		
  rclcpp::Subscription<rmf_msgs::msg::InventoryCheckRequest>::SharedPtr CheckInventory_;
  rclcpp::Subscription<rmf_msgs::msg::DispenserRequest>::SharedPtr DispenserRequest_;
  rclcpp::Subscription<rmf_msgs::msg::DispenserResult>::SharedPtr SubWorkcellRespond_;  
  rclcpp::Subscription<rmf_msgs::msg::DispenserState>::SharedPtr SubWorkcellState_;    
  
  rclcpp::Publisher<rmf_msgs::msg::InventoryCheckResponse>::SharedPtr InventoryCheckResponse_;
  rclcpp::Publisher<rmf_msgs::msg::DispenserRequest>::SharedPtr SubWorkcellRequest_;
  rclcpp::Publisher<rmf_msgs::msg::DispenserResult>::SharedPtr DispenserResponse_;
  
  rclcpp::Client<xbee_interface::srv::R2R>::SharedPtr R2R_client_;
  
  rclcpp::callback_group::CallbackGroup::SharedPtr SubWorkcell_callback_group_;
  rclcpp::callback_group::CallbackGroup::SharedPtr RFM_callback_group_;
  rclcpp::callback_group::CallbackGroup::SharedPtr R2R_group_;

  //mysqlconn param
  sql::Driver *driver;
	sql::Connection *con;
	sql::Statement *stmt;
	sql::ResultSet *res;
  sql::PreparedStatement *pstmt;

  //ros param
  // TODO: change var name
	std::string dispenser_name_;
	std::string ip_address;
	std::string username;
	std::string password;
	std::string database_name;
	std::string R2R_server_name;
	float R2R_docking_distance_threshold;
	int max_request_size;

	//flag
	bool new_request = false;
	bool R2R_response = false;

	//variables
	std::vector<sub_workcell> subworkcell;
	std::string request_id;
	std::string transporter_id; 
	std::vector<bool> current_trolley_compartment_status;
	std::vector<std::string> trolley_compartment_id;
  std::vector<bool> planned_trolley_compartment_status;
	

  void inventory_check_callback(const rmf_msgs::msg::InventoryCheckRequest::SharedPtr msg);
  /*check DB inventory when OTUI request for a set of item. Will make sure that the requested item is below the limit set in yaml.
  Will sort through the requested item and add to the queue of the relevant subworkcell
  */
	
	void dispenser_request_callback(const rmf_msgs::msg::DispenserRequest::SharedPtr msg);
  /*callback for RFM dispenser request. Will query R2R service, checking that the trolley is closeby and save the status and id of compartment.
  Will check that the trolley has enough space. Save the request_id to the cssd_wm class and set "new_request" to true, 
  allowing cssd:main() to start publishing to subworkcell, according to the request id.
  */

  void SubWorkcell_respond_callback(const rmf_msgs::msg::DispenserResult::SharedPtr msg);
  /*update subworkcell status when subworkcell send response. It contain an R2R query to make sure that the compartment the arm placed has an item on it. 
	This is to check that the item is not dropped along the way. If not successfully loaded, the status will turn 2 and the cssd:main will handle it.
	*/
  void SubWorkcell_state_callback(const rmf_msgs::msg::DispenserState::SharedPtr msg);
  // periodically update subworkcell status when subworkcell send periodic status updates.

  void failed_loading_handling(std::string request_id);
  /*handle subworkcell failure. called in when error occur such as when the status of any subworkcell is 2.
  Will release all leftover item in the request_id.
	*/
  bool R2R_query(std::string device_id);
  //query to get R2R info
  
  void pub_dispenser_task_result(bool success_result);
  //send the result accordingly to the result arg.

};


#endif

// for (std::vector<std::string>::iterator it = inventory.begin() ; it != inventory.end(); ++it)
// {
  
//   RCLCPP_INFO(this->get_logger(), *it);
// }