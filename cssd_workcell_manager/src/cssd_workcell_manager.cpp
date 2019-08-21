#include "cssd_workcell_manager.h"

/* assumptions:
1.Request for item only comes when vechicle has arrived
2.Arm only send fail when it is unable to recover. Once it sends fail, the whole request will fail. If required to be change see request_callback
3.
*/

/*to do
1. add a sleep at the end of main loop to prevent it from sending 2 request before state change.
*/

CssdWorkcellManager::CssdWorkcellManager(int no_of_subworkcell): Node("cssd_wm")
{ 
  //declaring parameter
  this->declare_parameter("dispenser_name_");
  this->declare_parameter("ip_address");
  this->declare_parameter("username");
  this->declare_parameter("password");
  this->declare_parameter("database_name");
  this->declare_parameter("R2R_docking_distance_threshold");
  this->declare_parameter("R2R_server_name");
  this->declare_parameter("max_request_size");

  this->get_parameter("dispenser_name_",dispenser_name_);
  this->get_parameter("ip_address",ip_address);
  this->get_parameter("username",username);
  this->get_parameter("password",password);
  this->get_parameter("database_name",database_name);
  this->get_parameter("R2R_docking_distance_threshold", R2R_docking_distance_threshold);
  this->get_parameter("R2R_server_name", R2R_server_name);
  this->get_parameter("max_request_size", max_request_size);

  std::cout<<" Starting "<< dispenser_name_ << std::endl;

  //creating calback group
  SubWorkcell_callback_group_=this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  RFM_callback_group_=this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  R2R_group_=this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  

  //creating pub/sub/client
  CheckInventory_ = this->create_subscription<rmf_msgs::msg::InventoryCheckRequest>("/dispenser_inventory_check_request", 
                                                                                    std::bind(&CssdWorkcellManager::inventory_check_callback, this, _1),
                                                                                    rmw_qos_profile_default,
                                                                                    RFM_callback_group_);
  
  DispenserRequest_ = this->create_subscription<rmf_msgs::msg::DispenserRequest>("/dispenser_request",
                                                                                std::bind(&CssdWorkcellManager::dispenser_request_callback, this, _1),
                                                                                rmw_qos_profile_default,
                                                                                RFM_callback_group_);
  
  SubWorkcellRespond_ = this->create_subscription<rmf_msgs::msg::DispenserResult>("/cssd_worckcell/dispenser_result",
                                                                          std::bind(&CssdWorkcellManager::SubWorkcell_respond_callback, this, _1),
                                                                          rmw_qos_profile_default,
                                                                          SubWorkcell_callback_group_);
  
  SubWorkcellState_ = this->create_subscription<rmf_msgs::msg::DispenserState>("/cssd_worckcell/dispenser_state",
                                                                        std::bind(&CssdWorkcellManager::SubWorkcell_state_callback, this, _1),
                                                                        rmw_qos_profile_default,
                                                                        SubWorkcell_callback_group_);
  
  InventoryCheckResponse_ = this->create_publisher<rmf_msgs::msg::InventoryCheckResponse>("/dispenser_inventory_check_response");
  SubWorkcellRequest_ =  this->create_publisher<rmf_msgs::msg::DispenserRequest>("/cssd_worckcell/dispenser_request");
  DispenserResponse_ = this->create_publisher<rmf_msgs::msg::DispenserResult>("/dispenser_result");
  
  R2R_client_ = this->create_client<xbee_interface::srv::R2R>(R2R_server_name,rmw_qos_profile_services_default,R2R_group_);

  //creating number of workcell variable based on number given in arg
  for (int i=1;i<=no_of_subworkcell;i++)
  {
    subworkcell.push_back(sub_workcell());
    (subworkcell.back().name) = "subworkcell" + std::to_string(i);
    std::string inventory_required = subworkcell.back().name + "_inventory";
    this->declare_parameter(inventory_required);
    this->get_parameter(inventory_required, subworkcell.back().item_carried_by_subworkcell);
  }



//check to make sure item carried is correct
  // for (std::vector<sub_workcell>::iterator it = subworkcell.begin() ; it != subworkcell.end(); ++it)
  // {
    
  //   RCLCPP_INFO(this->get_logger(), it->name);
  //   for (std::vector<std::string>::iterator i = it->item_carried_by_subworkcell.begin() ; i != it->item_carried_by_subworkcell.end(); ++i)
  //   {
      
  //     RCLCPP_INFO(this->get_logger(), *i);
  //   }
  // }

  //setting up sqlconn
  try 
  {
    driver = get_driver_instance();
    con = driver->connect("tcp://" + ip_address, username, password);
    con->setSchema(database_name);
    // con = driver->connect("tcp://127.0.0.1:3306", "malcomneo", "malcomneo");
    // con->setSchema("inventory");  } 
  }
  catch (sql::SQLException &e) 
  {
    RCLCPP_ERROR(this->get_logger(),"DB connection error.'%s' '%s'",(e.what()),(e.getErrorCode()));
  }

   RCLCPP_INFO(this->get_logger(),"Node is running");

}


void CssdWorkcellManager::inventory_check_callback(const rmf_msgs::msg::InventoryCheckRequest::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"Received inventory check request.");

  rmf_msgs::msg::InventoryCheckResponse response;
  response.check_id = msg -> check_id;

  //check whether the quantity requested is above max handled
  int quantity_requested = 0;
  for (int i=0;i<msg->items.size();i++)
  {
    quantity_requested +=msg->items[i].quantity;
  }
  if (quantity_requested>max_request_size)
  {
    response.all_available = false;
    InventoryCheckResponse_->publish(response);
    return;
  }

  //check whether the inventory has the quantity
  stmt = con->createStatement();
  res = stmt->executeQuery("SELECT item, COUNT(*) FROM workcell GROUP BY item"); 
  //if unsure what the result array look like, pass the query in mysql
  for (uint32_t i=0;i<msg->items.size();i++)
  { //looping through the item requested array

    while (res->next())
    { //looping through result until the item you get the item in the res array

      if (msg->items[i].item_type == res->getString("item"))
      {
        if (msg->items[i].quantity <= std::stoi(res->getString("count(*)")))
        { //if item meet quanitty, break the while loop and continue to the next iterator in for loop
          break;
        }
        else
        { //if does not meet. will publish false response and end function
          response.all_available = false;
          InventoryCheckResponse_->publish(response);
          return; 
        }
      }
      if (res->isLast() and msg->items[i].item_type != res->getString("item"))
      {
        response.all_available = false;
        InventoryCheckResponse_->publish(response);
        return;
      }
    }
  }

  // finish checking and all item has sufficient quantity. start adding to queue
  RCLCPP_INFO(this->get_logger(),"Inventory has enough quantity. Adding request to queue.");

  response.all_available = true;
  InventoryCheckResponse_->publish(response);

  for (uint32_t items_position=0; items_position<msg->items.size();items_position++)
  { //looping through requested item

    std::string item_type = msg->items[items_position].item_type;
    int32_t quantity = msg->items[items_position].quantity;

    for (auto subworkcell_pointer : subworkcell)
    { //looping through subworkcell
      if (std::find(subworkcell_pointer.item_carried_by_subworkcell.begin(), 
          subworkcell_pointer.tem_carried_by_subworkcell.end(), 
          item_type) != subworkcell_pointer->item_carried_by_subworkcell.end())
        {// if subworkcell is responsible for that item

        if ((subworkcell_pointer->queue.size() == 0) or (subworkcell_pointer->queue.back().request_id != msg->check_id) )
        {//if the last queue is not related to current order, create a new one
          subworkcell_pointer.queue.push_back(requests());
          subworkcell_pointer.queue.back().request_id = msg->check_id;
        }

        //get inventory from DB in an ordered fashion
        stmt = con->createStatement();
        res = stmt->executeQuery("SELECT * FROM workcell ORDER BY item");
        int item_entered_count=0; 
        while (res->next() and item_entered_count<quantity) 
        { //looping through the respond from DB and adding to the queue until quantity is met. 
          if(res ->getString("item") == item_type)
          {
            int aruco_id = std::stoi(res->getString("aruco_id"));
            subworkcell_pointer.queue.back().aruco_id.push_back(aruco_id);
            subworkcell_pointer.queue.back().item_type.push_back(item_type);            
            item_entered_count+=1;

            //remove from the DB once it is added.
            pstmt = con->prepareStatement("DELETE FROM workcell WHERE aruco_id = (?)");
            pstmt->setInt(1,aruco_id);
            pstmt->executeUpdate();
          }
        }
      }  
    }
  }
  RCLCPP_INFO(this->get_logger(),"Items added to queue.");

  //print out the queue to make sure that the items is added in the subworkcell
  // for (std::vector<sub_workcell>::iterator it = subworkcell.begin() ; it != subworkcell.end(); ++it)
  // {
  //           RCLCPP_INFO(this->get_logger(),it->name);

  //   for (std::vector<requests>::iterator queue = it->queue.begin() ; queue != it->queue.end(); ++queue)
  //   {
  //     std::cout<<queue -> request_id<<std::endl; 
  //     for (std::vector<uint32_t>::iterator aruco_id = queue->aruco_id.begin() ; aruco_id != queue->aruco_id.end(); ++aruco_id)
  //     {
        
  //       RCLCPP_INFO(this->get_logger(),std::to_string(*aruco_id));
  //     }
  //   }
    
  // }
}

void CssdWorkcellManager::failed_loading_handling(std::string request_id)
{ //loop through all the subworkcell. make sure that those with the same id is returned to the inventory.
  RCLCPP_INFO(this->get_logger(), "Starting to release items in request_id: %s", request_id);

  for (auto subworkcell_pointer : subworkcell)
  { //looping through the subworkcell
    for (auto queue_pointer : subworkcell_pointer.queue)
    {//looping through the queue
      if (queue_pointer.request_id == request_id)
      {
        for (int element=0;element<queue_pointer->aruco_id.size();element++)
        { //once found the request in queue with request id, loop through it and insert everything left back to the DB
          pstmt = con->prepareStatement("INSERT INTO workcell values (?,?)");
          pstmt->setInt(1,queue_pointer->aruco_id[element]);
          pstmt->setString(2,queue_pointer->item_type[element]);
          pstmt->executeUpdate();
        }

        //erases the request and break through the loop in the subworkcell.
        subworkcell_pointer->queue.erase(queue_pointer);
        break;
      }
    }
  }

  //send error to MFM
  rmf_msgs::msg::DispenserResult dispenser_result;
  dispenser_result.dispenser_name = dispenser_name_;
  dispenser_result.request_id = request_id;
  dispenser_result.success = 0;

}

void CssdWorkcellManager::SubWorkcell_respond_callback(const rmf_msgs::msg::DispenserResult::SharedPtr msg)
{
  for (auto subworkcell_pointer : subworkcell)
  {
    if (subworkcell_pointer.name == msg->dispenser_name)
    {
      switch (msg-> success)
      {
        case 0: subworkcell_pointer->dispenser_mode=2; return; //failure, turn dispenser_mode into 2. Error will be handled at main loop
        case 1: 
        { // loading is successful. check with R2R to make sure its true.
          std::vector<bool> compartment_status;
          std::vector<std::string> compartment_id;
          R2R_query(transporter_id);
          for (int i=0;i<compartment_id.size();i++)
          {
            if (compartment_id[i] == subworkcell_pointer->ongoing_compartment_id)
            {
              if (compartment_status[i] ==1)
              {
                RCLCPP_INFO(this->get_logger(), "Loading compartment %s succesful", compartment_id[i]);
                subworkcell_pointer->dispenser_mode = 0;
              }
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "Loading compartment %s failed", compartment_id[i]);
              subworkcell_pointer->dispenser_mode =2;
            }
         return; 
          }
        }
      }
    }
  }
}

bool CssdWorkcellManager::R2R_query(std::string device_id)
{
  this->R2R_response = false;
  while (!R2R_client_->wait_for_service(1s)) 
  {
    if (!rclcpp::ok()) 
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<xbee_interface::srv::R2R::Request>();
  request->mover_name = device_id;

  using ServiceResponseFuture =
        rclcpp::Client<xbee_interface::srv::R2R>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto response = future.get();

    if (response->estimated_distance > R2R_docking_distance_threshold or response->result == 0)
    {
      return 0;
    }
    RCLCPP_INFO(this->get_logger(), "R2R successful, trolley is at position.");
    R2R_response = true;
    current_trolley_compartment_status = response->states;
    trolley_compartment_id = response->state_names;
  };

    auto future_result = R2R_client_->async_send_request(request, response_received_callback);

  while(!R2R_response)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 1;
}


void CssdWorkcellManager::SubWorkcell_state_callback(const rmf_msgs::msg::DispenserState::SharedPtr msg)
{
  for (std::vector<sub_workcell>::iterator subworkcell_pointer = subworkcell.begin() ; subworkcell_pointer != subworkcell.end(); ++subworkcell_pointer)
  {
    if (subworkcell_pointer-> name == msg->dispenser_name)
    {
      switch (msg-> dispenser_mode)
      {
        case 0: subworkcell_pointer->dispenser_mode=0; return;
        case 1: subworkcell_pointer->dispenser_mode=1; RCLCPP_INFO(this->get_logger(), msg->dispenser_name + " dispenser is busy."); return;
        case 2: subworkcell_pointer->dispenser_mode=2;RCLCPP_ERROR(this->get_logger(), msg->dispenser_name +" dispenser has an error."); return;
      }
    }
  }
}


void CssdWorkcellManager::dispenser_request_callback(const rmf_msgs::msg::DispenserRequest::SharedPtr msg)
{
  request_id = msg->request_id;
  //check whether the manager is called
  if (msg->dispenser_name != dispenser_name_) return;

  RCLCPP_INFO(this->get_logger(), ("request received"));


 //R2R get trolley compartment state and make sure that the trolley is close
  if (R2R_query(transporter_id) == 0)
  {
    RCLCPP_ERROR(this->get_logger(), ("R2R unsuccessful."));
    pub_dispenser_task_result(false);
    new_request = false;
    failed_loading_handling(msg->request_id);
  }
  RCLCPP_INFO(this->get_logger(), ("request received"));

  //get quantity requested
  int quantity_requested = 0;
  for (int i=0;i<msg->items.size();i++)
  {
    quantity_requested +=msg->items[i].quantity;
  }
  //get quantity trolley can hold
  int trolley_empty_compartment = 0;
  for (int i=0;i<current_trolley_compartment_status.size();i++)
  {
    if (current_trolley_compartment_status[i] ==0) trolley_empty_compartment+=1;
  }

  if (quantity_requested>trolley_empty_compartment)
  {//if not enough space on trolley, prepare failed response and release reserved inventory
    failed_loading_handling(msg->request_id);
    new_request = false;
    pub_dispenser_task_result(false);
    return;
  }

  new_request = true;
  request_id = msg -> request_id;
  transporter_id = msg->transporter_type;
  planned_trolley_compartment_status = current_trolley_compartment_status;
}


void CssdWorkcellManager::pub_dispenser_task_result(bool success_result)
{
  rmf_msgs::msg::DispenserResult result_msg;
  result_msg.success = success_result;
  result_msg.request_id = request_id;
  result_msg.dispenser_name = dispenser_name_;
  DispenserResponse_ -> publish(result_msg);
}


// ????? 
void CssdWorkcellManager::task_execution_thread()
{ 
  while(true){
//  std::cout<<new_request<<std::endl;
    while (new_request)
    {
      int queue_remaining=0;

      do
      {
        queue_remaining =0;
        for (std::vector<sub_workcell>::iterator subworkcell_pointer = subworkcell.begin() ; subworkcell_pointer != subworkcell.end(); ++subworkcell_pointer)
        { 
          std::cout<<subworkcell_pointer->name<< "\t"<< subworkcell_pointer->dispenser_mode<<std::endl;
          switch (subworkcell_pointer-> dispenser_mode)
          {
            case 1:{queue_remaining+=1; continue;}
            case 2:
            {
              queue_remaining+=1;
              RCLCPP_ERROR(this->get_logger(), ("Error in %s.",subworkcell_pointer-> name));
              failed_loading_handling(request_id);
              new_request = false;
              break;
            }
          }

          for (std::vector<requests>::iterator queue_pointer = subworkcell_pointer->queue.begin() ; queue_pointer != subworkcell_pointer->queue.end(); ++queue_pointer)
          {
            if(queue_pointer->request_id == request_id)
            {
              rmf_msgs::msg::DispenserRequest request_msg;
              request_msg.dispenser_name = subworkcell_pointer -> name;
              request_msg.request_id = request_id;
              request_msg.transporter_type = transporter_id;
              
              rmf_msgs::msg::DispenserRequestItem item;
              item.item_type = "marker_id" + std::to_string(queue_pointer->aruco_id[0]);
              item.quantity = 1;

              for (int i=0;i<planned_trolley_compartment_status.size();i++)
              {
                if (planned_trolley_compartment_status[i] ==0)
                {
                  item.compartment_name = "marker_id" + trolley_compartment_id[i];
                  planned_trolley_compartment_status[i] = "1";
                  subworkcell_pointer-> ongoing_compartment_id = trolley_compartment_id[i];
                  break;
                }
              }
              
              request_msg.items.push_back(item);
              SubWorkcellRequest_ -> publish(request_msg);
              subworkcell_pointer -> dispenser_mode = 1;
              RCLCPP_INFO(this->get_logger(), ("New request sent to %s for item %d.",subworkcell_pointer-> name, item.item_type));
              queue_remaining+=1;
              if (queue_pointer->aruco_id.size() != 1)
              {
                queue_pointer->aruco_id.erase(queue_pointer->aruco_id.begin());
              }
              else
              {
                subworkcell_pointer->queue.erase(queue_pointer);
              }
              break;
            }
          }
        }
      }
      while (queue_remaining!=0); 

      if (queue_remaining==0)
      {
        new_request=false; 
        pub_dispenser_task_result(true);
      }
    }
  }
}


// void h_sig_sigint(int signum)
// {
//   // RCLCPP_INFO(this->get_logger(), "Receive signum: '%d'");
//   rclcpp::shutdown();
//   exit(1);
// }




// -----------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------- MAIN ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------


int main(int argc, char * argv[])
{

 // signal(SIGINT, h_sig_sigint);
  rclcpp::init(argc, argv);
  if ( atoi(argv[1])<1)
  { 
    std::cout<<"Error! Command line argument does not state number of workcell."<<std::endl;;
    return 1;
  }

  auto node = std::make_shared<CssdWorkcellManager>(atoi(argv[1]));
  std::thread main_wcm(&CssdWorkcellManager::task_execution_thread, node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}





