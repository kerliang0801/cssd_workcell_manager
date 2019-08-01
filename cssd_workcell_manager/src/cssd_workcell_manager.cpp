#include "cssd_workcell_manager.h"

/* assumptions:
1.Request for item only comes when vechicle has arrived
2.Arm only send fail when it is unable to recover. Once it sends fail, the whole request will fail. If required to be change see request_callback
3.
*/

/*to do
1. add a sleep at the end of main loop to prevent it from sending 2 request before state change.
*/

CssdWorkcellManager::CssdWorkcellManager(int no_of_RAWM): Node("cssd_wm")
{ 
  //declaring parameter
  this->declare_parameter("dispenser_name");
  this->declare_parameter("ip_address");
  this->declare_parameter("username");
  this->declare_parameter("password");
  this->declare_parameter("database_name");
  this->declare_parameter("R2R_docking_distance_threshold");
  this->declare_parameter("R2R_server_name");
  this->declare_parameter("max_request_size");

  this->get_parameter("dispenser_name",dispenser_name);
  this->get_parameter("ip_address",ip_address);
  this->get_parameter("username",username);
  this->get_parameter("password",password);
  this->get_parameter("database_name",database_name);
  this->get_parameter("R2R_docking_distance_threshold", R2R_docking_distance_threshold);
  this->get_parameter("R2R_server_name", R2R_server_name);
  this->get_parameter("max_request_size", max_request_size);

  //creating calback group
  RAWM_callback_group_=this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  RFM_callback_group_=this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  R2R_group_=RAWM_callback_group_=this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  

  //creating pub/sub/client
  CheckInventory_ = this->create_subscription<rmf_msgs::msg::InventoryCheckRequest>("/dispenser_inventory_check_request", 
                                                                                    std::bind(&CssdWorkcellManager::inventory_check_callback, this, _1),
                                                                                    rmw_qos_profile_default,
                                                                                    RFM_callback_group_);
  
  DispenserRequest_ = this->create_subscription<rmf_msgs::msg::DispenserRequest>("/dispenser_request",
                                                                                std::bind(&CssdWorkcellManager::dispenser_request_callback, this, _1),
                                                                                rmw_qos_profile_default,
                                                                                RFM_callback_group_);
  
  RAWMRespond_ = this->create_subscription<rmf_msgs::msg::DispenserResult>("/RAWM_result",
                                                                          std::bind(&CssdWorkcellManager::RAWM_respond_callback, this, _1),
                                                                          rmw_qos_profile_default,
                                                                          RAWM_callback_group_);
  
  RAWMState_ = this->create_subscription<rmf_msgs::msg::DispenserState>("/RAWM_state",
                                                                        std::bind(&CssdWorkcellManager::RAWM_state_callback, this, _1),
                                                                        rmw_qos_profile_default,
                                                                        RAWM_callback_group_);
  
  InventoryCheckResponse_ = this->create_publisher<rmf_msgs::msg::InventoryCheckResponse>("/dispenser_inventory_check_response");
  RAWMRequest_ =  this->create_publisher<rmf_msgs::msg::DispenserRequest>("/RAWM_request");
  DispenserResponse_ = this->create_publisher<rmf_msgs::msg::DispenserResult>("/dispenser_result");
  
  R2R_client_ = this->create_client<xbee_interface::srv::R2R>(R2R_server_name,rmw_qos_profile_services_default,R2R_group_);

  //creating number of workcell variable based on number given in arg
  for (int i=1;i<=no_of_RAWM;i++)
  {
    RAWM.push_back(sub_workcell());
    (RAWM.back().name) = "RAWM" + std::to_string(i);
    std::string inventory_required = RAWM.back().name + "_inventory";
    this->declare_parameter(inventory_required);
    this->get_parameter(inventory_required, RAWM.back().item_carried_by_RAWM);
  }



//check to make sure item carried is correct
  // for (std::vector<sub_workcell>::iterator it = RAWM.begin() ; it != RAWM.end(); ++it)
  // {
    
  //   RCLCPP_INFO(this->get_logger(), it->name);
  //   for (std::vector<std::string>::iterator i = it->item_carried_by_RAWM.begin() ; i != it->item_carried_by_RAWM.end(); ++i)
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

}


void CssdWorkcellManager::inventory_check_callback(const rmf_msgs::msg::InventoryCheckRequest::SharedPtr msg)
{
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

  stmt = con->createStatement();
  res = stmt->executeQuery("SELECT item, COUNT(*) FROM workcell GROUP BY item"); 
  //if unsure what the result array look like, pass the query in mysql
  for (uint32_t i=0;i<msg->items.size();i++)
  { //looping through the item requested array
            std::cout<<i<<std::endl;

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
  //if all item checked in request msg and have sufficient quantity
  response.all_available = true;
  InventoryCheckResponse_->publish(response);

  for (uint32_t items_position=0; items_position<msg->items.size();items_position++)
  { //looping through requested item

    std::string item_type = msg->items[items_position].item_type;
    int32_t quantity = msg->items[items_position].quantity;

    for (std::vector<sub_workcell>::iterator RAWM_pointer = RAWM.begin() ; RAWM_pointer != RAWM.end(); ++RAWM_pointer)
    { //looping through RAWM
      if (std::find(RAWM_pointer->item_carried_by_RAWM.begin(), RAWM_pointer->item_carried_by_RAWM.end(), item_type) != RAWM_pointer->item_carried_by_RAWM.end())
      {// if item is carried by the RAWM / if RAWM is responsible for that item
        if ((RAWM_pointer->queue.size() == 0) or (RAWM_pointer->queue.back().request_id != msg->check_id) )
        {//if the last queue is not related to current order, create a new one
          RAWM_pointer->queue.push_back(requests());
          RAWM_pointer->queue.back().request_id = msg->check_id;
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
            RAWM_pointer->queue.back().aruco_id.push_back(aruco_id);
            RAWM_pointer->queue.back().item_type.push_back(item_type);            
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

  //print out the queue to make sure that the items is added in the RAWM
  // for (std::vector<sub_workcell>::iterator it = RAWM.begin() ; it != RAWM.end(); ++it)
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
{  //loop through all the RAWM. make sure that those with the same id is returned to the inventory.
  RCLCPP_INFO(this->get_logger(), "Starting to release items in request_id: %s", request_id);

  for (std::vector<sub_workcell>::iterator RAWM_pointer = RAWM.begin() ; RAWM_pointer != RAWM.end(); ++RAWM_pointer)
  { //looping through the RAWM
    for (std::vector<requests>::iterator queue_pointer = RAWM_pointer->queue.begin() ; queue_pointer != RAWM_pointer->queue.end(); ++queue_pointer)
    {//looping through the queue
      if (queue_pointer->request_id == request_id)
      {
        for (int element=0;element<queue_pointer->aruco_id.size();element++)
        { //once found the request in queue with request id, loop through it and insert everything left back to the DB
          pstmt = con->prepareStatement("INSERT INTO workcell values (?,?)");
          pstmt->setInt(1,queue_pointer->aruco_id[element]);
          pstmt->setString(2,queue_pointer->item_type[element]);
          pstmt->executeUpdate();
        }

        //erases the request and break through the loop in the RAWM.
        RAWM_pointer->queue.erase(queue_pointer);
        break;
      }
    }
  }


  //send error to RFM
  rmf_msgs::msg::DispenserResult dispenser_result;
  dispenser_result.dispenser_name = dispenser_name;
  dispenser_result.request_id = request_id;
  dispenser_result.success = 0;

}

void CssdWorkcellManager::RAWM_respond_callback(const rmf_msgs::msg::DispenserResult::SharedPtr msg)
{
  for (std::vector<sub_workcell>::iterator RAWM_pointer = RAWM.begin() ; RAWM_pointer != RAWM.end(); ++RAWM_pointer)
  {
    if (RAWM_pointer-> name == msg->dispenser_name)
    {
      switch (msg-> success)
      {
        case 0: RAWM_pointer->dispenser_mode=2;return;
        case 1: 
        {
          for (std::vector<sub_workcell>::iterator RAWM_pointer = RAWM.begin() ; RAWM_pointer != RAWM.end(); ++RAWM_pointer)
          {
            if (RAWM_pointer->name == msg->dispenser_name)
            {// R2R get payload drop status. Check that the last position is actually filled. If the thing missing straight away fail
              std::vector<bool> compartment_status;
              std::vector<std::string> compartment_id;
              R2R_query(transporter_id);
              for (int i=0;i<compartment_id.size();i++)
              {
                if (compartment_id[i] == RAWM_pointer->ongoing_compartment_id)
                {
                  if (compartment_status[i] ==1)
                  {
                    RCLCPP_INFO(this->get_logger(), "Loading compartment %s succesful", compartment_id[i]);
                    RAWM_pointer->dispenser_mode = 0;
                  }
                }
                else
                {
                  RCLCPP_INFO(this->get_logger(), "Loading compartment %s failed", compartment_id[i]);
                  RAWM_pointer->dispenser_mode =2;
                }
             return; 
              } 
            }         
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
    trolley_compartment_status = response->states;
    trolley_compartment_id = response->state_names;
  };

    auto future_result = R2R_client_->async_send_request(request, response_received_callback);

  while(!R2R_response)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 1;
}




void CssdWorkcellManager::RAWM_state_callback(const rmf_msgs::msg::DispenserState::SharedPtr msg)
{
  for (std::vector<sub_workcell>::iterator RAWM_pointer = RAWM.begin() ; RAWM_pointer != RAWM.end(); ++RAWM_pointer)
  {
    if (RAWM_pointer-> name == msg->dispenser_name)
    {
      switch (msg-> dispenser_mode)
      {
        case 0: RAWM_pointer->dispenser_mode=0; return;
        case 1: RAWM_pointer->dispenser_mode=1; RCLCPP_INFO(this->get_logger(), msg->dispenser_name + " dispenser is busy."); return;
        case 2: RAWM_pointer->dispenser_mode=2;RCLCPP_ERROR(this->get_logger(), msg->dispenser_name +" dispenser has an error."); return;
      }
    }
  }
}


void CssdWorkcellManager::dispenser_request_callback(const rmf_msgs::msg::DispenserRequest::SharedPtr msg)
{
  request_id = msg->request_id;
  //check whether the manager is called
  if (msg->dispenser_name != dispenser_name) return;

  RCLCPP_INFO(this->get_logger(), ("request received"));


 //R2R get trolley compartment state and make sure that the trolley is close
  if (R2R_query(transporter_id) == 0)
  {
    RCLCPP_ERROR(this->get_logger(), ("R2R unsuccessful."));
    send_result_to_meta_FMS(0);
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
  for (int i=0;i<trolley_compartment_status.size();i++)
  {
    if (trolley_compartment_status[i] ==0) trolley_empty_compartment+=1;
  }

  if (quantity_requested>trolley_empty_compartment)
  {//if not enough space on trolley, prepare failed response and release reserved inventory
    failed_loading_handling(msg->request_id);
    new_request = false;
    send_result_to_meta_FMS(0);
    return;
  }

  new_request = true;
  request_id = msg -> request_id;
  transporter_id = msg->transporter_type;
}

void CssdWorkcellManager::send_result_to_meta_FMS(bool result)
{
  rmf_msgs::msg::DispenserResult result_msg;
  result_msg.success = result;
  result_msg.request_id = request_id;
  result_msg.dispenser_name = dispenser_name;
  DispenserResponse_ -> publish(result_msg);
}

void CssdWorkcellManager::main()
{ 
  while(true){
//  std::cout<<new_request<<std::endl;
  while (new_request)
  {
    int queue_remaining=0;
    do
    {
      queue_remaining =0;
      for (std::vector<sub_workcell>::iterator RAWM_pointer = RAWM.begin() ; RAWM_pointer != RAWM.end(); ++RAWM_pointer)
      { 
        std::cout<<RAWM_pointer->name<< "\t"<< RAWM_pointer->dispenser_mode<<std::endl;
        switch (RAWM_pointer-> dispenser_mode)
        {
          case 1:{queue_remaining+=1; continue;}
          case 2:
          {
            queue_remaining+=1;
            RCLCPP_ERROR(this->get_logger(), ("Error in %s.",RAWM_pointer-> name));
            failed_loading_handling(request_id);
            new_request = false;
            break;
          }
        }

        for (std::vector<requests>::iterator queue_pointer = RAWM_pointer->queue.begin() ; queue_pointer != RAWM_pointer->queue.end(); ++queue_pointer)
        {
          if(queue_pointer->request_id == request_id)
          {
            rmf_msgs::msg::DispenserRequest request_msg;
            request_msg.dispenser_name = RAWM_pointer -> name;
            request_msg.request_id = request_id;
            request_msg.transporter_type = transporter_id;
            
            rmf_msgs::msg::DispenserRequestItem item;
            item.item_type = "marker_id" + std::to_string(queue_pointer->aruco_id[0]);
            item.quantity = 1;

            for (int i=0;i<trolley_compartment_status.size();i++)
            {
              if (trolley_compartment_status[i] ==0)
              {
                item.compartment_name = "marker_id" + trolley_compartment_id[i];
                trolley_compartment_status[i] = "1";
                RAWM_pointer-> ongoing_compartment_id = trolley_compartment_id[i];
                break;
              }
            }
            
            request_msg.items.push_back(item);
            RAWMRequest_ -> publish(request_msg);
            RAWM_pointer -> dispenser_mode = 1;
            RCLCPP_INFO(this->get_logger(), ("New request sent to %s for item %d.",RAWM_pointer-> name, item.item_type));
            queue_remaining+=1;
            if (queue_pointer->aruco_id.size() != 1)
            {
              queue_pointer->aruco_id.erase(queue_pointer->aruco_id.begin());
            }
            else
            {
              RAWM_pointer->queue.erase(queue_pointer);
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
    send_result_to_meta_FMS(1);
  }
}}}


// void h_sig_sigint(int signum)
// {
//   // RCLCPP_INFO(this->get_logger(), "Receive signum: '%d'");
//   rclcpp::shutdown();
//   exit(1);
// }

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
  std::thread main_wcm(&CssdWorkcellManager::main, node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}





