#include "cssd_wm.h"




cssd_wm::cssd_wm(int no_of_workcell): Node("cssd_wm")
{
  CheckInventory_ = this->create_subscription<cssd_msgs::msg::InventoryCheckRequest>("/dispenser_inventory_check_request", std::bind(&cssd_wm::inventory_check, this, _1));
  DispenserRequest_ = this->create_subscription<cssd_msgs::msg::DispenserRequest>("/dispenser_request", std::bind(&cssd_wm::request_callback, this, _1));
  
  InventoryCheckResponse_ = this->create_publisher<cssd_msgs::msg::InventoryCheckResponse>("/dispenser_inventory_check_response");


  for (int i=1;i<=no_of_workcell;i++)
  {
    RAM.push_back(workcell());
    (RAM.back().name) = "workcell" + std::to_string(i);
    this->get_parameter(RAM.back().name + "_inventory", RAM.back().item_carried);
  }

  for (std::vector<workcell>::iterator it = RAM.begin() ; it != RAM.end(); ++it)
  {
    
    RCLCPP_INFO(this->get_logger(), it->name);
    for (std::vector<std::string>::iterator i = it->item_carried.begin() ; i != it->item_carried.end(); ++i)
    {
      
      RCLCPP_INFO(this->get_logger(), *i);
    }
  }

  try 
  {
    driver = get_driver_instance();
    con = driver->connect("tcp://127.0.0.1:3306", "malcomneo", "malcomneo");
    con->setSchema("inventory");

  } 
  catch (sql::SQLException &e) 
  {
    RCLCPP_ERROR(this->get_logger(),"DB connection error.'%s' '%s'",(e.what()),(e.getErrorCode()));
  }
}


void cssd_wm::inventory_check(const cssd_msgs::msg::InventoryCheckRequest::SharedPtr msg)
{
  cssd_msgs::msg::InventoryCheckResponse response;
  response.check_id = msg -> check_id;

  stmt = con->createStatement();
  res = stmt->executeQuery("SELECT item, COUNT(*) FROM workcell GROUP BY item");

  for (uint8_t i=0;i<msg->items.size();i++)
  { //looping through the item array
    while (res->next())
    {
      if (msg->items[i].item_type == res->getString("item"))
      {
        if (msg->items[i].quantity <= std::stoi(res->getString("count(*)")))
        {
          break;
        }
        else
        {
          response.all_available = false;
          InventoryCheckResponse_->publish(response);
          return;
        }
      }
    }
  }
  response.all_available = true;
  InventoryCheckResponse_->publish(response);
}



void cssd_wm::request_callback(const cssd_msgs::msg::DispenserRequest::SharedPtr msg)
{

  if (msg->dispenser_name != "cssd_workcell")
  {
    RCLCPP_INFO(this->get_logger(),"cssd dispenser not requested");
    return;
  }

  std::string request_id = msg->request_id;

  for (uint8_t items_position=0; items_position<msg->items.size();items_position++)
  { //looping through requested item
    std::string item_type = msg->items[items_position].item_type;
    uint32_t quantity = msg->items[items_position].quantity;

    for (std::vector<workcell>::iterator RAM_pointer = RAM.begin() ; RAM_pointer != RAM.end(); ++RAM_pointer)
    { //looping through RAM
      std::cout<< " in ram loop "<<std::endl;
      if (std::find(RAM_pointer->item_carried.begin(), RAM_pointer->item_carried.end(), item_type) != RAM_pointer->item_carried.end())
      {// if item is carried by the RAM in other words, if RAM is responsible for that item
              std::cout<< " found who own item "<<std::endl;
        if ((RAM_pointer->queue.size() == 0) or (RAM_pointer->queue.back().request_id != request_id) )
        {//if the last queue is not related to current order, create a new one
                std::cout<< " pushed back "<<std::endl;
          RAM_pointer->queue.push_back(requests());
          RAM_pointer->queue.back().request_id = request_id;
          RAM_pointer->queue.back().transporter_type = msg -> transporter_type;
        }
              std::cout<< " went past push back "<<std::endl;
        stmt = con->createStatement();
        res = stmt->executeQuery("SELECT * FROM workcell ORDER BY item");

        int item_entered_count=0;
        while (res->next() and item_entered_count<quantity) 
        {
          if(res ->getString("item") == item_type)
          {
            int aruco_id = std::stoi(res->getString("aruco_id"));
            RAM_pointer->queue.back().aruco_id.push_back(aruco_id);
            item_entered_count+=1;
            pstmt = con->prepareStatement("DELETE FROM workcell WHERE aruco_id = (?)");
            pstmt->setInt(1,aruco_id);
            pstmt->executeUpdate();
          }

        }
      }  
    }
  }
  for (std::vector<workcell>::iterator it = RAM.begin() ; it != RAM.end(); ++it)
  {
            RCLCPP_INFO(this->get_logger(),it->name);

    for (std::vector<requests>::iterator queue = it->queue.begin() ; queue != it->queue.end(); ++queue)
    {

      for (std::vector<uint32_t>::iterator aruco_id = queue->aruco_id.begin() ; aruco_id != queue->aruco_id.end(); ++aruco_id)
      {
        
        RCLCPP_INFO(this->get_logger(),std::to_string(*aruco_id));
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

int main(int argc, char * argv[])
{
  
 // signal(SIGINT, h_sig_sigint);
  rclcpp::init(argc, argv);
  if ( atoi(argv[1])<1)
  {
    std::cout<<"Error! Command line argument does not state number of workcell."<<std::endl;;
    return 1;
  }

  auto node = std::make_shared<cssd_wm>(atoi(argv[1]));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}





