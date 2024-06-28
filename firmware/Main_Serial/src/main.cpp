#include <Arduino.h>
#include <SPI.h>
#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-CriticalSection.h>


/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

//using namespace uavcan::node;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = 17;
static int const MKRCAN_MCP2515_INT_PIN = 20;


static SPISettings const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};

//SPI.beginTransaction(MCP2515x_SPI_SETTING);


/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const &);
void onHeartbeat_1_0_Received(uavcan::node::Heartbeat_1_0 const & msg, cyphal::TransferMetadata const & metadata);
uavcan::node::ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(uavcan::node::ExecuteCommand::Request_1_1 const &,cyphal::TransferMetadata const & metadata);
void onExecuteCommand_1_1_Response_Received(uavcan::node::ExecuteCommand::Response_1_1 const & rsp);


/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([]() { digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW); },
                       []() { digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const data) { return SPI.transfer(data); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

cyphal::Publisher<uavcan::node::Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>
  (1*1000*1000UL /* = 1 sec in usecs. */);

cyphal::ServiceServer add_new_link_srv = node_hdl.create_service_server<uavcan::node::ExecuteCommand::Request_1_1,uavcan::node::ExecuteCommand::Response_1_1>( 
  2*1000*1000UL,
  onExecuteCommand_1_1_Request_Received);


cyphal::ServiceClient<uavcan::node::ExecuteCommand::Request_1_1> srv_client = node_hdl.create_service_client<uavcan::node::ExecuteCommand::Request_1_1, uavcan::node::ExecuteCommand::Response_1_1>(
  2*1000*1000UL,
  onExecuteCommand_1_1_Response_Received);

cyphal::Subscription heartbeat_subscription = node_hdl.create_subscription<uavcan::node::Heartbeat_1_0>(onHeartbeat_1_0_Received);

cyphal::Subscription curr_angle_sub;
cyphal::Publisher<uavcan::primitive::array::Real16_1_0> target_angle_pub;

static uint16_t const UPDATE_PERIOD_ANGLE_ms = 10;
static uint16_t const HEARTBEAT_PERIOD_ANGLE_ms = 1000;

static unsigned long last_curr_angle_sub_timestamp = 0;

#define N_links  8   // need to be %8

float target_angles[N_links] = {0};
std::vector<int> links_ID;
std::vector<int> new_links_ID;

int getIdPosition(int NodeId) {
  
  // Find the position of NodeId in the vector
  auto it = std::find(links_ID.begin(), links_ID.end(), NodeId);

  // Check if the NodeId is found
  if (it != links_ID.end()) {
    // If found, return the position
    return std::distance(links_ID.begin(), it);
  } else {

    // If not found and the vector is full, return -1
    if (links_ID.size() >= N_links) {
      return -1;
    }
    // Add NodeId to the vector
    links_ID.push_back(NodeId);
    
    // Return the position of the newly added NodeId
    return links_ID.size() - 1;
  }
}

/**************************************************************************************
 * Register
 **************************************************************************************/

static uint16_t     node_id                 = 0;
static CanardPortID port_id_curr_angle_sub  = 1;
static CanardPortID port_id_target_angle_pub  = 0;


/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  //target_angles[0] = 5.0;
  Serial.begin(115200);
  while(!Serial) { }
  delay(1000);
  Serial.println("|---- OpenCyphal Master Node ----|");

  node_hdl.setNodeId(static_cast<CanardNodeID>(node_id));

  /* Setup SPI access */
  SPI.begin();
  SPI.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  
  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, LOW);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  

  curr_angle_sub = node_hdl.create_subscription<uavcan::primitive::scalar::Real16_1_0>(
    port_id_curr_angle_sub,
    [](uavcan::primitive::scalar::Real16_1_0 const & msg, cyphal::TransferMetadata const & metadata ){

      last_curr_angle_sub_timestamp = millis();

      char msg_buf[70];
        snprintf(
          msg_buf,
          sizeof(msg_buf),
          "Node ID= %d, Angle = %0.2f ",
          metadata.remote_node_id,
          msg.value);
      
      //Serial.println(msg_buf);

    }
  );


  target_angle_pub = node_hdl.create_publisher<uavcan::primitive::array::Real16_1_0>(port_id_target_angle_pub,1*1000*1000UL /* = 1 sec in usecs */);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("setup finished");
}




void loop()
{
  /* Process all pending OpenCyphal actions.
   */
  {
    CriticalSection crit_sec;
    node_hdl.spinSome();
  }

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  static unsigned long prev_angle_sensor = 0;

  unsigned long const now = millis();


  if(now - prev > HEARTBEAT_PERIOD_ANGLE_ms)
  {
    prev = now; 

    uavcan::node::Heartbeat_1_0 msg;

    msg.uptime = now / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    /*
    
    if(links_ID.size() > 0 ){
      //Serial.println("Heartbeat sent !");
      Serial.pritnln("Links_ID : ")
      for(int i=0; i < links_ID.size(); i++){
        Serial.print(links_ID[i]);
        Serial.print(" ");
      }
      Serial.println("");
    }
    */
    

  }

  if((now - prev_angle_sensor) > UPDATE_PERIOD_ANGLE_ms)
  {
    uavcan::primitive::array::Real16_1_0 angles;

    for (int i = 0; i < N_links; ++i) {
      angles.value.push_back(target_angles[i]);
    }

    if (target_angle_pub)
      target_angle_pub->publish(angles);
    
    String angleString = "Publishing angle commands:";

    for (int i = 0; i < N_links; ++i) {
        angles.value.push_back(target_angles[i]);
        angleString += " " + String(target_angles[i]);
    }

    Serial.println(angleString);

    prev_angle_sensor = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  while(!new_links_ID.empty() ){ 

    int current_Node_id = new_links_ID.front();
    new_links_ID.erase(new_links_ID.begin()); 

    // Get the index of the remote node id
    int index = getIdPosition(current_Node_id);

    // Ensure the index is valid before proceeding
    if (index != -1) {
      uavcan::node::ExecuteCommand::Request_1_1 req;
      req.command = index;  // Set the command to the index

      Serial.print("Sending Index ");
      Serial.print(index); 
      Serial.print(" to Node: ");
      Serial.println(current_Node_id);

      // Send the request with the index as the command
      if (!srv_client->request(current_Node_id, req)) {
        Serial.println("Sending Node Index Failed");
      }

    } 
    else {
      Serial.println("Failed to get a valid index for remote node id.");
    }
  
  }

}


/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}

void onHeartbeat_1_0_Received(uavcan::node::Heartbeat_1_0 const & msg, cyphal::TransferMetadata const & metadata)
{
  char msg_buf[70];
  snprintf(
    msg_buf,
    sizeof(msg_buf),
    "Node ID= %d, Uptime = %d, Health = %d, Mode = %d, VSSC = %d",
    metadata.remote_node_id,
    msg.uptime,
    msg.health.value,
    msg.mode.value,
    msg.vendor_specific_status_code);

  //Serial.println(msg_buf);
}

uavcan::node::ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(uavcan::node::ExecuteCommand::Request_1_1 const & req,cyphal::TransferMetadata const & metadata)
{
  uavcan::node::ExecuteCommand::Response_1_1 rsp;
  Serial.println("New Node Request!");

  std::string parameter;

  std::copy(
    req.parameter.begin(),
    req.parameter.end(),
    std::back_inserter(parameter));

  for (uint8_t i = 0; i < parameter.size(); i++) {
    Serial.print(parameter[i]);
  }
  Serial.println("");

  int index = getIdPosition(metadata.remote_node_id);

  if(index != -1){
    new_links_ID.push_back(metadata.remote_node_id);
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else{
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_NOT_AUTHORIZED;
  }

  return rsp;
}

void onExecuteCommand_1_1_Response_Received(uavcan::node::ExecuteCommand::Response_1_1 const & rsp)
{
  if (rsp.status == uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS)
    Serial.println("Node successfully received index");
  else
    Serial.println("Error while sending Node Index");
}




void updateJointPosition(int jointIndex, float positionCommand);
void parseMessage(String message);


void setup1() {
  // Initialize serial communication at 115200 baud rate
  //Serial.begin(115200);
}


void loop1() {
  if (Serial.available()) {
    String message = Serial.readStringUntil('>');
    if (message.startsWith("<")) {
      message.remove(0, 1); // Remove start sequence character
      parseMessage(message);
    }
  }
}

void updateJointPosition(int jointIndex, float positionCommand) {
  // Implement this function to update the position of the joint
  // For example:
  /*
  Serial.print("Joint ");
  Serial.print(jointIndex);
  Serial.print(": ");
  Serial.println(positionCommand);
  */
  int index = getIdPosition(jointIndex);

  if(index != -1){
    target_angles[index] = positionCommand;
  }
  
}


void parseMessage(String message) {
  while (message.length() > 0) {
    int separatorIndex = message.indexOf(';');
    String jointCommand = separatorIndex == -1 ? message : message.substring(0, separatorIndex);
    int commaIndex = jointCommand.indexOf(',');

    if (commaIndex != -1) {
      int jointIndex = jointCommand.substring(0, commaIndex).toInt();
      float positionCommand = jointCommand.substring(commaIndex + 1).toFloat();

      // Update joint position
      updateJointPosition(jointIndex, positionCommand);
    }

    if (separatorIndex == -1) {
      break;
    } else {
      message.remove(0, separatorIndex + 1);
    }
  }
}
