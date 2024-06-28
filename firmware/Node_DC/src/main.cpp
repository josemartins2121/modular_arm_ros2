#include <Arduino.h>

#define ENCA 20 // Yellow
#define ENCB 21 // White

#define IN1 18
#define IN2 19
#define PWM 16

//limit switch pin
int limitSwitch = 6;

float position_offset = -1;


volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

unsigned long lastUpdateTime = 0;
int currentTargetIndex = 0;
const int numTargets = 5;
int targets[numTargets] = {3600,2700,1800, 900,0}; // Array of target positions


float target = 0;
void readEncoder();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);



/**************************************************************************************
 * INCLUDE
 **************************************************************************************/


#include <Wire.h>
#include <SPI.h>
#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-CriticalSection.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_MOSI_PIN  = 11;
static int const MKRCAN_MCP2515_MISO_PIN = 12;
static int const MKRCAN_MCP2515_SCK_PIN  = 10;

static int const MKRCAN_MCP2515_CS_PIN  = 13;
static int const MKRCAN_MCP2515_INT_PIN = 9;

static SPISettings const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};

static uint16_t const UPDATE_PERIOD_ANGLE_ms = 5;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const &);
void onExecuteCommand_1_1_Response_Received(uavcan::node::ExecuteCommand::Response_1_1  const & rsp);
uavcan::node::ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(uavcan::node::ExecuteCommand::Request_1_1 const &,cyphal::TransferMetadata const & metadata);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([]() { digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW); },
                       []() { digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const data) { return SPI1.transfer(data); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

cyphal::Publisher<uavcan::node::Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>
  (1*1000*1000UL /* = 1 sec in usecs. */);

cyphal::ServiceClient<uavcan::node::ExecuteCommand::Request_1_1> get_index_client = node_hdl.create_service_client<uavcan::node::ExecuteCommand::Request_1_1,uavcan::node::ExecuteCommand::Response_1_1 >( 
  2*1000*1000UL,
  onExecuteCommand_1_1_Response_Received);

cyphal::ServiceServer add_new_link_srv = node_hdl.create_service_server<uavcan::node::ExecuteCommand::Request_1_1,uavcan::node::ExecuteCommand::Response_1_1>( 
  2*1000*1000UL,
  onExecuteCommand_1_1_Request_Received);



float curr_angle = 0;
cyphal::Publisher<uavcan::primitive::scalar::Real16_1_0> curr_angle_pub;
cyphal::Subscription target_angle_sub;

static unsigned long target_angle_index = -1;

static unsigned long last_target_angle_sub_timestamp = 0;





/**************************************************************************************
 * Register
 **************************************************************************************/

static uint16_t     node_id                 = 3;
static uint16_t     master_node_id          = 0;
static CanardPortID port_id_curr_angle_pub  = 1;
static CanardPortID port_id_target_angle_sub  = 0;



void setup1(){
  
  Serial.begin(115200);
  while(!Serial) { }
  /* Setup LED pins */
  Serial.println("Starting Setup of Open Cyphal node");

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  node_hdl.setNodeId(static_cast<CanardNodeID>(node_id));

  /* Setup SPI access */
  SPI1.setRX(MKRCAN_MCP2515_MISO_PIN);
  SPI1.setTX(MKRCAN_MCP2515_MOSI_PIN);
  SPI1.setSCK(MKRCAN_MCP2515_SCK_PIN);
  
  SPI1.begin();
  SPI1.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, LOW);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  curr_angle_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real16_1_0>(port_id_curr_angle_pub,1*1000*1000UL /* = 1 sec in usecs */);

  target_angle_sub = node_hdl.create_subscription<uavcan::primitive::array::Real16_1_0>(
    port_id_target_angle_sub,
    [](uavcan::primitive::array::Real16_1_0 const & msg, cyphal::TransferMetadata const & metadata ){

      last_target_angle_sub_timestamp = millis();
      for (size_t sid = 0; sid < msg.value.size(); sid++)
      {
          //Serial.print(msg.value[sid]);
          //Serial.print(" ");
      }
      //Serial.println("");
      if(target_angle_index!=-1){
        target = msg.value[target_angle_index] * (800/(PI/2));
        Serial.println(msg.value[target_angle_index]);
      }
      //Serial.println(target);
    }
  );

  Serial.println("Adding Link to Master");
  std::string const cmd_param("Finding Master");
  uavcan::node::ExecuteCommand::Request_1_1 req;
  req.command = node_hdl.getNodeId();
  std::copy(cmd_param.begin(),cmd_param.end(),std::back_inserter(req.parameter));


  while (true) {

    if (!get_index_client->request(master_node_id, req)) {
      Serial.println("No Response from a master! Retrying in 1 second...");
      
    } else {
      Serial.println("Response received from the master!");
      break;  // Exit the loop if response is received
    }
      
    delay(1000);  // Wait for 1 second before retrying
  }  
  
}


void loop1(){
  

  /* Process all pending OpenCyphal actions */
  {
    CriticalSection crit_sec;
    node_hdl.spinSome();
  }

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  static unsigned long prev_angle_sensor = 0;

  unsigned long const now = millis();

  if(now - prev > 1000)
  {
    prev = now; 

    uavcan::node::Heartbeat_1_0 msg;

    msg.uptime = now / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    //Serial.println("Heartbeat sent !");

  }
  
  if((now - prev_angle_sensor) > UPDATE_PERIOD_ANGLE_ms)
  {
    uavcan::primitive::scalar::Real16_1_0 angle;

    angle.value = curr_angle;

    if (curr_angle_pub)
      curr_angle_pub->publish(angle);
    //Serial.println("Publish angle Data");

    prev_angle_sensor = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

}

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}

void onExecuteCommand_1_1_Response_Received(uavcan::node::ExecuteCommand::Response_1_1  const & rsp)
{
  if(rsp.status == uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS){
    Serial.println("Found a master");
  }
  else {
    Serial.println("Received Error code from master");
  }

}

uavcan::node::ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(uavcan::node::ExecuteCommand::Request_1_1 const & req,cyphal::TransferMetadata const & metadata)
{
  uavcan::node::ExecuteCommand::Response_1_1 rsp;
  Serial.println("Node Index Received");

  std::string parameter;

  std::copy(
    req.parameter.begin(),
    req.parameter.end(),
    std::back_inserter(parameter));

  for (uint8_t i = 0; i < parameter.size(); i++) {
    Serial.print(parameter[i]);
  }
  Serial.println("");

  // Extract the index from req.command
  int index = req.command;


  if(index != -1){
    target_angle_index = index;
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else{
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_NOT_AUTHORIZED;
  }

  return rsp;
}




void setup() {

  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  
  pinMode(ENCB,INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(PWM,OUTPUT);

  pinMode(limitSwitch,INPUT_PULLUP);


  // Turn off motors - Initial state
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  target = -5000;
}


void loop() {

  //int target = 1200;
\
  // Update target every 2 seconds (2000 milliseconds)
  if (millis() - lastUpdateTime >= 4000) {
    lastUpdateTime = millis();
    currentTargetIndex = (currentTargetIndex + 1) % numTargets; // Cycle through targets
  }

  //int target = targets[currentTargetIndex];

  /*
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    target = input.toInt()*(800/90);  // Convert the input to an integer target value
    Serial.print("New target set: ");
    Serial.println(target);
  }
  */
  //int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int pos = 0; 
  noInterrupts();
  pos = posi;
  interrupts();

  
  if(position_offset==-1){

    if(!digitalRead(limitSwitch)){
      position_offset = pos ;
      target = 0;
    }

  }
  

  // error
  int e = pos - (target+position_offset);

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);

  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;

  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);

  // store previous error
  eprev = e;

  //Serial.print(target);
  //Serial.print(" ");
  //Serial.print(pos);
  //Serial.println();
}


void readEncoder(){

  int b = digitalRead(ENCB);

  if(b > 0){
   posi++;
  }
  else{
    posi--;
  }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){

  analogWrite(pwm,pwmVal);

  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }

  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  } 
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }

}