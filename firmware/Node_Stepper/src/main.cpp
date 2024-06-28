/**************************************************************************************
 * INCLUDE
 **************************************************************************************/


#include <SimpleFOC.h>
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

static uint16_t     node_id                 = 1;
static uint16_t     master_node_id          = 0;
static CanardPortID port_id_curr_angle_pub  = 1;
static CanardPortID port_id_target_angle_sub  = 0;


#define CS_PIN 20

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI,CS_PIN);

// Stepper motor & driver instance
StepperMotor motor = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(4, 3, 2, 1,5,0);

//limit switch pin
int limitSwitch = 6;

// target set point variable
float target = 0;
// velocity set point variable
float target_parameter = 0;

//
float offset_pos = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target, cmd); 
Serial.println(motor.zero_electric_angle);};
void doParameter(char* cmd) { command.scalar(&target_parameter, cmd); }
void incParameter(char* cmd) { motor.zero_electric_angle+= 0.01;
Serial.println(motor.zero_electric_angle);}
void reduParameter(char* cmd) { motor.zero_electric_angle-= 0.01;
Serial.println(motor.zero_electric_angle); }



int magnetStatus = 0; //value of the status register (MD, ML, MH)
void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly
  //(magnetStatus & 32) != 32
  while(1) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    Serial.print("Magnet status: ");
    Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  Serial.println("Magnet found!");
  while(1);
}





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
        target = -msg.value[target_angle_index] * 20;
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

  pinMode(limitSwitch,INPUT_PULLUP);

  // initialise magnetic sensor hardware
  sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config

  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // motor voltage while sensor aligning 
  motor.voltage_sensor_align = 4 ;


  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;

  // default voltage_power_supply
  motor.voltage_limit = 12;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 3000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  motor.P_angle.P = 50;
 
  motor.velocity_limit = 10;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; 

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('P', doParameter, "target parameter");
  command.add('+', incParameter, "incremeter parameter");
  command.add('-', reduParameter, "reduce parameter");
  
  

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  motor.zero_electric_angle = 0.15;
  _delay(1000);
  target = -2*PI*20;
  
}

float position_offset = -1;

void loop() {
  
  // main FOC algorithm function
  motor.loopFOC();
  
  // Motion control function
  
  motor.move(target+position_offset);
  
  
  static unsigned long prev_sensor = 0;
  static unsigned long prev_mov = 0;

  
  unsigned long const now = millis();


  if(target_parameter!= 0){
    motor.zero_electric_angle += target_parameter ;
    target_parameter = 0;
    Serial.println(motor.zero_electric_angle);
  }
  
  

  
  
  if((now-prev_sensor) > 10 )
  {
    prev_sensor = now;
    
    if(position_offset==-1){

    sensor.update();
    float curr_pos = sensor.getAngle();
    //Serial.print(curr_pos);
    //Serial.print(" ");
    //Serial.println(digitalRead(limitSwitch));
  

    if(!digitalRead(limitSwitch)){
      position_offset = curr_pos + PI*20 ;
      target = 0;
    }

    }
    
  }
  /*
  if((now-prev_mov) > 5000 )
  {
    prev_mov = now;
    
    if(position_offset!=-1){

      if(target == -10){
        target = -20;
      }
      else target = -10;

    }
    
  }
  */
  
  // function intended to be used with serial plotter to monitor motor variables
  //motor.monitor();

  // user communication
  command.run();
  

}