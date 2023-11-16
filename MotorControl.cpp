#include "MotorControl.h"

  CANInterface::CANInterface(int RX, int TX, int baudrate) {
    RX_ = RX;
    TX_ = TX;
    baudrate_ = baudrate;
  }


  void CANInterface::initCAN() {
    CAN.setPins(RX_, TX_);
    if (!CAN.begin(baudrate_)) {
      Serial.println("Starting CAN failed!");
      while (1);
    }
    else {
      Serial.println("CAN Initialized");
    }
  }

  void CANInterface::transmit(int id, float* data) {
    uint8_t dataBuffer[4];

    CAN.beginPacket(id);
    memcpy(dataBuffer, data, sizeof(float));
    CAN.write(dataBuffer, sizeof(dataBuffer));
    CAN.endPacket();
    Serial.println("Transmitted CAN message");
  }


  float CANInterface::receive(int id){
    // Try to parse packet
    int packetSize = CAN.parsePacket();

    if (packetSize) {
      // Received a packet
      if (!CAN.packetExtended() && CAN.packetId() == id) {
        // Check if the packet size matches the expected size of a float (4 bytes)
        if (packetSize == sizeof(float)) {
          float receivedData; // Declare the received float variable

          // Read the 4 bytes of the float from the CAN packet and store them in receivedData
          CAN.readBytes((char*)&receivedData, sizeof(float));
          Serial.print("Received data: ");
          Serial.println(receivedData);
          return receivedData; // Return the received float
        }
      }
  }

  // Return a default value (you can choose a meaningful default)
  return 0.0;

  }

  MotorControl::MotorControl() : sensor(AS5600_I2C), I2C(nullptr), motor(nullptr), driver(nullptr) {
    velocity_P_ = 0.1;
    velocity_I_ = 1.0;
    velocity_D_ = 0.0;
    LPF_velocity_Tf_ = 0.01;
    velocity_output_ramp_ = 300;

    angle_P_ = 1.0;
    angle_I_ = 0.0;
    angle_D_ = 0.0;
    LPF_angle_Tf_ = 0.0;
    angle_output_ramp_ = 100000;
    
    current_Pq_ = 5.0;
    current_Iq_ = 300.0;
    current_Pd_ = 5.0;
    current_Id_ = 300.0;
    LPF_current_q_Tf_ = 0.01;
    LPF_current_d_Tf_ = 0.01;
 
    voltage_limit_ = 12.0;
    current_limit_ = 0.5;
    velocity_limit_ = 2.0;

    driver_voltage_limit_ = 12.0;
  }

    // Motor info: https://www.aliexpress.us/item/3256804639269762.html?gatewayAdapt=glo2usa4itemAdapt
    void MotorControl::setBLDCMotor(int poles) {
      // motor = new BLDCMotor(poles,7.1 ,100, 1.45); // TODO: Find a correct value
      motor = new BLDCMotor(poles);
      if(motor == nullptr) {
        Serial.println("Motor Memory allocation failed!");
      }
    }

    void MotorControl::setBLDCDriver(int pwmA, int pwmB, int pwmC, int enable) {
      driver = new BLDCDriver3PWM(pwmA, pwmB, pwmC, enable);
      if (driver == nullptr) {
        Serial.println("Driver Memory allocation failed!");
      }
    }



    void MotorControl::setI2C(int encoder_num ,int sda, int scl, uint32_t frequency) {
      I2C = new TwoWire(encoder_num);
      I2C->begin(sda, scl, frequency);
    }

    void MotorControl::linkSensor() {
      sensor.init(I2C);
      motor->linkSensor(&sensor);
    }

    void MotorControl::linkDriver() {
      driver->voltage_power_supply = driver_voltage_limit_;
      if (driver != nullptr) {
        driver->init();
        motor->linkDriver(driver);
      } 
      else {
        Serial.println("Driver not initialized.");
        return;
      }
  
    }

    void MotorControl::linkCurrentSense() {
      // TODO: https://docs.simplefoc.com/current_sense
      // motor->linkCurrentSense(&current_sense);
      Serial.println("Current Sense Not implemented yet");
    }

    void MotorControl::setModulation(FOCModulationType modulation){
      // PWM Modulation Type: FOC torque control requires sinusoidal currents therefore please use either Sinusoidal PWM or Space vector PWM
      motor->foc_modulation = FOCModulationType::SpaceVectorPWM;
    }

    void MotorControl::setAlignVoltage(float voltage){
      motor->voltage_sensor_align = voltage; // default 3V
    }

    void MotorControl::setSensorOffset(float offset){
      motor->sensor_offset = offset; // default 0 rad
    }

    void MotorControl::setDriverConfig(float phase_resistance, float KV_rating, float phase_inductance){
      motor->phase_resistance = phase_resistance; // Ohms 
      motor->KV_rating = KV_rating; // rpm/volt 
      motor->phase_inductance = phase_inductance; // mH
    }

    void MotorControl::setTorqueController (TorqueControlType torque_type){
      motor->torque_controller = torque_type;
    }

    void MotorControl::setMotionController(MotionControlType motion_type){
      motor->controller = motion_type;
    }

    void MotorControl::setDefaultPID() {
      if (motor->controller == TorqueControlType::foc_current){
        //Default PID fro FOC Current Control
        setPID_current(current_Pq_, current_Iq_, current_Pd_, current_Id_, LPF_current_q_Tf_, LPF_current_d_Tf_);
      }

      if (motor->controller == MotionControlType::angle){
        // Set Default PID for Angle Control
        setPID_angle(angle_P_, angle_I_, angle_D_, angle_output_ramp_, LPF_angle_Tf_);
        setPID_velocity(velocity_P_, velocity_I_, velocity_D_, LPF_velocity_Tf_, velocity_output_ramp_);
        setMotorLimit(voltage_limit_, velocity_limit_); 
      }
    }

    void MotorControl::skipAlignment(float zero_electric_angle, Direction sensor_direction) {
      motor->zero_electric_angle  = zero_electric_angle; // rad
      motor->sensor_direction = sensor_direction; // CW or CCW
    }

    void MotorControl::configMotor(){
      // Link the motor and the sensor
      linkSensor();
      // link the motor to the Driver
      linkDriver();
      // link the motor to the current sense
      // linkCurrentSense();
      // set Motor PWM Modulation, Sensor & Motor Alightnment
      setModulation(FOCModulationType::SpaceVectorPWM);
      // set alignment voltage
      setAlignVoltage(3);
      // set Encoder zero offset
      setSensorOffset(0);
      // set torque contol type
      setTorqueController(TorqueControlType::voltage);
      // set motion control loop type
      setMotionController(MotionControlType::angle);
    }

    void MotorControl::init() {
      //init Serial for Arduino IDE
      Serial.begin(115200);
      motor->useMonitoring(Serial);

      // TODO: Monitoring: https://docs.simplefoc.com/monitoring

      // Start the Motor
      motor->init();
      Serial.println("Motor init success!");

      motor->initFOC();
      if (motor->initFOC()){
        Serial.println("FOC init success!");
      }
    }

    void MotorControl::run(float target_pos) {
      motor->loopFOC();
      motor->move(target_pos);
    }

    float MotorControl::getEncoder(){
      return sensor.getAngle();
    }


    void MotorControl::setPID_velocity(float P, float I, float D,
                                       float LPF_Tf, int output_ramp){
      motor->PID_velocity.P = P;
      motor->PID_velocity.I = I;
      motor->PID_velocity.D = D;
      motor->PID_velocity.output_ramp = output_ramp;
      motor->LPF_velocity.Tf = LPF_Tf;
    }

    void MotorControl::setPID_angle(float P, float I, float D, 
                                    int output_ramp , float LPF_Tf) {
      motor->P_angle.P = P;
      motor->P_angle.I = I;
      motor->P_angle.D = D;
      motor->P_angle.output_ramp = output_ramp;
      motor->LPF_angle.Tf = LPF_Tf;
    }

    void MotorControl::setPID_current(float Pq, float Iq, float Pd, float Id, 
                                      float LPF_q_Tf, float LPF_d_Tf) {
      motor->PID_current_q.P = Pq;
      motor->PID_current_q.I = Iq;
      motor->PID_current_d.P = Pd;
      motor->PID_current_d.I = Id;
      motor->LPF_current_q.Tf = LPF_q_Tf;
      motor->LPF_current_d.Tf = LPF_d_Tf;
    }

    void MotorControl::setMotorLimit(float voltage_limit, float velocity_limit) {
      motor->voltage_limit = voltage_limit;
      motor->velocity_limit = velocity_limit;
    }

    void MotorControl::showDebugMsgs(){

      Serial.print("Sensor zero offset is:");
      Serial.println(motor->zero_electric_angle, 4);
      Serial.print("Sensor natural direction is: ");
      Serial.println(motor->sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");
    }


    