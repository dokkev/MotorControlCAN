#ifndef MOTORCONTROL_H    
#define MOTORCONTROL_H    

#include <SimpleFOC.h>
#include <CAN.h>


 class CANInterface {
  private:
    int RX_;
    int TX_;
    int baudrate_;

  public:
    CANInterface(int RX, int TX, int baudrate);
    void initCAN();
    void transmit(int id, float* data);
    float receive(int id);
};


/// \brief Library for controlling a motor using SimpleFOC with ESP32
class MotorControl {
  private:
    /**
      * Motor, Driver, Encoder Objects
      */
    BLDCMotor* motor;
    BLDCDriver3PWM* driver;
    MagneticSensorI2C sensor;
    TwoWire* I2C;

    /**
      * PID parameters
      */
    float velocity_P_;
    float velocity_I_ ;
    float velocity_D_;
    float LPF_velocity_Tf_;     // velocity low pass filtering
    float velocity_output_ramp_;  // jerk control using voltage voltage ramp [V/s]

    float angle_P_;
    float angle_I_;
    float angle_D_;
    float LPF_angle_Tf_; // angle low pass filtering use only for very noisy position sensors - try to avoid and keep the values very small
    int angle_output_ramp_; // acceleration control using output ramp [rad/s^2]
    
    float current_Pq_;
    float current_Iq_;
    float current_Pd_;
    float current_Id_;
    float LPF_current_q_Tf_;
    float LPF_current_d_Tf_;

    /**
      * Motor Limitations
      */
    float voltage_limit_;
    float current_limit_; // use if phase_resistance set. Choose either voltage or current limit
    float velocity_limit_;

    /**
     * Driver Limitations
     */
    float driver_voltage_limit_;


  public:
    MotorControl();

    /// @brief Create an instance of the BLDCMotor class and provide it the number of pole pairs of the motor.
    /// @param poles 
    void setBLDCMotor(int poles);

    /// @brief create the interface to the BLDC driver
    /// https://docs.simplefoc.com/bldcdriver3pwm for more info
    /// @param pwmA 
    /// @param pwmB 
    /// @param pwmC 
    /// @param enable 
    void setBLDCDriver(int pwmA, int pwmB, int pwmC, int enable);

    /// @brief set the I2C Encoder configuration
    /// @param encoder_num
    /// @param sda
    /// @param scl
    /// @param frequency
    void setI2C(int encoder_num, int sda, int scl, uint32_t frequency);
    
    /// @brief link the sensor to the motor
    /// See the position https://docs.simplefoc.com/sensors for more info
    void linkSensor();

    /// @brief link the driver to the motor
    void linkDriver();

    // @brief link the current sensor to the motor
    /// See https://docs.simplefoc.com/current_sense for more info
    void linkCurrentSense();

    /// @brief set the FOC modulation type
    /// @param modulation
    /// SinePWM; (default)
    /// SpaceVectorPWM;
    /// Trapezoid_120;
    /// Trapezoid_150;
    void setModulation(FOCModulationType modulation);

    /// @brief Set voltage used for the motor and sensor alignment
    /// @param voltage 
    void setAlignVoltage(float voltage);

    /// @brief Set sensor absolute zero offset
    /// @param offset
    void setSensorOffset(float offset);

    /// @brief Set motor phase resistance and KV rating are optional parameters which are not used for current based torque modes. 
    /// These variables are used to estimate the motor current in the voltage torque mode and for open-loop motion control.
    /// @param phase_resistance 
    /// @param KV_rating 
    /// @param phase_inductance
    void setDriverConfig(float phase_resistance, float KV_rating, float phase_inductance);

    /// @brief set torque mode to be used
    /// @param torque_type
    /// voltage;    ( default )
    /// dc_current;
    /// foc_current;
    void setTorqueController (TorqueControlType torque_type);

    /// @brief set the motion contol loop to be used
    /// @param motion_type
    /// torque      - torque control 
    /// velocity    - velocity motion control
    /// angle       - position/angle motion control
    /// velocity_openloop    - velocity open-loop control
    /// angle_openloop       - position open-loop control
    void setMotionController (MotionControlType motion_type);

    /// @brief  set the default PID values for the motor
    void setDefaultPID();

    /// @brief Skip motor alignment upon FOC motor init
    /// @param zero_electric_angle 
    /// @param sensor_direction 
    void skipAlignment(float zero_electric_angle, Direction sensor_direction);

    /// @brief Set new PID values for the motor velocity control loop
    /// @param velocity_P
    /// @param velocity_I
    /// @param velocity_D
    /// @param LPF_Tf (optional)
    /// @param output_ramp (optional)
    void setPID_velocity(float P, float I, float D,
                         float LPF_Tf, int output_ramp);

    /// @brief Set new PID values for the motor angle control loop
    /// @param angle_P
    /// @param angle_I
    /// @param angle_D
    /// @param LPF_angle_Tf (optional)
    /// @param angle_output_ramp (optional)
    void setPID_angle(float P, float I, float D, 
                      int output_ramp, float LPF_Tf);

    /// @brief Set new PID values for the motor current control loop
    /// @param Pq
    /// @param Iq
    /// @param Pd
    /// @param Id
    /// @param LPF_current_q_Tf (optional)
    /// @param LPF_current_d_Tf (optional)
    void setPID_current(float Pq, float Iq, float Pd, float Id, 
                                      float LPF_q_Tf, float LPF_d_Tf);

    /// @brief Set limits for the motor voltage and velocity
    /// @param voltage_limit
    /// @param velocity_limit
    void setMotorLimit(float voltage_limit, float velocity_limit);


    /// @brief Display debug messages
    void showDebugMsgs();

    /// @brief Run functions to configure the motor parameters
    void configMotor();

    /// @brief Run functions to initialize the motor
    void init();

    /// @brief Run functions to control the motor real-time and display the sensor angle
    void run(float target_pos);

    /// @brief Get the sensor angle
    float getEncoder();
};


#endif // MOTORCONTROL_H    // Put this line at the end of your file.