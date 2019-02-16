
#include <frc/AnalogGyro.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include <frc/DoubleSolenoid.h>
#include <cameraserver/CameraServer.h>
#include "frc/BuiltInAccelerometer.h"
#include <frc/ADXRS450_Gyro.h>
#include <iostream>  
#include <cmath>
#include "rev/CANSparkMax.h"
using namespace std;



class Robot : public frc::TimedRobot {
  private:
  // Gyro calibration constant, may need to be adjusted. Gyro value of 360 is
  // set to correspond to one full revolution.
  static constexpr double kVoltsPerDegreePerSecond = 0.0128;
  float driveAngle = 0.0;
  float elbowAngle = 0.0;
  float wristAngle = 0.0;
  Boolean elbowStatus = false;
  //drive Control
  WPI_TalonSRX m_frontLeft{0};
  WPI_TalonSRX m_rearLeft{1};
  WPI_TalonSRX m_frontRight{2};
  WPI_TalonSRX m_rearRight{3};
  //game peice motors
  WPI_VictorSPX m_balls{4};
  WPI_VictorSPX m_wrist{5};
  //arm motor
  rev::CANSparkMax m_elbow{6, rev::CANSparkMax::MotorType::kBrushed};
  //drive set up
  WPI_TalonSRX m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};
  //Gyros
  frc::AnalogGyro driveGyro{0};
  frc::AnalogGyro wristGyro{1};
  frc::AnalogGyro elbowGyro{2};
  //double solenoids
  frc::DoubleSolenoid extension{0, 0, 1}; // pcm id 0, ports 0,1 
  frc::DoubleSolenoid hatchet{0, 2, 3}; //pcm id 0, ports 2,3
  frc::DoubleSolenoid release{1, 0, 1}; // pcm id 1, ports 0,1


  frc::Joystick m_joystick{0};


  float deadBand(float value){
      if(value < 0.1 || value > -0.1){
        return round(value);// (int)(value * 100.0)/100.0;
      }
  }
  int angleControl(float angle){
    if(angle > 120){
      return 120;
    }
    else if(angle < 0){
      return 0;
    }
    else{return round(angle);}
  }
  double convertAxis(double val){
    return (0.5*val+0.5);
  }


 public:
  void RobotInit() override {
    // Invert the left side motors. You may need to change or remove this to
    // match your robot.
    ///Removed since wasn't neccesary.
    //m_frontLeft.SetInverted(true);
    //m_rearLeft.SetInverted(true);

    driveGyro.SetSensitivity(0.0128);
    wristGyro.SetSensitivity(0.0128);
    elbowGyro.SetSensitivity(0.0128);

    //set up camera 
    cs::AxisCamera camera =
        frc::CameraServer::GetInstance()->AddAxisCamera("10.18.47.74");

  }

  /**
   * Mecanum drive is used with the gyro angle as an input.
   */
  void TeleopPeriodic() override {
    //update angle values
    driveAngle = driveGyro.GetAngle();
    wristAngle = wristGyro.GetAngle();
    elbowAngle = elbowGyro.GetAngle();

    //standard example
    //m_robotDrive.DriveCartesian(m_joystick.GetX(), m_joystick.GetY(), m_joystick.GetZ(), m_gyro.GetAngle());

    ///xbox drive
    //m_robotDrive.DriveCartesian(deadBand(m_joystick.GetX()), deadBand(-m_joystick.GetY()), deadBand(m_joystick.GetZ()), (-angle * 0.03));
    
    //specific to PS4
    m_robotDrive.DriveCartesian
    (
      deadBand
      (//calculate trigger values for forward and back 
        (-convertAxis(m_joystick.GetRawAxis(3))) + convertAxis(m_joystick.GetRawAxis(4))
      ), 
      deadBand(-m_joystick.GetRawAxis(0)), 
      deadBand(m_joystick.GetZ()), 
      (-angle * 0.03)
    );

    ///elbow control buttons
    ///Button 1 is Square move up
    ///Button 2 is X move down
    if(m_joystick.GetRawButton(1)){
      //asuming this is the correct rotation for forward
      m_elbow.Set(0.2);
    }
    else if(m_joystick.GetRawButton(2)){
      m_elbow.Set(-0.2);
    }
    else{
      m_elbow.Set(0);
    }

    ///Wrist functionality buttons
    ///Button 3 is O moves wrist down
    ///Button 4 is Triangle Moves wrist up
    if(m_joystick.GetRawButton(3)){
      m_wrist.Set(-0.1);
    }
    else if(m_joystick.GetRawButton(4)){
      m_wrist.Set(0.1);
    }
    else{
      m_wrist.Set(0);
    }

    /*Arm extenstion calculations
      A bit of work done here
      Here is the algorithm as to what is going on
      When the button is pressed:
        We check that the elbow status is false and that the angle is > 100 
          if true we release the arm and extend it completely
          if not we don't do anything.
      when the button is not pressed we check the angle for less than 100
      if it is we retract the arm and lock the release
    */
    if this is true we release the lock and extend the 
    if(m_joystick.GetRawButton(5)){
      if( !elbowStatus && (elbowAngle > 100)){
        elbowStatus = true;
        release.Set(frc::DoubleSolenoid::Value::kForward);
        Wait(0.2);
        extension.Set(frc::DoubleSolenoid::Value::kForward);
      }
    }
    else{
      if(elbowStatus && (elbowAngle < 100){
        extension.Set(frc::DoubleSolenoid::Value::kReverse);
        Wait(0.2);
        release.Set(frc::DoubleSolenoid::Value::kReverse);
        wait(0.1);
        release.Set(frc::DoubleSolenoid::Value::kForward)
        elbowStatus = false;
        
      }
      else{
        release.Set(frc::DoubleSolenoid::Value::kForward)
        extension.Set(frc::DoubleSolenoid::Value::kForward);
      }
    }
    ///using the left joystick we are going to release or retract the hatchet pistons
    if(m_joystick.GetRawAxis(1) > 0.5){
      hatchet.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else{
      hatchet.Set(frc::DoubleSolenoid::Value::kReverse);
    }

    ///using the right joystick we are going to control the ball intake
    m_balls.Set(m_joystick.GetRawAxis(5));

    //camera servo control
    // will automatically center itself looking down
    //button press will allow for looking up at the arm. 
    if(m_joystick.GetRawButton(11)){
      camServo.SetAngle(120);
    }
    else{
      camServo.SetAngle(120-angleControl(elbowAngle));
    }

  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
