// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/phoenix/motorcontrol/FollowerType.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include <iostream>
#include <frc/DriverStation.h>
#include <frc/Encoder.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/motorcontrol/PWMVictorSPX.h>



/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot {
  frc::PWMVictorSPX m_leftMotor{1};
  frc::PWMVictorSPX m_rightMotor{2};
   frc::PWMVictorSPX m_leftMotorTwo{3};
  frc::PWMVictorSPX m_rightMotorTwo{4};
  frc::PWMVictorSPX feedWheel{5};
  frc::PWMVictorSPX shooterWheel{6};



  frc::DifferentialDrive m_robotDrive{
      [&](double output) { m_leftMotor.Set(output); },
      [&](double output) { m_rightMotor.Set(output); }};
  frc::Joystick m_stick{0};

frc::sim::DifferentialDrivetrainSim m_driveSim =
  frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
    frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
    frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
    frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
);
frc::Field2d m_field;
frc::Timer timer;
 public:
  Robot() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftMotor);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightMotor);
  }

 // bool isShooterWheelSpinning() {
 //   return true;
//}

  void RobotInit() override {
    bool booleanFlag = false;
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.SetInverted(false);
    m_leftMotorTwo.SetInverted(true);
    m_rightMotorTwo.SetInverted(true);
  m_leftMotor.AddFollower(m_leftMotorTwo);
  m_rightMotor.AddFollower(m_rightMotorTwo);
  m_robotDrive.SetDeadband(0.1);

  
// Check the left trigger axis value on the Xbox controller
    //    double leftTriggerValue = m_stick.GetRawAxis(2); // Assuming the left trigger axis is axis 2
 frc::SmartDashboard::PutData("Field", &m_field);
 
 //frc::SmartDashboard::PutBoolean("Shooter Wheel", isShooterWheelSpinning);
 //frc::SmartDashboard::Put("Drive", m_robotDrive);
 // Do this in either robot periodic or subsystem periodic
//m_field.SetRobotPose(m_odometry.GetPose());
  }

  void AutonomousInit() override {
        // Set the timer to count for 3 seconds
        timer.Reset();
        timer.Start();
    }

    void AutonomousPeriodic() override {
        // Drive the robot backward at a constant speed for 3 seconds
        if (timer.Get() < 3.0_s) {
            m_robotDrive.ArcadeDrive(0.3,0.4);
        } else {
            m_robotDrive.ArcadeDrive(0.0, 0.0); // Stop driving after 3 seconds
        }
    }

void RobotPeridoic() override {
feedWheel.Get();
  // Feed/Shooter Wheels
     frc::SmartDashboard::PutNumber("Feed Wheel", feedWheel.Get());  
     frc::SmartDashboard::PutNumber("Shooter Wheels", shooterWheel.Get());  
     // Drive Wheels
     frc::SmartDashboard::PutNumber("Left Back", m_leftMotorTwo.Get());  
     frc::SmartDashboard::PutNumber("Left Front", m_leftMotor.Get());  
     frc::SmartDashboard::PutNumber("Right Front", m_rightMotor.Get());  
     frc::SmartDashboard::PutNumber("Right Back", m_rightMotorTwo.Get());  
  //   frc::SmartDashboard::PutNumber("Drive", m_robotDrive);
     // Run Motor Checks
   //   frc::SmartDashboard::GetString("Is it a Dead Feed Wheel?", feedWheel.IsAlive());
}

void DisabledPeriodic() override {
 // feedWheel.Set(0.5);
}

  void TeleopPeriodic() override {

    // Intake 
      if (m_stick.GetRawButtonPressed(4)) {
      //   booleanFlag = true;
            feedWheel.Set(-1.0);
            shooterWheel.Set(-1);
        } else if(m_stick.GetRawButtonReleased(4)) {
       //   booleanFlag = false;
            feedWheel.Set(0.0);
            shooterWheel.Set(0);
        }
  // Rev 
 if (m_stick.GetRawButtonPressed(1)) { 
            shooterWheel.Set(1.0); 
        } else if(m_stick.GetRawButtonReleased(1)) {
            shooterWheel.Set(0.0);
        }
   //Shoot after rev
   if(m_stick.GetRawButtonPressed(3)) {
    feedWheel.Set(1);
   } else if(m_stick.GetRawButtonReleased(3)){
    feedWheel.Set(0);
   }
   //Drive train 
 // frc::SmartDashboard::PutData("Drive", m_robotDrive);
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), -m_stick.GetX());
  }
};



#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
