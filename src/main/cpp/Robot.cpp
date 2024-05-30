// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <cameraserver/CameraServer.h>
#include <frc/filter/SlewRateLimiter.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot {
  //frc::XboxController m_stick{0};
  frc::XboxController m_stick{0};
  frc::DoubleSolenoid m_solenoidBallPusher{1, frc::PneumaticsModuleType::CTREPCM, 0, 1};
  frc::Solenoid       m_solenoidLift0     {1, frc::PneumaticsModuleType::CTREPCM, 2};
  frc::Solenoid       m_solenoidLift1     {1, frc::PneumaticsModuleType::CTREPCM, 3};

  frc::PWMVictorSPX m_left1{0};
  frc::PWMVictorSPX m_left2{2};
  frc::PWMVictorSPX m_right1{1};
  frc::PWMVictorSPX m_right2{3};

  frc::PWMVictorSPX m_shoot0{6};
  frc::PWMVictorSPX m_shoot1{7};

  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{2 / 1_s};

  // The motors on the left side of the drive
  frc::MotorControllerGroup m_leftMotors{m_left1 , m_left2};

  // The motors on the right side of the drive
  frc::MotorControllerGroup m_rightMotors{m_right1, m_right2};

  // The robot's drive
  frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};

  frc::Timer m_ShootMotorTimer;

   double prevY = 0.0;
   double prevX = 0.0;

 public:
  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.SetInverted(true);
    frc::CameraServer::StartAutomaticCapture();
  }

  void TeleopPeriodic() override {
    double const ShootingSpeed = 1.0;
    double const IntakeSpeed   = 0.8;

   {
      double const accel = 0.1;
      double apppliedY = 0.0;
      double apppliedX = 0.0;

      double y = -m_stick.GetLeftY();
      double x = m_stick.GetLeftTriggerAxis() + -m_stick.GetRightTriggerAxis();

      /*if ( ( y * prevY ) > 0.0 )
      {
         if ( ( y - prevY ) > accel )
         {
            apppliedY = prevY + accel;
         }
         else
         {
            apppliedY = y;
         }
      }
      else
      {
         if ( ( y - prevY ) < -accel )
         {
            apppliedY = prevY - accel;
         }
         else
         {
            apppliedY = y;
         }
      }

      if ( ( x * prevX ) > 0.0 )
      {
         if ( ( x - prevX ) > accel )
         {
            apppliedX = prevX + accel;
         }
         else
         {
            apppliedX = x;
         }
      }
      else
      {
         if ( ( x - prevX )< -accel )
         {
            apppliedX = prevX - accel;
         }
         else
         {
            apppliedX = x;
         }
      }*/
      apppliedY = m_yspeedLimiter.Calculate( y );
      apppliedX = m_xspeedLimiter.Calculate( x );

      m_drive.ArcadeDrive( apppliedY, apppliedX );

      prevY = apppliedY;
      prevX = apppliedX;
   }

    // Lift Control
    if ( m_stick.GetRawButton( 7 ) ) 
    {
      m_ShootMotorTimer.Start();

       m_solenoidLift0.Set( true );
       m_solenoidLift1.Set( true );
    } 
    else 
    {
       m_ShootMotorTimer.Reset();
       m_solenoidLift0.Set( false );
       m_solenoidLift1.Set( false );
    }

    // Intake/Shooting Control
    if ( m_ShootMotorTimer.Get() > (units::time::second_t)0.5 ) 
    {
       m_shoot0.Set( ShootingSpeed );
       m_shoot1.Set( -ShootingSpeed );
    } 
    else if ( m_stick.GetRawButton( 6 ) )
    {
       m_shoot0.Set( -IntakeSpeed );
       m_shoot1.Set( IntakeSpeed );
    }
    else
    {
       m_shoot0.Set( 0.0 );
       m_shoot1.Set( 0.0 );
    }

    // Ball Pusher Control
    if ( m_stick.GetRawButton( 8 ) ) 
    {
       m_solenoidBallPusher.Set(frc::DoubleSolenoid::kReverse);
    } 
    else 
    {
       m_solenoidBallPusher.Set(frc::DoubleSolenoid::kForward);
    }
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
