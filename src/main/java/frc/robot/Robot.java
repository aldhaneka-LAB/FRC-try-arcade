// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final VictorSPX intake_1 = new VictorSPX(7);

  private int front_right_motor = 2;
  private int front_left_motor = 4;
  
  private int back_right_motor = 1;
  private int back_left_motor = 3;  

  // private SlewRateLimiter test = new SlewRateLimiter(1);
  private final CANSparkMax shooter = new CANSparkMax(6, MotorType.kBrushless);
  private PIDController m_shooterController = new PIDController(0, 0, 0);  
  private double shooterSetpoint = 500;
  private RelativeEncoder shooter_encoder;

  private final CANSparkMax f_rightmotor = new CANSparkMax(front_right_motor, MotorType.kBrushless);
  private final CANSparkMax f_leftmotor = new CANSparkMax(front_left_motor, MotorType.kBrushless);

  private final CANSparkMax r_rightmotor = new CANSparkMax(back_right_motor, MotorType.kBrushless);
  private final CANSparkMax r_leftmotor = new CANSparkMax(back_left_motor, MotorType.kBrushless);

  private SlewRateLimiter groundMotorX = new SlewRateLimiter(0.7);
  private SlewRateLimiter groundMotorY = new SlewRateLimiter(0.7);
  private SlewRateLimiter groundMotorZ = new SlewRateLimiter(0.7);

  private RelativeEncoder fe_rightmotor;
  private RelativeEncoder fe_leftmotor;
  private RelativeEncoder re_rightmotor;
  private RelativeEncoder re_leftmotor;

  // private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  private final MecanumDrive m_robotDrive = new MecanumDrive(f_leftmotor, r_leftmotor, f_rightmotor, r_rightmotor);
  

  private final PS4Controller joystick = new PS4Controller(0);

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private boolean enableRobot = false;

  @Override
  public void robotInit() {

    r_rightmotor.setInverted(true);
    f_rightmotor.setInverted(true);

    fe_rightmotor = f_rightmotor.getEncoder();
    // f_rightmotor.encoder
    fe_leftmotor = f_leftmotor.getEncoder();
    re_rightmotor = r_rightmotor.getEncoder();
    re_leftmotor = r_leftmotor.getEncoder();

    // f_leftmotor.encoder
    // fe_rightmotor.setEnco

    fe_rightmotor.setPositionConversionFactor((1/10) * 0.2 * Math.PI);

    intake_1.setInverted(true);

    // initialise PID for shooter sparkmax 
    // pid_shooter = shooter.getPIDController();
    shooter_encoder = shooter.getEncoder();


    // PID coefficients
    kP = 0.01; 
    kI = 0;
    kD = 0; 
    shooterSetpoint = 500;
    m_shooterController.setP(kP);
    m_shooterController.setI(kI);
    m_shooterController.setD(kD);


    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("setpoint", shooterSetpoint);


  }

  @Override
  public void teleopPeriodic() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double shooterpoint = SmartDashboard.getNumber("setpoint", 0);

    if((p != kP)) { m_shooterController.setP(p); kP = p; }
    if((i != kI)) { m_shooterController.setI(i); kI = i; }
    if((d != kD)) { m_shooterController.setD(d); kD = d; }
    if (shooterSetpoint != shooterpoint) {
      shooterSetpoint = shooterpoint;
    }


    double value = m_shooterController.calculate(shooter_encoder.getVelocity(), shooterSetpoint);
    shooter.set(value);

    
    SmartDashboard.putNumber("shooter.pidcalculated", value);

    SmartDashboard.putNumber("shooter_encoder.velocityrpm", shooter_encoder.getVelocity());


    // Timer.getFPGATimestamp()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    if (joystick.getOptionsButtonPressed()) { 
      enableRobot = !enableRobot;
    }

    if (joystick.getL1Button()) {

      intake_1.set(ControlMode.PercentOutput, 1);

    } else if(joystick.getR1Button()) {
      intake_1.set(ControlMode.PercentOutput, -1);
    } else {
      intake_1.set(ControlMode.PercentOutput, 0);

    }
      
      double ySpeed = groundMotorX.calculate(-joystick.getLeftY());
      double xSpeed = groundMotorY.calculate(joystick.getLeftX());
      double zSpeed = groundMotorZ.calculate(joystick.getRightX());
 
      m_robotDrive.driveCartesian(ySpeed, xSpeed, zSpeed);




    SmartDashboard.putNumber("SetEncoderRightMotor", fe_rightmotor.getPosition());
    SmartDashboard.putNumber("getRightTriggerAxis", joystick.getR2Axis());

    // SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
   



  }

  @Override
  public void autonomousPeriodic() {
    // TODO Auto-generated method stub
    // super.autonomousPeriodic();

  }
}
