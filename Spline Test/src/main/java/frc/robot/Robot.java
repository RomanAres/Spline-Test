/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Auto.Timer;
import frc.robot.Auto.PathPlanner;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> chooser = new SendableChooser<>();

  private DifferentialDrive diffDrive;

  private Joystick driveController;
  
  private CANSparkMax leftFront;
  private CANSparkMax leftBack;
  private CANSparkMax rightFront;
  private CANSparkMax rightBack;

  private Timer timer = new Timer();

  private static double accumulator = 0.0;
  private static int counter = 0;

  private boolean isAutoFinished = false; // auto flag for spline
  
  private double totalTime = 15; //seconds
  private double timeStep = 0.1; //period of control loop on Rio, seconds
  private double robotTrackWidth = 2; //distance between left and right wheels, feet

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    rightFront = new CANSparkMax(2, MotorType.kBrushless);
    rightBack = new CANSparkMax(6, MotorType.kBrushless);
    leftFront = new CANSparkMax(3, MotorType.kBrushless);
    leftBack = new CANSparkMax(4, MotorType.kBrushless);

    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

    // rightFront.setInverted(true);
    // leftFront.setInverted(true);

    diffDrive = new DifferentialDrive(leftFront, rightFront);
    diffDrive.setSafetyEnabled(false);

    driveController = new Joystick(0);

    chooser.setDefaultOption("Default Auto", kDefaultAuto);
    chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //m_autoSelected = chooser.getSelected();

    isAutoFinished = false;

    double[][] waypoints = new double[][] {
        {0,1},
        {1,3},
        {0,5},
        {1,7}
    };
    
    final PathPlanner path = new PathPlanner(waypoints);
    path.calculate(totalTime, timeStep, robotTrackWidth);
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    Robot.accumulator += timer.getDT();
      //                                       some arbitrary number close to 0
    if (!isAutoFinished && (accumulator - timeStep <= 0.005)) {

      diffDrive.tankDrive(PathPlanner.smoothLeftVelocity[counter][1], PathPlanner.smoothRightVelocity[counter][1]);
      System.out.println(PathPlanner.smoothLeftVelocity[counter][1] + "  " + PathPlanner.smoothLeftVelocity[counter][1]);

      Robot.counter++;
      accumulator = 0.0;

      if (counter == PathPlanner.smoothLeftVelocity.length) {
        isAutoFinished = true;
      }
    } 
    
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    diffDrive.arcadeDrive(-driveController.getRawAxis(1), driveController.getRawAxis(4));

  
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
