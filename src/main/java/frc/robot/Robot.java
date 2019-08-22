/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Constants
  private static final int ROTATION_ID = 2;
  private static final int DRIVE_ID = 1;
  private static final int GAMEPAD_ID = 0;
  private static final int DRIVE_MAX_VELOCITY = 5767;

  private CANSparkMax m_rotation;
  private CANSparkMax m_drive;
  private CANEncoder m_rotationEncoder;
  private CANEncoder m_driveEncoder;
  private CANPIDController m_rotationPositionController;
  private CANPIDController m_driveVelocityController;


  public double rotation_kP, rotation_kI, rotation_kD, rotation_kIz, rotation_kFF, rotation_kMaxOutput, rotation_kMinOutput;
  public double drive_kP, drive_kI, drive_kD, drive_kIz, drive_kFF, drive_kMaxOutput, drive_kMinOutput;

  private Joystick GamepadController;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // Initializing Rotation Motor
    m_rotation = new CANSparkMax(ROTATION_ID, MotorType.kBrushless);
    m_rotation.restoreFactoryDefaults();
    m_rotationPositionController = m_rotation.getPIDController();
    m_rotationEncoder = m_rotation.getEncoder();
    rotation_kP = 0.1;
    rotation_kI = 0; //1e-4
    rotation_kD = 0; //1
    rotation_kIz = 0;
    rotation_kFF = 0;
    rotation_kMaxOutput = 0.5;
    rotation_kMinOutput = -0.5;

    m_rotationPositionController.setP(rotation_kP);
    m_rotationPositionController.setI(rotation_kI);
    m_rotationPositionController.setD(rotation_kD);
    m_rotationPositionController.setIZone(rotation_kIz);
    m_rotationPositionController.setFF(rotation_kFF);
    m_rotationPositionController.setOutputRange(rotation_kMinOutput, rotation_kMaxOutput);
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Rotation P Gain", rotation_kP);
    SmartDashboard.putNumber("Rotation I Gain", rotation_kI);
    SmartDashboard.putNumber("Rotation D Gain", rotation_kD);
    SmartDashboard.putNumber("Rotation I Zone", rotation_kIz);
    SmartDashboard.putNumber("Rotation Feed Forward", rotation_kFF);
    SmartDashboard.putNumber("Rotation Max Output", rotation_kMaxOutput);
    SmartDashboard.putNumber("Rotation Min Output", rotation_kMinOutput);

    // Initializing Drive Motor
    m_drive = new CANSparkMax(DRIVE_ID, MotorType.kBrushless);
    m_drive.restoreFactoryDefaults();
    m_driveVelocityController = m_drive.getPIDController();
    m_driveEncoder = m_drive.getEncoder();
    drive_kP = 1.0E-5;
    drive_kI = 1.0E-6;
    drive_kD = 1.0E-5;
    drive_kIz = 100;
    drive_kFF = 1.79e-4;
    drive_kMaxOutput = 1;
    drive_kMinOutput = -1;

    m_driveVelocityController.setP(drive_kP);
    m_driveVelocityController.setI(drive_kI);
    m_driveVelocityController.setD(drive_kD);
    m_driveVelocityController.setIZone(drive_kIz);
    m_driveVelocityController.setFF(drive_kFF);
    m_driveVelocityController.setOutputRange(drive_kMinOutput, drive_kMaxOutput);
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Drive P Gain", drive_kP);
    SmartDashboard.putNumber("Drive I Gain", drive_kI);
    SmartDashboard.putNumber("Drive D Gain", drive_kD);
    SmartDashboard.putNumber("Drive I Zone", drive_kIz);
    SmartDashboard.putNumber("Drive Feed Forward", drive_kFF);
    SmartDashboard.putNumber("Drive Max Output", drive_kMaxOutput);
    SmartDashboard.putNumber("Drive Min Output", drive_kMinOutput);

    
    GamepadController = new Joystick(GAMEPAD_ID);
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
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // read PID coefficients from SmartDashboard
    double rp = SmartDashboard.getNumber("Rotation P Gain", 0);
    double ri = SmartDashboard.getNumber("Rotation I Gain", 0);
    double rd = SmartDashboard.getNumber("Rotation D Gain", 0);
    double riz = SmartDashboard.getNumber("Rotation I Zone", 0);
    double rff = SmartDashboard.getNumber("Rotation Feed Forward", 0);
    double rmax = SmartDashboard.getNumber("Rotation Max Output", 0);
    double rmin = SmartDashboard.getNumber("Rotation Min Output", 0);
    double dp = SmartDashboard.getNumber("Drive P Gain", 0);
    double di = SmartDashboard.getNumber("Drive I Gain", 0);
    double dd = SmartDashboard.getNumber("Drive D Gain", 0);
    double diz = SmartDashboard.getNumber("Drive I Zone", 0);
    double dff = SmartDashboard.getNumber("Drive Feed Forward", 0);
    double dmax = SmartDashboard.getNumber("Drive Max Output", 0);
    double dmin = SmartDashboard.getNumber("Drive Min Output", 0);

    double rotations = Math.toDegrees(Math.atan2(GamepadController.getRawAxis(4), -GamepadController.getRawAxis(5))) / 20;
    double magnitude = Math.sqrt(Math.pow(GamepadController.getRawAxis(4), 2) + Math.pow(GamepadController.getRawAxis(5), 2));
    double targetVel = (double) magnitude * DRIVE_MAX_VELOCITY; // Target velocity of drive motor in RPM

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((rp != rotation_kP)) { m_rotationPositionController.setP(rp); rotation_kP = rp; }
    if((ri != rotation_kI)) { m_rotationPositionController.setI(ri); rotation_kI = ri; }
    if((rd != rotation_kD)) { m_rotationPositionController.setD(rd); rotation_kD = rd; }
    if((riz != rotation_kIz)) { m_rotationPositionController.setIZone(riz); rotation_kIz = riz; }
    if((rff != rotation_kFF)) { m_rotationPositionController.setFF(rff); rotation_kFF = rff; }
    if((rmax != rotation_kMaxOutput) || (rmin != rotation_kMinOutput)) { 
      m_rotationPositionController.setOutputRange(rmin, rmax); 
      rotation_kMinOutput = rmin; rotation_kMaxOutput = rmax; 
    }
    if((dp != drive_kP)) { m_driveVelocityController.setP(dp); drive_kP = dp; }
    if((di != drive_kI)) { m_driveVelocityController.setI(di); drive_kI = di; }
    if((dd != drive_kD)) { m_driveVelocityController.setD(dd); drive_kD = dd; }
    if((diz != drive_kIz)) { m_driveVelocityController.setIZone(diz); drive_kIz = diz; }
    if((dff != drive_kFF)) { m_driveVelocityController.setFF(dff); drive_kFF = dff; }
    if((dmax != drive_kMaxOutput) || (dmin != drive_kMinOutput)) { 
      m_driveVelocityController.setOutputRange(dmin, dmax); 
      drive_kMinOutput = dmin; drive_kMaxOutput = dmax; 
    }

    if(magnitude > 0.05) {
      m_rotationPositionController.setReference(rotations, ControlType.kPosition);
      m_driveVelocityController.setReference(targetVel, ControlType.kVelocity);
    } else {
      m_rotation.set(0);
      m_drive.set(0);
    }
    
    SmartDashboard.putNumber("Position SetPoint", rotations);
    SmartDashboard.putNumber("Velocity SetPoint", targetVel);
    SmartDashboard.putNumber("Rotation Position", m_rotationEncoder.getPosition());
    SmartDashboard.putNumber("Drive Velocity", m_driveEncoder.getVelocity());

    //m_rotation.set(GamepadController.getRawAxis(4));
    //m_drive.set(GamepadController.getRawAxis(1));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * For applying deadband to an input value
   * @return The value with deadband applied and output scaled to use full range
   */
  public double getWithDeadband(double value, double deadband) {
    return Math.abs(value) < Math.abs(deadband) ? 0 : value; /// (1 - deadband) - Integer.signum((int) value) * deadband / (1 - deadband);
  }
}
