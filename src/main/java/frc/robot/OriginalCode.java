package frc.robot;

// note: refactor the code

//main imports 
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

// motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import edu.wpi.first.wpilibj.PWMMotorController.addFollower;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//pneumatics
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

//joysticks
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;

//encoders
/*import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;*/


//gyroscope
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class OriganalCode extends TimedRobot {

  //Creating varibales for the motor controllers
  PWMVictorSPX driveLeftA = new PWMVictorSPX(0);
  PWMVictorSPX driveLeftB = new PWMVictorSPX(1);
  // left motor controllers
  MotorControllerGroup leftMotors = new MotorControllerGroup(driveLeftA, driveLeftB);

  PWMVictorSPX driveRightA = new PWMVictorSPX(8);
  PWMVictorSPX  driveRightB = new PWMVictorSPX(9);  // right motor controllers
  MotorControllerGroup rightMotors = new MotorControllerGroup(driveRightA, driveRightB);

  //differential drive
  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  // variables for the arm controls
  CANSparkMax armYAxis = new CANSparkMax(11, MotorType.kBrushless);
  WPI_TalonSRX armXAxis = new WPI_TalonSRX(3);

  //variables for the pneumatics system
  Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  DoubleSolenoid solenoid = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);

  //PneumaticsControlModule controlModule = new PneumaticsControlModule(1);

  // joysticks
  Joystick driverController = new Joystick(2);
  XboxController armController = new XboxController(0);

  //initialize the encoder
  //private RelativeEncoder yAxisEncoder;

  //initialize the gyrscope
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  //Constants for controlling the arm. needs adjustments for this robot

  //current limit for the arm
  static final int ArmCurrentLimitA = 20;

  //Arm power output for y axis
  static final double ArmYOutputPower = 0.6;

  // Arm power output for x axis
  static final double ArmXOutputPower = 0.65;

  //Varibles needed for the code
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0; 

  double autoStart = 0;
  boolean goForAuto = true;

  //gyroscope constants
  double error;
  double kp = 0.5;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
   @Override

  //function for setting the initial conditions of all the hardware
  public void robotInit() {

    //initial conditions for the drive motors
    leftMotors.setInverted(true);
    rightMotors.setInverted(false);
    
    //initla conditions for the arm
    armYAxis.setInverted(true);
    armYAxis.setIdleMode(IdleMode.kBrake);
    armYAxis.setSmartCurrentLimit(ArmCurrentLimitA);
    ((CANSparkMax) armYAxis).burnFlash();
    armXAxis.setInverted(false);
    armXAxis.setNeutralMode(NeutralMode.Brake);

    //initial conditions for the intake
    compressor.enableDigital();
    solenoid.set(Value.kForward);

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);

    //encoders
    /*yAxisEncoder = armYAxis.getEncoder(Type.kHallSensor, 4096);
    yAxisEncoder.setPosition(0);*/

    // limit the direction of the arm's rotations (kReverse = up)
    armYAxis.enableSoftLimit(SoftLimitDirection.kForward, false);
    armYAxis.enableSoftLimit(SoftLimitDirection.kReverse, false);

    /*armYAxis.setSoftLimit(SoftLimitDirection.kForward, 0);
    armYAxis.setSoftLimit(SoftLimitDirection.kReverse, 0);*/

    //recalibrate the gyro every time the robotis turned on
    gyro.calibrate();

  }

  /**
   * *set the arm output power. Positive is out, negative is in
   * 
   * @param percent
   */

    //function to set the arm output power in the vertical direction
  public void setArmYAxisMotor(double percent) {
    armYAxis.set(percent);
    SmartDashboard.putNumber("armYAxis power(%)", percent);
  }

  //function to set the arm output power in the horizontal direction
  public void setArmXAxisMotor(double percent) {
    armXAxis.set(percent);
    SmartDashboard.putNumber("armXaxis power(%)", percent);
  }
  
 /**
  * set the arm output power.
  *
  * @param percent desired speed
  * @param amps current limit
  */
  
  //function for starting autonomous
  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
    goForAuto = true;

    //reset the gyro to 0
    gyro.reset();
  }

  //function that is called periodically during autonomous
  @Override
  public void autonomousPeriodic() {
    //read the gyro value
    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    /*Shuffleboard.getTab("SmartDashboard")
      .add("Gyro Angle", gyro.getAngle())
      .withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("min", -360, "max", 360))
      .getEntry();*/

    // define the error
    error = 0 - gyro.getAngle();

    //get time since start of auto then run drive code for autonomous
    double autoTimeElapsed =  Timer.getFPGATimestamp() - autoStart;
    if(goForAuto){
      //drop the cargo
      if ((autoTimeElapsed > 8) && (autoTimeElapsed < 11)){
        solenoid.set(Value.kReverse);

      /*} else if((autoTimeElapsed > 12) && (autoTimeElapsed < 13)){
        //drive backwards onto the charging station (to dock: deltaT=1.775 *1.6 worked during the practice matches)
        leftMotors.set(-0.60);
        rightMotors.set(-0.60);*/
        /*if (error != 0){
          double speed = error*kp;
          leftMotors.set(speed);
          rightMotors.set(speed);
        }*/

      } else {
        //do nothing for the rest of auto
        leftMotors.set(0);
        rightMotors.set(0);
      }
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //compressor.enableDigital();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // enable the gyro - for testing
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());

    //Set up arcade steer
    double forward = -driverController.getRawAxis(1);
    double turn = driverController.getRawAxis(4);
    
    // set up arcade drive
    drive.arcadeDrive(forward, turn);
    
    //Code for the arm
    double armPower;

    // motion for the arm in the vertical direction
    if (armController.getLeftY() > 0.5) {
      //raise the arm
      armPower = ArmYOutputPower;
      //*armController.getLeftY();
    }
    else if (armController.getLeftY() < -0.5) {
      //lower the arm
      armPower = -ArmYOutputPower;
      //*armController.getLeftY();
    }
    else {
      //do nothing and let it sit where it is
      armPower = 0.0;
      armYAxis.setIdleMode(IdleMode. kBrake);
    }
    setArmYAxisMotor(armPower);
    
    // motion for the arm in the horizontal direction
    if (armController.getLeftTriggerAxis() > 0.5) {
      //extend the arm
      // we could set it to srmpower = armXOuptuPower x get left trigger axis ( test it on the pivot firs)
      armPower = ArmXOutputPower;
      //*armController.getLeftTriggerAxis();
    }
    else if (armController.getRightTriggerAxis() > 0.5) {
      //retract the arm
      armPower = -ArmXOutputPower;
      //*armController.getRightTriggerAxis();
    }
    else {
      // do nothing and let it sit where it is
      armPower = 0.0;
      //armXAxis.stopMotor();
      armXAxis.setNeutralMode(NeutralMode.Brake);
    }
    setArmXAxisMotor(armPower);

    //Intake controls

    //solenoid controls
    if(armController.getLeftBumperPressed()){

      //fire the air one way
      solenoid.set(Value.kForward);
      
    } else if(armController.getRightBumperPressed()){

      //fire the air the other way
      solenoid.set(Value.kReverse);
    }

   }

  //function for disabling everything at the end of the game
  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    armYAxis.set(0);
    armXAxis.set(0);
  }
}
