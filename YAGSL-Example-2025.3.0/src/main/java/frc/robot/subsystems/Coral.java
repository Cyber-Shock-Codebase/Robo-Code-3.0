package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
// import au.grapplerobotics.ConfigurationFailedException;
// import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.Shooter;
// import frc.robot.simulation.SimulatableCANSparkMax;
// import frc.robot.subsystems.leds.LEDs;

public class Coral extends SubsystemBase {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Coral mInstance;
  private PeriodicIO mPeriodicIO;
  // public final LEDs m_leds = LEDs.getInstance();

  public static Coral getInstance() {
    if (mInstance == null) {
      mInstance = new Coral();
    }
    return mInstance;
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    REVERSE,
    INDEX,
    READY,
    SCORE
  }

  // private ThriftyNova mLeftMotor;
  // private ThriftyNova mRightMotor;
   private SparkMax mLeftMotor;
   private SparkMax mRightMotor;
   private DigitalInput ForBeam;
   private DigitalInput BackBeam;
  // private LaserCan mLaserCAN;

  public intakeCoral intakeCoral;
  public scoreL1 scoreL1;
  public scoreL24 scoreL24;

  public Coral() {
    super("Coral");
    intakeCoral = new intakeCoral();

    mPeriodicIO = new PeriodicIO();

    mLeftMotor = new SparkMax(Shooter.LeftMotorId, MotorType.kBrushless);
    mRightMotor = new SparkMax(Shooter.RightMotorId, MotorType.kBrushless);
    ForBeam = new DigitalInput(Shooter.ForBeamID);
    BackBeam = new DigitalInput(Shooter.BackBeamID);

    SparkMaxConfig coralConfig = new SparkMaxConfig();

    coralConfig.idleMode(IdleMode.kBrake);

    mLeftMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    mRightMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // mLaserCAN = new LaserCan(Constants.Coral.kLaserId);
    // try {
    //   mLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
    //   mLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    //   mLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    // } catch (ConfigurationFailedException e) {
    //   System.out.println("Configuration failed! " + e);
    // }
  }

  private static class PeriodicIO {
    double rpm = 0.0;
    double speed_diff = 0.0;

    int index_debounce = 0;

    // LaserCan.Measurement measurement;

    IntakeState state = IntakeState.NONE;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    // mPeriodicIO.measurement = mLaserCAN.getMeasurement();

    checkAutoTasks();
  }

  
  public void writePeriodicOutputs() {
    mLeftMotor.set(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    mRightMotor.set(-mPeriodicIO.rpm);
  }

  
  public void stop() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  
  public void outputTelemetry() {
    SmartDashboard.putNumber("RPM/target", mPeriodicIO.rpm);

    // LaserCan.Measurement measurement = mPeriodicIO.measurement;
    // if (measurement != null) {
    //   putNumber("Laser/distance", measurement.distance_mm);
    //   putNumber("Laser/ambient", measurement.ambient);
    //   putNumber("Laser/budget_ms", measurement.budget_ms);
    //   putNumber("Laser/status", measurement.status);

    //   putBoolean("Laser/hasCoral", isHoldingCoral());
    // }
  }

  
  public void reset() {
    stopCoral();
  }
public class intakeCoral extends CommandBase {
  public  intakeCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Coral.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       // Coral.getInstance().intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Coral.getInstance().intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Coral.getInstance().stopCoral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Coral.getInstance().isHoldingCoral();
  }
}
public class scoreL1 extends Command {
  public scoreL1() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Coral.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       // Coral.getInstance().intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Coral.getInstance().scoreL1();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.isFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
public class scoreL24 extends Command {
  public scoreL24() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Coral.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       // Coral.getInstance().intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Coral.getInstance().scoreL24();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.isFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public boolean isHoldingCoral() {
    return !ForBeam.get();
  }

  public boolean isCoralproblematic() {
    return !BackBeam.get();
  }

  // Returns true if the forbream detects a coral and the backbeam does not. 
  // If the backbeam is blocked, the coral is not ready because it is not far enough in the shooter.
  /*
  Returns the following boolean values:
  true - Coral is ready (forbeam detects a coral and backbeam does not)
  false - Coral is not ready (forbeam does not detect a coral, but backbeam does. Coral is to far in the shooter) 
       OR (forbeam and backbeam both detect a coral)   
  */ 
  public boolean isCoralReady() {
    if(!ForBeam.get() && BackBeam.get()) {
      mPeriodicIO.state = IntakeState.READY;
      return true;
    }
    else {
      return false;
    }
   // return ForBeam.get() && !BackBeam.get();
  }
  public void setSpeed(double rpm) {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = rpm;
  }

  public void intake() {
    if(isCoralReady()) {
      return; // we don't need intake if the coral is ready
    }
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Shooter.IntakeSpeed;
    mPeriodicIO.state = IntakeState.INTAKE;
   
    // m_leds.setColor(Color.kYellow);
  }

  public void reverse() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Shooter.ReverseSpeed;
    mPeriodicIO.state = IntakeState.REVERSE;
  }

  public void index() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Shooter.IndexSpeed;
    mPeriodicIO.state = IntakeState.INDEX;

    // m_leds.setColor(Color.kBlue);
  }

  public void scoreL1() {
    if(!isCoralReady()) {
      return; // we can't score if the coral is not ready
    }
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Shooter.L1Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL24() {
    if(!isCoralReady()) {
      return; // we can't score if the coral is not ready
    }
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Shooter.L24Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void stopCoral() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void checkAutoTasks() {
    switch (mPeriodicIO.state) {
      case INTAKE:
        if (isHoldingCoral()) {
          mPeriodicIO.index_debounce++;

          if (mPeriodicIO.index_debounce > 10) {
            mPeriodicIO.index_debounce = 0;
            index();
          }
        }
        break;
      case INDEX:
        if (!isHoldingCoral()) {
          stopCoral();

          mPeriodicIO.state = IntakeState.READY;
          // m_leds.setColor(Color.kBlue);
        }
        break;
      default:
        break;
    }
  }
}
