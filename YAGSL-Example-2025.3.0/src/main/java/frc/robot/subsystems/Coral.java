package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// import au.grapplerobotics.ConfigurationFailedException;
// import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private Coral() {
    super("Coral");

    mPeriodicIO = new PeriodicIO();

    mLeftMotor = new SparkMax(Shooter.LeftMotorId, MotorType.kBrushless);
    mRightMotor = new SparkMax(Shooter.RightMotorId, MotorType.kBrushless);
    ForBeam = new DigitalInput(0);
    BackBeam = new DigitalInput(1);

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

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public boolean isHoldingCoral() {
    return ForBeam.get();
  }

  public void setSpeed(double rpm) {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = rpm;
  }

  public void intake() {
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
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Shooter.L1Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL24() {
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
