package frc.robot.subsystems.climber;
import frc.robot.subsystems.climber.climberIO;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOSpark implements climberIO {
    private final SparkMax climbSparkMax;
    private final AbsoluteEncoder climbEncoder;
public ClimberIOSpark(){
    climbSparkMax = new SparkMax(9,MotorType.kBrushless);
    
    var ClimbConfig = new SparkMaxConfig();
    ClimbConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(Current_Limit).voltageCompensation(2);
    ClimbConfig.signals.appliedOutputPeriodMs(10).busVoltagePeriodMs(5).outputCurrentPeriodMs(5);
    tryUntil10k(
        climbSparkMax,
        4,()-> climbMotor.configure( ClimbConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters )
    );


}
  @Override
  public Void runVolts(Voltage volts){
    climbSparkMax.setVoltage(volts);
  }
}
