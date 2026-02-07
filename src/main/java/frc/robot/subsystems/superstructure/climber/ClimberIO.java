package frc.robot.subsystems.superstructure.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;


public class ClimberIO {
    private final SparkMax hookMotor = new SparkMax(ClimberConstants.HOOK_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax telescopeMotor = new SparkMax(ClimberConstants.TELESCOPE_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder hookEncoder = hookMotor.getEncoder();
    private final RelativeEncoder telescopeEncoder = telescopeMotor.getEncoder();

    public ClimberIO() {
        SparkMaxConfig hookConfig = new SparkMaxConfig();
        hookConfig.smartCurrentLimit(ClimberConstants.HOOK_CURRENT_LIMIT);
        SparkUtil.tryUntilOk(hookMotor,5,()->hookMotor.configure(hookConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters));

        SparkMaxConfig telescopeConfig = new SparkMaxConfig();
        telescopeConfig.smartCurrentLimit(ClimberConstants.TELESCOPE_CURRENT_LIMIT);
        SparkUtil.tryUntilOk(telescopeMotor, 5, ()->telescopeMotor.configure(hookConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
    public void setHookSpeed(double speed){
        hookMotor.set(ClimberConstants.HOOK_SPEED);
    }
    public void stopHook(){
        hookMotor.stopMotor();
    }
    public double getHookPosition(){
        return hookEncoder.getPosition();
    }
    public void setTelescopeSpeed(double speed){
        telescopeMotor.set(ClimberConstants.TELESCOPE_SPEED);
    }
    public void stopTelescope(){
        telescopeMotor.stopMotor();
    }
    public double getTelescopePosition(){
        return telescopeEncoder.getPosition();
    }
}
