package frc.robot.subsystems.superstructure.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;


public class ClimberIO {

    public static class ClimberIOInputs
    {
        
    }

    public void updateInputs(ClimberIOInputs inputs)
    {
        // Default to nothing for replay
    }

    public void setHookSpeed(double speed){
        // Default to nothing for replay
    }
    public void stopHook(){
        // Default to nothing for replay
    }
    public double getHookPosition(){
        // Default to nothing for replay
    }
    public void setTelescopeSpeed(double speed){
        // Default to nothing for replay
    }
    public void stopTelescope(){
        // Default to nothing for replay
    }
    public double getTelescopePosition(){
        // Default to nothing for replay
    }
}
