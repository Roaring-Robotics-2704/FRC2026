package frc.robot.subsystems.superstructure.climber;
import  edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {

    private final ClimberIO io = new ClimberIO();
public Climber(ClimberIO climberIO) {
        //TODO Auto-generated constructor stub
        private ClimberState currentstate = ClimberState.IN;


    }
    // rotate hooks parallel to ground
    public void rotateHooksDown(){
        if (io.getHookPosition() < ClimberConstants.HOOK_TARGET_DEGREES) {
            io.setHookSpeed(ClimberConstants.HOOK_SPEED);
        }
        else{
            io.stopHook();
        }
    }
// stop hooks
    public void stopHooks(){
        io.stopHook();
    }

//raise climber
    public void extendClimber(){
        if (io.getHookPosition() < ClimberConstants.EXTENDED_HEIGHT){
            io.setTelescopeSpeed(ClimberConstants.TELESCOPE_SPEED);
        }
        else{
            io.stopTelescope();
        }
    }

//bring climber back down
    public void retractClimber(){
        if (io.getHookPosition() > ClimberConstants.RETRACTED_HEIGHT){
            io.setTelescopeSpeed(-ClimberConstants.TELESCOPE_SPEED);
        }
        else{
            io.stopTelescope();
        }
    }
    public void stopTelescope(){
        io.stopTelescope();
    }

}
