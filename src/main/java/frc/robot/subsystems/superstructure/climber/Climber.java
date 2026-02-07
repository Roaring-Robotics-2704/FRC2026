package frc.robot.subsystems.superstructure.climber;

public class Climber {
    private final ClimberIO io = new ClimberIO();
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
