// package frc.robot.subsystems.superstructure.intake;


// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.KilogramSquareMeters;
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.Volts;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.MOTOR_GEAR_RATIO;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.ROLLER_MOTOR_TYPE;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_MAX_DISTANCE;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_MOTOR_TYPE;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KA;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KD;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KG;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KI;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KP;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KS;
// import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KV;

// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.trajectory.ExponentialProfile;
// import edu.wpi.first.math.trajectory.ExponentialProfile.State;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.simulation.ElevatorSim;
// import frc.robot.util.simUtils.SimMotorConfigs;
// import frc.robot.util.simUtils.SimulatedMotor;

// public class IntakeIOSim implements IntakeIO {

//     LinearSystem<N2, N1, N2> slideSystem = LinearSystemId.createElevatorSystem(SLIDE_MOTOR_TYPE, 0, 0, 0);
//     SimulatedMotor slideMotorSim = new SimulatedMotor(
//         new SimMotorConfigs(SLIDE_MOTOR_TYPE, MOTOR_GEAR_RATIO, KilogramSquareMeters.of(0.002), Volts.of(0.2)));
//     SimulatedMotor rollerMotorSim = new SimulatedMotor(
//         new SimMotorConfigs(ROLLER_MOTOR_TYPE, 2, KilogramSquareMeters.of(0.002), Volts.of(0.2)));
//     private PIDController slidePID = new PIDController(SLIDE_POSITION_KP, SLIDE_POSITION_KI, SLIDE_POSITION_KD);
//     private ElevatorFeedforward slideFeedforward = new ElevatorFeedforward(
//         SLIDE_POSITION_KS, SLIDE_POSITION_KG, SLIDE_POSITION_KV, SLIDE_POSITION_KA);
//     private ExponentialProfile slideProfile = new ExponentialProfile(
//         ExponentialProfile.Constraints.fromCharacteristics(12, SLIDE_POSITION_KV, SLIDE_POSITION_KA));

//     ElevatorSim slideSim = new ElevatorSim(slideSystem, SLIDE_MOTOR_TYPE, 0, SLIDE_MAX_DISTANCE.in(Meters), false, 0);


//     /** Instantiates the Real Intake hardware. */
//     public IntakeIOSim() {

//     }

//     @Override
//     public void updateInputs(IntakeIOInputs inputs) {
//         inputs.slidePosition.mut_replace(Meters.of(slideSim.getPositionMeters()));
//         inputs.rollerVelocity.mut_replace(
//             slideMotorSim.getEncoderVelocity());
//         inputs.slideAppliedVoltage.mut_replace(slideMotorSim.getAppliedVoltage());

//     }

//     @Override
//     public void setSlideVoltage(Voltage voltage) {
//         slideMotorSim.useSimpleDCMotorController().requestVoltage(voltage);
//     }

//     @Override
//     public void setRollerVoltage(Voltage voltage) {
//         rollerMotorSim.useSimpleDCMotorController().requestVoltage(voltage);
//     }

//     @Override
//     public void setPosition(Distance position) {
//         State slideProfileState = slideProfile.calculate(0.02,
//             new State(slideSim.getPositionMeters(), slideSim.getVelocityMetersPerSecond()),
//             new State(position.in(Meters), 0));
//         double output = slidePID.calculate(slideSim.getPositionMeters(), position.in(Meters));
//         output += slideFeedforward.calculateWithVelocities(
//             slideSim.getVelocityMetersPerSecond(), slideProfileState.velocity);
//         slideMotorSim.useSimpleDCMotorController().requestVoltage(Volts.of(output));
//     }
    
//     @Override
//     public void stopMotors() {
//         slideMotorSim.useSimpleDCMotorController().requestVoltage(Volts.of(0));
//         rollerMotorSim.useSimpleDCMotorController().requestVoltage(Volts.of(0));
//     }

//     @Override
//     public void resetSlideEncoder(Distance position) {
        
//     }

// }
