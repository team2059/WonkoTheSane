// package org.team2059.Wonko.subsystems.algae;

// import org.team2059.Wonko.Constants.AlgaeCollectorConstants;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// public class AlgaeIntakeIOSim implements AlgaeCollectorIO {
//     private final LinearSystem<N2, N1, N2> tiltPlant = 
//         LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.003, 45);
    
        
//     private final LinearSystem<N2, N1, N2> intakePlant = 
//         LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.05, 3);

//     private final DCMotorSim intakeMotorSim1 = new DCMotorSim(intakePlant, DCMotor.getNeoVortex(2), 0.003);
//     private final DCMotorSim intakeMotorSim2 = new DCMotorSim(intakePlant, DCMotor.getNeoVortex(2), 0.003);

//     private final DCMotorSim tiltMotorSim = new DCMotorSim(tiltPlant, DCMotor.getNEO(1), 0.0003);

//     private boolean isGamepieceDetected = false;

//     private final PIDController tiltPidController = new PIDController(AlgaeCollectorConstants.kPAlgae, AlgaeCollectorConstants.kIAlgae, AlgaeCollectorConstants.kDAlgae);

//     @Override
//     public void updateInputs(AlgaeCollectorIOInputs inputs) {
//         intakeMotorSim1.update(0.02);
//         intakeMotorSim2.update(0.02);
//         tiltMotorSim.update(0.02);

        
//         inputs.motor1CurrentAmps = intakeMotorSim1.getCurrentDrawAmps();
//         inputs.motor1AppliedVolts = intakeMotorSim1.getInputVoltage();

        
//         inputs.motor1CurrentAmps = intakeMotorSim1.getCurrentDrawAmps();
//         inputs.motor1AppliedVolts = intakeMotorSim1.getInputVoltage();
        
//         inputs.tiltMotorAppliedVolts = tiltMotorSim.getInputVoltage();
//         isGamepieceDetected = intakeMotorSim1.getCurrentDrawAmps() > AlgaeCollectorConstants.stallDetectionAmps;
//         inputs.hasAlgae = isAlgae();
//     }

//     @Override
//     public void setIntakeSpeed(double speed) {
//         intakeMotorSim1.setInputVoltage(MathUtil.clamp(12 * speed, -12, 12));
//         intakeMotorSim2.setInputVoltage(MathUtil.clamp(12 * speed, -12, 12));
//     }

//     public boolean isAlgae() {
//         return isGamepieceDetected; 
//     } 
    
//     public void setTiltVoltage(double voltage) {
//         tiltMotorSim.setInput(MathUtil.clamp(voltage, -12, 12));
//     }

//     public void stopTilt() {
//         tiltMotorSim.setInputVoltage(0);
//     }

//     public void stopIntake() {
//         intakeMotorSim1.setInputVoltage(0);
//         intakeMotorSim2.setInputVoltage(0);
//     }

//     public void stopAll() {
//         stopTilt();
//         stopIntake();
//     }

//     @Override
//     public void setTiltSpeed(double speed) {
//         tiltMotorSim.setInputVoltage(MathUtil.clamp(12*speed, -12, 12));
//     }
// }