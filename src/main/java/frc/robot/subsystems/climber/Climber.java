package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
    private ClimberIO io;
    private final ClimberIOInputs climberInputs = new ClimberIOInputs();

    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final TunableNumber testingMode = 
        new TunableNumber("Shooter/TestingMode", 0);
    private final TunableNumber climberVoltage = 
        new TunableNumber("Climber/Voltage", 0.0);

    private ClimberState climberState;

    private enum ClimberState {
        IDLE,
        EXTENDING,
        RETRACTING
    }
    
    public Climber(ClimberIO io) {
        this.io = io;

        if (testingMode.get() == 1) {
            ShuffleboardTab tab = Shuffleboard.getTab(ClimberConstants.SUBSYSTEM_NAME);
            tab.add(ClimberConstants.SUBSYSTEM_NAME, this);
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    

    public void setClimberVoltage(double voltage) {
        io.setClimberVoltage(voltage);
    }

    public void idleStateClimber(){
        io.setPosition(0);
    }

    public void extendingStateClimber(){;
        io.setPosition(ClimberConstants.EXTENDED_POSITION);
    }

    public void retractingStateClimber(){;
        io.setPosition(ClimberConstants.EXTENDED_POSITION);
    }


    public void runClimberStateMachine() {

        if (climberState == ClimberState.IDLE) {
           idleStateClimber();
        }
        else if (climberState == ClimberState.EXTENDING) {
            extendingStateClimber();
        }
        else if (climberState == ClimberState.RETRACTING) {
            retractingStateClimber();
        }
    }
    
}
