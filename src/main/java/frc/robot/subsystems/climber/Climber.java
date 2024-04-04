package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.subsystems.climber.ClimberConstants;

public class Climber extends SubsystemBase {
    private ClimberIO io;
    private final ClimberIOInputs climberInputs = new ClimberIOInputs();

    private final TunableNumber testingMode = 
        new TunableNumber("Shooter/TestingMode", 0);
    private final TunableNumber climberVoltage = 
        new TunableNumber("Climber/Voltage", 0.0);
    
    public Climber(ClimberIO io) {
        this.io = io;

        if (testingMode.get() == 1) {
            ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
            tab.add(SUBSYSTEM_NAME, this);
        }
    }

    @Override
    public void periodic() {
        // empty for now
    }

    public void setClimberVoltage() {
        io.setClimberVoltage(climberVoltage.get());
    }

    public void setCurrentPositionAsZero() {
        io.setCurrentPositionAsZero();
    }
    
}
