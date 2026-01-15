package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;


public class LedCommand extends CommandBase {

    private final LEDSubsystem ledSubsystem;
    private final AllianceEnum alliance;


    public LedCommand(LEDSubsystem ledSubsystem, AllianceEnum alliance) {
        this.ledSubsystem = ledSubsystem;
        this.alliance = DataStorage.alliance;
        addRequirements(ledSubsystem);
    }

    public void initialize() {
    }
    @Override
    public void execute() {
        if (alliance == AllianceEnum.Red) {
            ledSubsystem.setPattern(LEDSubsystem.RED);
        } else {
            ledSubsystem.setPattern(LEDSubsystem.BLUE);
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
