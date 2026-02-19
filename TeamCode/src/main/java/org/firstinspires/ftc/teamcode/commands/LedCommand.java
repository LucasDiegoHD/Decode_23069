package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

public class LedCommand extends CommandBase {
    private final LEDSubsystem ledSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    public LedCommand(LEDSubsystem ledSubsystem, IndexerSubsystem indexerSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        int bolas = indexerSubsystem.getPieceCount();

        if (bolas == 0) {
            ledSubsystem.setPattern(LEDSubsystem.OFF);
        }
        else if (bolas == 1) {
            ledSubsystem.setPattern(LEDSubsystem.RED);
        }
        else if (bolas == 2) {
            ledSubsystem.setPattern(LEDSubsystem.YELLOW);
        }
        else if (bolas >= 3) {
            ledSubsystem.setPattern(LEDSubsystem.GREEN);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}