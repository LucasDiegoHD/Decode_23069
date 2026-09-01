package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.templates.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.LEDSubsystem;

public class LedCommand extends CommandBase {
    private final LEDSubsystem ledSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    private int lastBolas = -1;

    public LedCommand(LEDSubsystem ledSubsystem, IndexerSubsystem indexerSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(ledSubsystem);
    }
    @Override
    public void initialize() {
        lastBolas = -1;
    }

    @Override
    public void execute() {
        int bolas = indexerSubsystem.getPieceCount();

        if (bolas != lastBolas) {
            if (bolas == 0) {
                ledSubsystem.setPattern(LEDSubsystem.OFF);
            }
            else if (bolas == 1) {
                ledSubsystem.setPattern(LEDSubsystem.BLUE);
            }
            else if (bolas == 2) {
                ledSubsystem.setPattern(LEDSubsystem.ORANGE);
            }
            else if (bolas >= 3) {
                ledSubsystem.setPattern(LEDSubsystem.GREEN);
            }

            lastBolas = bolas;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}