package frc.robot.commands.coroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class L3LeftPlacement extends SequentialCommandGroup {

    public L3LeftPlacement() {

        super();

        addRequirements();

        addCommands(
            // align
            // drive most of the way there
            // parallel command group of elevator up and pivot claw
            // parallel command group here with following actions: drive forward all the way (CAN BE DONE RAW OR WITH A DRIVE TO POSE PART), grab algae if it's there, score coral
        );

        // MOST DRIVING CAN BE DRIVE-TO-POSE DRIVING!

        // Put each of the six trajectories in a Constants class and have a helper method down here to select the right end pose

    }
    
}
