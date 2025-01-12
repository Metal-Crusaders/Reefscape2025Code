package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class TestPathAuto extends SequentialCommandGroup {

    public TestPathAuto(CommandSwerveDrivetrain swerve) {

        PathPlannerAuto testAuto = new PathPlannerAuto("TestPathAuto");

        addRequirements(swerve);

        addCommands(testAuto);

    }

}