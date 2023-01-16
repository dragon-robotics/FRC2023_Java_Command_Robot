// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class AutoLoader {

    /**
     * Enumeration for the possible autos that we'll create
     */
    public enum AutoCommand {
        NONE, // Does Nothing
        COMMUNITY_EXIT, // Just leaves the tarmac without scoring
    }

    private SendableChooser<AutoCommand> m_autoChooser;

    /**
     * Constructor used to initialize the sendable chooser
     */
    public AutoLoader() {
        // Determine which alliance we're in to determine if we need to mirror and
        // invert the trajectories //

        // Initialize the sendable chooser //
        m_autoChooser = new SendableChooser<>();

        // Default option is to always have no auto command running //
        // m_autoChooser.setDefaultOption("None", AutoCommand.NONE);
        m_autoChooser.setDefaultOption("Community Zone Exit", AutoCommand.COMMUNITY_EXIT);

        // Initialize the rest of the options //
        SmartDashboard.putData(m_autoChooser);
    }

    /**
     * 
     * @return The selected command from the m_autoChooser
     */
    public AutoCommand getSelected() {
        return m_autoChooser.getSelected();
    }

}
