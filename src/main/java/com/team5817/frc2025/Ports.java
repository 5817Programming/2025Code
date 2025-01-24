package com.team5817.frc2025;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
	/*
	 * LIST OF CHANNEL AND CAN IDS
	 *
	 * Swerve Modules go:
	 * 0 1
	 * 2 3
	 *
	 * spotless:off
	 */

	/* DRIVETRAIN CAN DEVICE IDS */
	public static final CanDeviceId FL_DRIVE = new CanDeviceId(0, "canivore1");
	public static final CanDeviceId FL_ROTATION = new CanDeviceId(4, "canivore1");
	public static final CanDeviceId FL_CANCODER = new CanDeviceId(8, "canivore1");

	public static final CanDeviceId FR_DRIVE = new CanDeviceId(1, "canivore1");
	public static final CanDeviceId FR_ROTATION = new CanDeviceId(5, "canivore1");
	public static final CanDeviceId FR_CANCODER = new CanDeviceId(9, "canivore1");

	public static final CanDeviceId BL_DRIVE = new CanDeviceId(2, "canivore1");
	public static final CanDeviceId BL_ROTATION = new CanDeviceId(6, "canivore1");
	public static final CanDeviceId BL_CANCODER = new CanDeviceId(10, "canivore1");

	public static final CanDeviceId BR_DRIVE = new CanDeviceId(3, "canivore1");
	public static final CanDeviceId BR_ROTATION = new CanDeviceId(7, "canivore1");
	public static final CanDeviceId BR_CANCODER = new CanDeviceId(11, "canivore1");

	public static final CanDeviceId INTAKE_PIVOT = new CanDeviceId(8, "canivore1"); 
	public static final CanDeviceId INTAKE_ROLLER = new CanDeviceId(9, "canivore1"); 
	public static final CanDeviceId ELEVATOR = new CanDeviceId(10, "canivore1"); 
	public static final CanDeviceId INDEXER = new CanDeviceId(11, "canivore1"); 
	public static final CanDeviceId ENDEFFECTOR_WRIST = new CanDeviceId(12, "canivore1"); 
	public static final CanDeviceId ENDEFFECTOR_ROLLER = new CanDeviceId(13, "canivore1"); 

	public static final CanDeviceId INTAKE_CANCODER = new CanDeviceId(19, "canivore1");

	public static final int PIGEON = 23;
	
	public static final CanDeviceId LEDS = new CanDeviceId(21, "rio");

	/* BEAM BREAK DIO CHANNELS*/
	// spotless:on
}
