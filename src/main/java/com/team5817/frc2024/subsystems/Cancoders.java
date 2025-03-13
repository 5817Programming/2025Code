package com.team5817.frc2024.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.team254.lib.drivers.CanDeviceId;
import com.team5817.frc2024.Ports;

import edu.wpi.first.units.measure.Angle;
import java.util.Optional;

/**
 * Container to hold the Cancoders so we can initialize them
 * earlier than everything else and DI them to the swerve modules.
 */
public class Cancoders {
	private final CANcoder mFrontLeft;
	private final CANcoder mFrontRight;
	private final CANcoder mBackLeft;
	private final CANcoder mBackRight;

	private final CanTsObserver mFrontRightObserver;
	private final CanTsObserver mFrontLeftObserver;
	private final CanTsObserver mBackLeftObserver;
	private final CanTsObserver mBackRightObserver;

	private static final double kBootUpErrorAllowanceTime = 10.0;

	/**
	 * Observer class to monitor CANcoder timestamp updates.
	 */
	private static class CanTsObserver {
		private final CANcoder cancoder;
		private Optional<Double> lastTs = Optional.empty();
		private int validUpdates = 0;
		private static final int kRequiredValidTimestamps = 10;

		/**
		 * Constructs a CanTsObserver for the given CANcoder.
		 *
		 * @param cancoder the CANcoder to observe
		 */
		public CanTsObserver(CANcoder cancoder) {
			this.cancoder = cancoder;
		}

		/**
		 * Checks if the CANcoder has received the required number of valid timestamp updates.
		 *
		 * @return true if the required number of valid updates have been received, false otherwise
		 */
		public boolean hasUpdate() {
			// Need to call this to update ts
			StatusSignal<Angle> absolutePositionSignal = cancoder.getAbsolutePosition();

			double ts = absolutePositionSignal.getTimestamp().getTime();
			if (lastTs.isEmpty()) {
				lastTs = Optional.of(ts);
			}
			if (ts > lastTs.get()) {
				validUpdates++;
				lastTs = Optional.of(ts);
			}
			return validUpdates > kRequiredValidTimestamps;
		}
	}

	private static Cancoders sInstance;

	/**
	 * Returns the singleton instance of the Cancoders class.
	 *
	 * @return the singleton instance
	 */
	public static Cancoders getInstance() {
		if (sInstance == null) {
			sInstance = new Cancoders();
		}
		return sInstance;
	}

	/**
	 * Builds and configures a CANcoder with the given device ID.
	 *
	 * @param canDeviceId the device ID of the CANcoder
	 * @return the configured CANcoder
	 */
	private CANcoder build(CanDeviceId canDeviceId) {
		CANcoder thisCancoder = new CANcoder(canDeviceId.getDeviceNumber(), canDeviceId.getBus());
		// CANcoderConfigurator configurator = thisCancoder.getConfigurator();
		// double initpose = thisCancoder.getAbsolutePosition().getValue().in(Degree);
		// CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

		// canCoderConfig.MagnetSensor.MagnetOffset =
		// -thisCancoder.getAbsolutePosition().getValueAsDouble()+initpose;
		// canCoderConfig.MagnetSensor.SensorDirection =
		// SensorDirectionValue.CounterClockwise_Positive;

		// double startTime = Timer.getFPGATimestamp();
		// boolean timedOut = false;
		// boolean goodInit = false;
		// int attempt = 1;
		// while (!goodInit && !timedOut && attempt < 20) {
		// System.out.println("Initing CANCoder " + canDeviceId.getDeviceNumber() + " /
		// attempt: " + attempt + " / "
		// + (Timer.getFPGATimestamp() - startTime) + " seconds elapsed");
		// StatusCode settingsCode = configurator.apply(canCoderConfig);
		// StatusCode sensorCode =
		// thisCancoder.getAbsolutePosition().setUpdateFrequency(20);

		// goodInit = settingsCode == StatusCode.OK && sensorCode == StatusCode.OK;

		// timedOut = (Timer.getFPGATimestamp()) - startTime >=
		// kBootUpErrorAllowanceTime;
		// attempt++;
		// }

		return thisCancoder;
	}

	/**
	 * Constructs the Cancoders instance and initializes the CANcoders.
	 */
	private Cancoders() {
		mFrontLeft = build(Ports.FL_CANCODER);
		mFrontLeftObserver = new CanTsObserver(mFrontLeft);

		mFrontRight = build(Ports.FR_CANCODER);
		mFrontRightObserver = new CanTsObserver(mFrontRight);

		mBackLeft = build(Ports.BL_CANCODER);
		mBackLeftObserver = new CanTsObserver(mBackLeft);

		mBackRight = build(Ports.BR_CANCODER);
		mBackRightObserver = new CanTsObserver(mBackRight);
	}

	/**
	 * Checks if all CANcoders have been initialized with valid timestamp updates.
	 *
	 * @return true if all CANcoders have been initialized, false otherwise
	 */
	public boolean allHaveBeenInitialized() {
		return mFrontLeftObserver.hasUpdate()
				&& mFrontRightObserver.hasUpdate()
				&& mBackLeftObserver.hasUpdate()
				&& mBackRightObserver.hasUpdate();
	}

	/**
	 * Returns the front left CANcoder.
	 *
	 * @return the front left CANcoder
	 */
	public CANcoder getFrontLeft() {
		return mFrontLeft;
	}

	/**
	 * Returns the front right CANcoder.
	 *
	 * @return the front right CANcoder
	 */
	public CANcoder getFrontRight() {
		return mFrontRight;
	}

	/**
	 * Returns the back left CANcoder.
	 *
	 * @return the back left CANcoder
	 */
	public CANcoder getBackLeft() {
		return mBackLeft;
	}

	/**
	 * Returns the back right CANcoder.
	 *
	 * @return the back right CANcoder
	 */
	public CANcoder getBackRight() {
		return mBackRight;
	}
}
