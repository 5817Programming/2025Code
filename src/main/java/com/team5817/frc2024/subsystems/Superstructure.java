package com.team5817.frc2024.subsystems;

import com.team254.lib.util.TimeDelayedBoolean;
import com.team5817.frc2024.led.TimedLEDState;
import com.team5817.frc2024.loops.ILooper;
import com.team5817.frc2024.loops.Loop;
import com.team5817.lib.drivers.BeamBreak;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import java.util.ArrayList;
import java.util.List;

public class Superstructure extends Subsystem {

	private static Superstructure mInstance;

	public static synchronized Superstructure getInstance() {
		if (mInstance == null) {
			mInstance = new Superstructure();
		}

		return mInstance;
	}

	// Request tracking variables
	private Request activeRequest = null;
	private ArrayList<Request> queuedRequests = new ArrayList<>(0);
	private boolean hasNewRequest = false;
	private boolean allRequestsComplete = false;

	// Subsystems

	// LEDs
	private final LEDs mLEDs = LEDs.getInstance();
	private TimedLEDState mHeldState = TimedLEDState.NOTE_HELD_SHOT;


	// Target tracking
	private Drive mDrive;
	private double mDistanceToTarget = 0.0;
	private double mAngularErrToTarget = 0.0;

	Superstructure() {
		mDrive = Drive.getInstance();
	}
	public boolean requestsCompleted() {
		return allRequestsComplete;
	}

	public void request(Request r) {
		setActiveRequest(r);
		clearRequestQueue();
	}

	private void setActiveRequest(Request request) {
		activeRequest = request;
		hasNewRequest = true;
		allRequestsComplete = false;
	}

	private void clearRequestQueue() {
		queuedRequests.clear();
	}

	private void setRequestQueue(List<Request> requests) {
		clearRequestQueue();
		for (Request req : requests) {
			queuedRequests.add(req);
		}
	}

	private void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
		request(activeRequest);
		setRequestQueue(requests);
	}

	private void addRequestToQueue(Request req) {
		queuedRequests.add(req);
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				clearRequestQueue();
			}

			@Override
			public void onLoop(double timestamp) {
				try {
					if (hasNewRequest && activeRequest != null) {
						activeRequest.act();
						hasNewRequest = false;
					}

					if (activeRequest == null) {
						if (queuedRequests.isEmpty()) {
							allRequestsComplete = true;
						} else {
							request(queuedRequests.remove(0));
						}
					} else if (activeRequest.isFinished()) {
						activeRequest = null;
					}

				} catch (Exception e) {
					e.printStackTrace();
				}
			}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

	@Override
	public void stop() {
		activeRequest = null;
		clearRequestQueue();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void readPeriodicInputs() {
	}

	@Override
	public void outputTelemetry() {
	}

	/* Superstructure functions */



	
	/**
	 * BeamBreak Sensor reading.
	 * 
	 * @param mBreak BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * 
	 * @return Boolean for if target state is acheived. 
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state) {
		return new Request() {

			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return mBreak.get() == target_state;
			}
		};
	}

	/**
	 * Debounced BeamBreak Sensor reading.
	 * 
	 * @param mBreak BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * @param delayed_wait_seconds Debounces time from a BeamBreak Sensor. 
	 * 
	 * @return Boolean for if target state is acheived after debouncing the signal. 
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state, double delayed_wait_seconds) {
		return new Request() {

			TimeDelayedBoolean timeout = new TimeDelayedBoolean();

			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return timeout.update(mBreak.get() == target_state, delayed_wait_seconds);
			}
		};
	}

	/**
	 * Update state of LEDs based on BeamBreak readings.
	 */
	private Request updateLEDsRequest() {
		return new Request() {

			@Override
			public void act() {
				updateLEDs();
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		};
	}

	/**
	 * Update state of LEDs based on BeamBreak readings.
	 */
	private void updateLEDs() {
	}


	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	private Request idleRequest() {
		return new ParallelRequest(
		);
	}

	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	public void idleState() {
		request(idleRequest());
	}
}