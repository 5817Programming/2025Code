package com.team5817.lib.util;

import com.team5817.lib.requests.LambdaRequest;
import com.team5817.lib.requests.Request;
import edu.wpi.first.wpilibj.Timer;

public class Stopwatch {

	private double startTime = Double.POSITIVE_INFINITY;

	public void start() {
		startTime = Timer.getTimestamp();
	}

	public void startIfNotRunning() {
		if (Double.isInfinite(startTime)) {
			start();
		}
	}

	public double getTime() {
		if (Double.isInfinite(startTime)) {
			return 0.0;
		}
		return Timer.getTimestamp() - startTime;
	}

	public void reset() {
		startTime = Double.POSITIVE_INFINITY;
	}

	public Request getStartRequest() {
		return new LambdaRequest(this::start);
	}
}
