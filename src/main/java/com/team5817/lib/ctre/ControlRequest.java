/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.team5817.lib.ctre;

import java.util.Map;

import com.ctre.phoenix6.StatusCode;

/**
 * Abstract Control Request class that other control requests extend for use.
 */
public abstract class ControlRequest {
    protected final String name;
    protected final double Output = 0;
    protected final double Velocity = 0;

    /**
     * Constructs a new Control Request with the given name.
     *
     * @param name Name of the control request
     */
    public ControlRequest(String name) {
        this.name = name;
    }

    /**
     * Sets the period at which this control will update at.
     * This is designated in Hertz, with a minimum of 20 Hz
     * (every 50 ms) and a maximum of 1000 Hz (every 1 ms).
     * <p>
     * If this field is set to 0 Hz, the control request will
     * be sent immediately as a one-shot frame. This may be useful
     * for advanced applications that require outputs to be
     * synchronized with data acquisition. In this case, we
     * recommend not exceeding 50 ms between control calls.
     *
     * @param newUpdateFreqHz Parameter to modify
     * @return Itself
     */
    public abstract ControlRequest withUpdateFreqHz(double newUpdateFreqHz);

    /**
     * Gets the name of this control request.
     *
     * @return Name of the control request
     */
    public String getName() {
        return name;
    }

    public abstract StatusCode sendRequest(String network, int deviceHash);

    /**
     * Gets information about this control request.
     *
     * @return Map of control parameter names and corresponding applied values
     */
    public abstract Map<String, String> getControlInfo();
}
