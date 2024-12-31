float costFunction(PID &pid, float setpoint, int steps) {
    float actual = 0;  // Starting value (e.g., position, speed)
    float total_error = 0;

    for (int i = 0; i < steps; ++i) {
        float control_signal = pid.compute(setpoint, actual);
        actual += control_signal;  // Update system state (simplified for example)
        total_error += abs(setpoint - actual);  // Sum of absolute errors

        // Optionally, simulate system dynamics like damping, inertia, etc.
    }

    return total_error;  // Lower error is better
}
