class PID {
public:
    float Kp, Ki, Kd;
    float previous_error, integral;

    PID(float p, float i, float d) : Kp(p), Ki(i), Kd(d), previous_error(0), integral(0) {}

    float compute(float setpoint, float actual) {
        // Calculate error
        float error = setpoint - actual;
        integral += error;
        float derivative = error - previous_error;

        // PID output
        float output = Kp * error + Ki * integral + Kd * derivative;

        // Save error for next cycle
        previous_error = error;

        return output;
    }
};
