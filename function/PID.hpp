typedef struct
{

	/* Controller gains */
	double Kp;
	double Ki;
	double Kd;

	/* Derivative low-pass filter time constant */
	double tau;

	/* Output limits */
	double limMin;
	double limMax;

	/* Integrator limits */
	double limMinInt;
	double limMaxInt;

	/* Sample time (in seconds) */
	double T;

	/* Controller "memory" */
	double integrator;
	double prevError; /* Required for integrator */
	double differentiator;
	double prevMeasurement; /* Required for differentiator */

	/* Controller output */
	double out;

} PIDController;

void PIDController_Init(PIDController &pid)
{

	/* Clear controller variables */
	pid.integrator = 0.0f;
	pid.prevError = 0.0f;

	pid.differentiator = 0.0f;
	pid.prevMeasurement = 0.0f;

	pid.out = 0.0f;
}

float PIDController_Update(PIDController &pid, float setpoint, float measurement,double coff)
{

	/*
	 * Error signal
	 */
	double error = coff*(setpoint - measurement);

	/*
	 * Proportional
	 */
	float proportional = pid.Kp * error;

	/*
	 * Integral
	 */
	pid.integrator = pid.integrator + 0.5f * pid.Ki * pid.T * (error + pid.prevError);

	/* Anti-wind-up via integrator clamping */
	if (pid.integrator > pid.limMaxInt)
	{

		pid.integrator = pid.limMaxInt;
	}
	else if (pid.integrator < pid.limMinInt)
	{

		pid.integrator = pid.limMinInt;
	}

	/*
	 * Derivative (band-limited differentiator)
	 */

	pid.differentiator = -(2.0f * pid.Kd * coff*(measurement - pid.prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
							+ (2.0f * pid.tau - pid.T) * pid.differentiator) /
						  (2.0f * pid.tau + pid.T);

	/*
	 * Compute output and apply limits
	 */
	pid.out = proportional + pid.integrator + pid.differentiator;

	if (pid.out > pid.limMax)
	{

		pid.out = pid.limMax;
	}
	else if (pid.out < pid.limMin)
	{

		pid.out = pid.limMin;
	}

	/* Store error and measurement for later use */
	pid.differentiator = error;
	pid.prevMeasurement = measurement;

	/* Return controller output */
	return pid.out;
}
void print_PID(const PIDController &pid)
{
	std::cout<<"Ki="<<pid.Ki<<"\t"<<"Kd="<<pid.Kd<<"\t"<<"Kp="<<pid.Kp<<"\t"<<std::endl;
}
