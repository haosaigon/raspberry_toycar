# kalman_filter.py
class KalmanFilter:
    def __init__(self, process_variance=1e-3, measurement_variance=1e-2, initial_estimate=0.0):
        # Process noise variance (Q)
        self.Q = process_variance
        # Measurement noise variance (R)
        self.R = measurement_variance
        # Initial estimates
        self.x = initial_estimate  # State estimate
        self.P = 1.0               # Estimate covariance

    def update(self, measurement):
        """
        Perform a single Kalman update step using a new measurement.
        Returns the new filtered value.
        """
        # Prediction step
        self.P = self.P + self.Q

        # Kalman Gain
        K = self.P / (self.P + self.R)

        # Update estimate
        self.x = self.x + K * (measurement - self.x)

        # Update covariance
        self.P = (1 - K) * self.P

        return self.x
