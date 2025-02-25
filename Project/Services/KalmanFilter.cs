namespace Project.Services
{
    public class KalmanFilter
    {
        private double Q = 0.1;  // Process noise
        private double R = 1.0;  // Measurement noise
        private double P = 1.0;  // Estimation error
        private double X = 0.0;  // State
        private double K = 0.0;  // Kalman gain

        public double Update(double measurement)
        {
            // Prediction
            P = P + Q;

            // Update
            K = P / (P + R);
            X = X + K * (measurement - X);
            P = (1 - K) * P;

            return X;
        }

        public void Reset()
        {
            P = 1.0;
            X = 0.0;
            K = 0.0;
        }
    }
}
