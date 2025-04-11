namespace Project.Services
{
    public class KalmanFilter
    {
        // State vector [angle, angular_velocity]
        private double[] X = new double[] { 0, 0 };

        // State transition matrix
        private double[,] F;

        // Measurement matrix
        private double[,] H;

        // State covariance matrix
        private double[,] P;

        // Process noise covariance
        private double[,] Q;

        // Measurement noise covariance
        private double R;

        // Identity matrix
        private double[,] I;

        // Time step
        private double dt;

        public KalmanFilter(double processNoise = 0.001, double measurementNoise = 0.1, double dt = 0.1)
        {
            this.dt = dt;

            // Servo hareketleri için optimize edilmiş matrisler
            F = new double[,] { { 1, dt }, { 0, 1 } };
            H = new double[,] { { 1, 0 } };

            // Başlangıç kovaryans matrisi - daha yüksek başlangıç belirsizliği
            P = new double[,] { { 10, 0 }, { 0, 10 } };

            // Process noise - servo hareketi için optimize edildi
            Q = new double[,] {
                { dt * dt * 0.1, dt * 0.2 },
                { dt * 0.2, 0.5 }
            };

            // Measurement noise - sensör gürültüsü için optimize edildi
            R = measurementNoise * 2;

            I = new double[,] { { 1, 0 }, { 0, 1 } };

            Q = MatrixScale(Q, processNoise);
        }

        public double Update(double measurement)
        {
            // Predict
            X = MatrixMultiply(F, X);
            P = MatrixAdd(
                MatrixMultiply(MatrixMultiply(F, P), MatrixTranspose(F)),
                Q
            );

            // Update
            double[] y = new double[] { measurement - X[0] };
            double[,] S = MatrixAdd(
                MatrixMultiply(MatrixMultiply(H, P), MatrixTranspose(H)),
                new double[,] { { R } }
            );
            double[,] K = MatrixMultiply(
                MatrixMultiply(P, MatrixTranspose(H)),
                1.0 / S[0, 0]
            );

            X = MatrixAdd(X, MatrixMultiply(K, y));
            P = MatrixMultiply(
                MatrixSubtract(I, MatrixMultiply(K, H)),
                P
            );

            return X[0];
        }

        public void Reset(double initialAngle = 0)
        {
            X[0] = initialAngle;
            X[1] = 0;
            P = new double[,] { { 1, 0 }, { 0, 1 } };
        }

        // Matrix operations helpers
        private double[] MatrixMultiply(double[,] a, double[] b)
        {
            int m = a.GetLength(0);
            double[] result = new double[m];

            for (int i = 0; i < m; i++)
            {
                result[i] = 0;
                for (int j = 0; j < b.Length; j++)
                {
                    result[i] += a[i, j] * b[j];
                }
            }

            return result;
        }

        private double[,] MatrixMultiply(double[,] a, double[,] b)
        {
            int m = a.GetLength(0);
            int n = b.GetLength(1);
            int p = a.GetLength(1);

            double[,] result = new double[m, n];

            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    result[i, j] = 0;
                    for (int k = 0; k < p; k++)
                    {
                        result[i, j] += a[i, k] * b[k, j];
                    }
                }
            }

            return result;
        }

        private double[,] MatrixMultiply(double[,] a, double scalar)
        {
            int m = a.GetLength(0);
            int n = a.GetLength(1);
            double[,] result = new double[m, n];

            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    result[i, j] = a[i, j] * scalar;
                }
            }

            return result;
        }

        private double[,] MatrixScale(double[,] a, double scalar)
        {
            return MatrixMultiply(a, scalar);
        }

        private double[] MatrixAdd(double[] a, double[] b)
        {
            double[] result = new double[a.Length];
            for (int i = 0; i < a.Length; i++)
            {
                result[i] = a[i] + b[i];
            }
            return result;
        }

        private double[,] MatrixAdd(double[,] a, double[,] b)
        {
            int m = a.GetLength(0);
            int n = a.GetLength(1);
            double[,] result = new double[m, n];

            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    result[i, j] = a[i, j] + b[i, j];
                }
            }

            return result;
        }

        private double[,] MatrixSubtract(double[,] a, double[,] b)
        {
            int m = a.GetLength(0);
            int n = a.GetLength(1);
            double[,] result = new double[m, n];

            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    result[i, j] = a[i, j] - b[i, j];
                }
            }

            return result;
        }

        private double[,] MatrixTranspose(double[,] a)
        {
            int m = a.GetLength(0);
            int n = a.GetLength(1);
            double[,] result = new double[n, m];

            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    result[j, i] = a[i, j];
                }
            }

            return result;
        }
    }
}
