class PID {
    public:
        PID(double Kp, double Ki, double Kd);

        void setTunings(double Kp, double Ki, double Kd);
        void setOutputLimits(double min, double max);
        void setMode(bool mode);
        void setSampleTime(int sampleTime);

        double compute(double input, double setpoint);
    
    private:
        double _Kp;
        double _Ki;
        double _Kd;

        double _min;
        double _max;

        bool _mode; // true = manual, false = automatic

        int _sampleTime;

        double _lastInput;
        double _output;
        double _integral;
        unsigned long _lastTime;
};