#ifndef DYNAMIC_GYRO_OFFSET_CALIBRATOR_H
#define DYNAMIC_GYRO_OFFSET_CALIBRATOR_H

#include "types.h"
#include <cstdint>
#include "logging/Logger.h"

class DynamicGyroOffsetCalibrator {
    private:
        // parameters used for the condition under which dynamic gyro offset calibration is applied
        static constexpr uint32_t CALIBRATION_DURATION     = 2000 * 1000; // microseconds
        static constexpr double GYRO_DEVIATION_UPPER_BOUND = 3.0;         // deg/sec
        static constexpr double ACC_UPPER_BOUND            = 0.8;         // m/s²
        static constexpr double GYRO_VARIANCE_UPPER_BOUND  = GYRO_DEVIATION_UPPER_BOUND * GYRO_DEVIATION_UPPER_BOUND;

    public:
        DynamicGyroOffsetCalibrator(double gyroScaleX,
                                    double gyroScaleY,
                                    double gyroScaleZ,
                                    SlimeVR::Logging::Logger& logger) :
            gyroScaleXYZ{gyroScaleX, gyroScaleY, gyroScaleZ}, logger(logger) {}

        // Gxyz is raw gyro value
        // returns true if calibration restarted
        bool addGyroSample(uint32_t dtMicros, const int16_t* Gxyz);

        // Axyz is in m/s²
        // returns true if calibration restarted
        bool addAccSample(uint32_t dtMicros, const sensor_real_t* Axyz);

        bool isCalibrationFinished() { return isCalibrationFinished_; }
        void getCalibratedGyroOffset(int16_t* Gxyz);
        void restartCalibration();

    private:
        void calculateGyroAverage(uint16_t* AvgGxyz);
        void calculateGyroVariance(uint16_t* VarGxyz);

        uint32_t elapsedTime = 0;      // milliseconds
        uint32_t gyroSampleCount = 0;  // number of gyro samples added so far
        int64_t gyroSum[3]       = {}; // used to calculate the average
        int64_t gyroSquareSum[3] = {}; // used to calculate the standard deviation
        double gyroScaleXYZ[3]   = {}; // rad/s/LSB
        bool isCalibrationFinished_ = false;

        SlimeVR::Logging::Logger& logger;
};
#endif