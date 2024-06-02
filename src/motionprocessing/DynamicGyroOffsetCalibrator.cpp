#include "DynamicGyroOffsetCalibrator.h"
#include <cmath>

bool DynamicGyroOffsetCalibrator::addGyroSample(uint32_t dtMicros, const int16_t* Gxyz) {
    if (isCalibrationFinished_) {
        return false;
    }

    elapsedTime += dtMicros;

    // if the gyro variance is greater than the threshold, re-calibrate.
    if (elapsedTime >= CALIBRATION_DURATION) {
        uint16_t VarGxyz[3];
        calculateGyroVariance(VarGxyz);
        for (int i = 0; i < 3; i++) {
            // VarGxyz was calculated in raw gyro unit, so convert it to deg/s by multiplying ((gyroScale * (180 / PI))^2
            double coefficient = gyroScaleXYZ[i] * (180. / PI);
            coefficient *= coefficient;
            //logger.debug("%s: VarGxyz[%d]=%lf", __func__, i, VarGxyz[i] * coefficient);
            if (VarGxyz[i] * coefficient > GYRO_VARIANCE_UPPER_BOUND) {
                logger.debug("%s: calibration restarted", __func__);
                restartCalibration();
                return true;
            }
        }
        isCalibrationFinished_ = true;
        return false;
    }

    for (int i = 0; i < 3; i++) {
        gyroSum[i] += Gxyz[i];
        gyroSquareSum[i] += Gxyz[i] * Gxyz[i];
    }

    gyroSampleCount++;

    return false;
}

bool DynamicGyroOffsetCalibrator::addAccSample(uint32_t /* dtMicros */, const sensor_real_t* Axyz) {
    if (isCalibrationFinished_) {
        return false;
    }

    for (int i = 0; i < 3; i++) {
        if (std::abs(Axyz[i]) > ACC_UPPER_BOUND) {
            //logger.debug("%s: calibration restarted", __func__);
            //restartCalibration();
            //return true;
        }
    }

    return false;
}

void DynamicGyroOffsetCalibrator::getCalibratedGyroOffset(int16_t* Gxyz) {
    if (!isCalibrationFinished_) {
        logger.debug("%s is called before calibration is finished", __func__);
        return;
    }

    for (int i = 0; i < 3; i++) {
        if (gyroSampleCount == 0) {
            logger.debug("%s is called with gyroSampleCount == 0, something is wrong!", __func__);
            return;
        }

        Gxyz[i] = gyroSum[i] / gyroSampleCount;
    }
}

void DynamicGyroOffsetCalibrator::restartCalibration() {
    elapsedTime = 0;
    gyroSampleCount = 0;

    for (int i = 0; i < 3; i++) {
        gyroSum[i] = 0;
        gyroSquareSum[i] = 0;
    }

    isCalibrationFinished_ = false;
}

void DynamicGyroOffsetCalibrator::calculateGyroAverage(uint16_t* AvgGxyz) {
    if (gyroSampleCount == 0) {
        logger.debug("%s is called with gyroSampleCount == 0, something is wrong!", __func__);
        return;
    }

    for (int i = 0; i < 3; i++) {
        AvgGxyz[i] = gyroSum[i] / gyroSampleCount;
    }
}

void DynamicGyroOffsetCalibrator::calculateGyroVariance(uint16_t* VarGxyz) {
    if (gyroSampleCount == 0) {
        logger.debug("%s is called with gyroSampleCount == 0, something is wrong!", __func__);
        return;
    }

    uint16_t AvgGxyz[3];
    calculateGyroAverage(AvgGxyz);

    for (int i = 0; i < 3; i++) {
        VarGxyz[i] = gyroSquareSum[i] / gyroSampleCount - AvgGxyz[i] * AvgGxyz[i];
    }
}