/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */


#include <math.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RPM_FILTER)

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "drivers/dshot.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/motor.h"

#include "scheduler/scheduler.h"

#include "sensors/gyro.h"

#include "rpm_filter.h"

#define RPM_FILTER_MAXHARMONICS 3
#define SECONDS_PER_MINUTE      60.0f
#define ERPM_PER_LSB            100.0f
#define MIN_UPDATE_T            0.001f

#define RPM_LPF_UPDATE_DELAY_US 5000

static pt1Filter_t rpmFilters[MAX_SUPPORTED_MOTORS];

typedef struct rpmNotchFilter_s
{
    uint8_t harmonics;
    float   minHz;
    float   maxHz;
    float   q;
    float   loopTime;

    biquadFilter_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_MAXHARMONICS];
} rpmNotchFilter_t;

typedef union rpmGyroLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} rpmGyroLowpass_t;

typedef union rpmDtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} rpmDtermLowpass_t;

FAST_RAM_ZERO_INIT static float   erpmToHz;
FAST_RAM_ZERO_INIT static float   filteredMotorErpm[MAX_SUPPORTED_MOTORS];
FAST_RAM_ZERO_INIT static float   minMotorFrequency;
FAST_RAM_ZERO_INIT static float   avgMotorFrequency;
FAST_RAM_ZERO_INIT static uint8_t numberFilters;
FAST_RAM_ZERO_INIT static uint8_t numberRpmNotchFilters;
FAST_RAM_ZERO_INIT static uint8_t filterUpdatesPerIteration;
FAST_RAM_ZERO_INIT static float   pidLooptime;
FAST_RAM_ZERO_INIT static rpmNotchFilter_t filters[2];
FAST_RAM_ZERO_INIT static rpmNotchFilter_t* gyroFilter;
FAST_RAM_ZERO_INIT static rpmNotchFilter_t* dtermFilter;

FAST_RAM_ZERO_INIT static uint8_t currentMotor;
FAST_RAM_ZERO_INIT static uint8_t currentHarmonic;
FAST_RAM_ZERO_INIT static uint8_t currentFilterNumber;
FAST_RAM static rpmNotchFilter_t* currentFilter = &filters[0];


FAST_RAM_ZERO_INIT static rpmGyroLowpass_t rpmGyroLpfFilter[XYZ_AXIS_COUNT];
FAST_RAM_ZERO_INIT static rpmDtermLowpass_t rpmDtermLpfFilter[XYZ_AXIS_COUNT];
FAST_RAM_ZERO_INIT static filterApplyFnPtr rpmGyroLpfApplyFn;
FAST_RAM_ZERO_INIT static filterApplyFnPtr rpmDtermLpfApplyFn;

FAST_RAM_ZERO_INIT static float dT;
FAST_RAM_ZERO_INIT static uint16_t rpmGyroLpfMinHz;
FAST_RAM_ZERO_INIT static uint16_t rpmGyroLpfMaxHz;
FAST_RAM_ZERO_INIT static uint8_t  rpmGyroLowpassType;
FAST_RAM_ZERO_INIT static uint16_t rpmDtermLpfMinHz;
FAST_RAM_ZERO_INIT static uint16_t rpmDtermLpfMaxHz;
FAST_RAM_ZERO_INIT static uint8_t  rpmDtermLowpassType;
FAST_RAM_ZERO_INIT static float gyroCutoffFreq;
FAST_RAM_ZERO_INIT static float dtermCutoffFreq;

PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 3);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    config->gyro_rpm_notch_harmonics = 3;
    config->gyro_rpm_notch_min = 100;
    config->gyro_rpm_notch_q = 500;

    config->dterm_rpm_notch_harmonics = 0;
    config->dterm_rpm_notch_min = 100;
    config->dterm_rpm_notch_q = 500;

    config->rpm_lpf = 150;

    config->rpm_gyro_lpf_min_hz = 0;
    config->rpm_gyro_lpf_max_hz = 0;
    config->rpm_gyro_lowpass_type = FILTER_PT1;
    config->rpm_dterm_lpf_min_hz = 0;
    config->rpm_dterm_lpf_max_hz = 0;
    config->rpm_dterm_lowpass_type = FILTER_BIQUAD;
}

static void rpmNotchFilterInit(rpmNotchFilter_t* filter, int harmonics, int minHz, int q, float looptime)
{
    filter->harmonics = harmonics;
    filter->minHz = minHz;
    filter->q = q / 100.0f;
    filter->loopTime = looptime;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int i = 0; i < harmonics; i++) {
                biquadFilterInit(
                    &filter->notch[axis][motor][i], minHz * i, looptime, filter->q, FILTER_NOTCH);
            }
        }
    }
}

void rpmFilterInit(const rpmFilterConfig_t *config)
{
    currentFilter = &filters[0];
    currentMotor = currentHarmonic = currentFilterNumber = 0;

    numberRpmNotchFilters = 0;
    if (!motorConfig()->dev.useDshotTelemetry) {
        gyroFilter = dtermFilter = NULL;
        return;
    }

    pidLooptime = gyro.targetLooptime * pidConfig()->pid_process_denom;
    if (config->gyro_rpm_notch_harmonics) {
        gyroFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(gyroFilter, config->gyro_rpm_notch_harmonics,
                           config->gyro_rpm_notch_min, config->gyro_rpm_notch_q, gyro.targetLooptime);
        // don't go quite to nyquist to avoid oscillations
        gyroFilter->maxHz = 0.48f / (gyro.targetLooptime * 1e-6f);
    } else {
        gyroFilter = NULL;
    }
    if (config->dterm_rpm_notch_harmonics) {
        dtermFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(dtermFilter, config->dterm_rpm_notch_harmonics,
                           config->dterm_rpm_notch_min, config->dterm_rpm_notch_q, pidLooptime);
        // don't go quite to nyquist to avoid oscillations
        dtermFilter->maxHz = 0.48f / (pidLooptime * 1e-6f);
    } else {
        dtermFilter = NULL;
    }

    for (int i = 0; i < getMotorCount(); i++) {
        pt1FilterInit(&rpmFilters[i], pt1FilterGain(config->rpm_lpf, pidLooptime * 1e-6f));
    }

    erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE  / (motorConfig()->motorPoleCount / 2.0f);

    const float loopIterationsPerUpdate = MIN_UPDATE_T / (pidLooptime * 1e-6f);
    numberFilters = getMotorCount() * (filters[0].harmonics + filters[1].harmonics);
    const float filtersPerLoopIteration = numberFilters / loopIterationsPerUpdate;
    filterUpdatesPerIteration = rintf(filtersPerLoopIteration + 0.49f);
}

void rpmLowpassInit(const rpmFilterConfig_t *config)
{
    rpmGyroLpfApplyFn = nullFilterApply;
    rpmDtermLpfApplyFn = nullFilterApply;

    dT = gyro.targetLooptime * 1e-6f;
    rpmGyroLpfMinHz = config->rpm_gyro_lpf_min_hz;
    rpmGyroLpfMaxHz = config->rpm_gyro_lpf_max_hz;
    rpmGyroLowpassType = config->rpm_gyro_lowpass_type;

    rpmDtermLpfMinHz = config->rpm_dterm_lpf_min_hz;
    rpmDtermLpfMaxHz = config->rpm_dterm_lpf_max_hz;
    rpmDtermLowpassType = config->rpm_dterm_lowpass_type;

    if (rpmGyroLpfMinHz > 0) {
        switch (rpmGyroLowpassType) {
            case FILTER_PT1:
                rpmGyroLpfApplyFn = (filterApplyFnPtr) pt1FilterApply;
                for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                    pt1FilterInit(&rpmGyroLpfFilter[axis].pt1Filter, pt1FilterGain(rpmGyroLpfMinHz, dT));
                }
                break;
            case FILTER_BIQUAD:
                rpmGyroLpfApplyFn = (filterApplyFnPtr) biquadFilterApplyDF1;
                for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                    biquadFilterInitLPF(&rpmGyroLpfFilter[axis].biquadFilter, rpmGyroLpfMinHz, gyro.targetLooptime);
                }
                break;
        }
    }

    if (rpmDtermLpfMinHz > 0) {
        switch (rpmDtermLowpassType) {
            case FILTER_PT1:
                rpmDtermLpfApplyFn = (filterApplyFnPtr) pt1FilterApply;
                for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                    pt1FilterInit(&rpmDtermLpfFilter[axis].pt1Filter, pt1FilterGain(rpmDtermLpfMinHz, dT));
                }
                break;
            case FILTER_BIQUAD:
                rpmDtermLpfApplyFn = (filterApplyFnPtr) biquadFilterApplyDF1;
                for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                    biquadFilterInitLPF(&rpmDtermLpfFilter[axis].biquadFilter, rpmDtermLpfMinHz, gyro.targetLooptime);
                }
        }
    }
}

static float applyFilter(rpmNotchFilter_t* filter, int axis, float value)
{
    if (filter == NULL) {
        return value;
    }
    for (int motor = 0; motor < getMotorCount(); motor++) {
        for (int i = 0; i < filter->harmonics; i++) {
            value = biquadFilterApplyDF1(&filter->notch[axis][motor][i], value);
        }
    }
    return value;
}

float rpmFilterGyro(int axis, float value)
{
    return applyFilter(gyroFilter, axis, value);
}

float rpmFilterDterm(int axis, float value)
{
    return applyFilter(dtermFilter, axis, value);
}

float applyRpmLowpassGyro(int axis, float value) {
    return rpmGyroLpfApplyFn((filter_t *)&rpmGyroLpfFilter[axis], value);
}

float applyRpmLowpassDterm(int axis, float value) {
    return rpmDtermLpfApplyFn((filter_t *)&rpmDtermLpfFilter[axis], value);
}

FAST_RAM_ZERO_INIT static float motorFrequency[MAX_SUPPORTED_MOTORS];

FAST_CODE_NOINLINE void rpmFilterUpdate()
{
    if (gyroFilter == NULL && dtermFilter == NULL) {
        return;
    }

    for (int motor = 0; motor < getMotorCount(); motor++) {
        filteredMotorErpm[motor] = pt1FilterApply(&rpmFilters[motor], getDshotTelemetry(motor));
        if (motor < 4) {
            DEBUG_SET(DEBUG_RPM_FILTER, motor, motorFrequency[motor]);
        }
    }

    for (int i = 0; i < filterUpdatesPerIteration; i++) {
        float frequency = constrainf(
            (currentHarmonic + 1) * motorFrequency[currentMotor], currentFilter->minHz, currentFilter->maxHz);
        biquadFilter_t* template = &currentFilter->notch[0][currentMotor][currentHarmonic];
        // uncomment below to debug filter stepping. Need to also comment out motor rpm DEBUG_SET above
        /* DEBUG_SET(DEBUG_RPM_FILTER, 0, harmonic); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 1, motor); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 2, currentFilter == &gyroFilter); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 3, frequency) */
        biquadFilterUpdate(
            template, frequency, currentFilter->loopTime, currentFilter->q, FILTER_NOTCH);
        for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilter_t* clone = &currentFilter->notch[axis][currentMotor][currentHarmonic];
            clone->b0 = template->b0;
            clone->b1 = template->b1;
            clone->b2 = template->b2;
            clone->a1 = template->a1;
            clone->a2 = template->a2;
        }

        if (++currentHarmonic == currentFilter->harmonics) {
            currentHarmonic = 0;
            if (++currentFilterNumber == numberRpmNotchFilters) {
                currentFilterNumber = 0;
                if (++currentMotor == getMotorCount()) {
                    currentMotor = 0;
                }
                motorFrequency[currentMotor] = erpmToHz * filteredMotorErpm[currentMotor];
                minMotorFrequency = 0.0f;
            }
            currentFilter = &filters[currentFilterNumber];
        }

    }
}

void rpmLpfUpdate(timeUs_t currentTimeUs) {

    static timeUs_t lastRpmLpfUpdateUs = 0;

    if (cmpTimeUs(currentTimeUs, lastRpmLpfUpdateUs) >= RPM_LPF_UPDATE_DELAY_US) {

        rpmAvgMotorFrequency();

        gyroCutoffFreq = constrainf(getCutoffFrequency(rpmGyroLowpassType), rpmGyroLpfMinHz, rpmGyroLpfMaxHz);
        dtermCutoffFreq = constrainf(getCutoffFrequency(rpmDtermLowpassType), rpmDtermLpfMinHz, rpmDtermLpfMaxHz);

        if (rpmGyroLowpassType == FILTER_PT1) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&rpmGyroLpfFilter[axis].pt1Filter, pt1FilterGain(gyroCutoffFreq, dT));
            }
        } else if (rpmGyroLowpassType == FILTER_BIQUAD) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&rpmGyroLpfFilter[axis].biquadFilter, gyroCutoffFreq, gyro.targetLooptime);
            }
        }

        if (rpmDtermLowpassType == FILTER_PT1) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&rpmDtermLpfFilter[axis].pt1Filter, pt1FilterGain(dtermCutoffFreq, dT));
            }
        } else if (rpmDtermLowpassType == FILTER_BIQUAD) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&rpmDtermLpfFilter[axis].biquadFilter, dtermCutoffFreq, gyro.targetLooptime);
            }
        }
        lastRpmLpfUpdateUs = currentTimeUs;
        DEBUG_SET(DEBUG_RPM_LPF, 0, avgMotorFrequency);
        DEBUG_SET(DEBUG_RPM_LPF, 1, gyroCutoffFreq);
        DEBUG_SET(DEBUG_RPM_LPF, 2, dtermCutoffFreq);
    }
}

bool isRpmFilterEnabled(void)
{
    return (motorConfig()->dev.useDshotTelemetry && (rpmFilterConfig()->gyro_rpm_notch_harmonics || rpmFilterConfig()->dterm_rpm_notch_harmonics));
}

float rpmMinMotorFrequency()
{
    if (minMotorFrequency == 0.0f) {
        minMotorFrequency = 10000.0f;
        for (int i = getMotorCount(); i--;) {
            if (motorFrequency[i] < minMotorFrequency) {
                minMotorFrequency = motorFrequency[i];
            }
        }
    }
    return minMotorFrequency;
}

void rpmAvgMotorFrequency() {
    avgMotorFrequency = 0;
    for (int i = 0; i < getMotorCount(); i++) {
        avgMotorFrequency += motorFrequency[i];
    }
    avgMotorFrequency = avgMotorFrequency / getMotorCount();
}

float getCutoffFrequency(uint8_t filterType) {
    float percent = 0;

    if (filterType == FILTER_PT1) {
        percent = 0.25f;
    } else if (filterType == FILTER_BIQUAD) {
        percent = 0.15f;
    }
    return minMotorFrequency - minMotorFrequency * percent;
}

#endif
