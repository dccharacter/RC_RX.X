#include "xc.h"
#include "main.h"
#include "ITG3200.h"
#include "i2c.h"
#include "usart.h"


void ITG3200_Init(ITG3200_InitTypeDef* itg3200) {
    /*uint8_t _Range = itg3200->ITG3200_DigitalLowPassFilterCfg | 0x18;

    ITG3200_WriteRegister(0x15, itg3200->ITG3200_SampleRateDivider);
    ITG3200_WriteRegister(0x16, _Range);
    ITG3200_WriteRegister(0x3E, itg3200->ITG3200_PowerControl);
    ITG3200_WriteRegister(0x17, itg3200->ITG3200_IrqConfig); */

    ITG3200_WriteRegister(0x3E, 0x80);
    ITG3200_WriteRegister(0x16, 0x1D);
    ITG3200_WriteRegister(0x3E, 0x01);
    __delay_ms(100);

    ITG3200_SetGains(1.8, 1.8, 1.8);
    ITG3200_SetOffsets(0, 0, 0);
    ITG3200_SetRevPolarity(0, 1, 0);

    ITG3200_ZeroCalibrate(30, 2);
}

uint8_t ITG3200_ReadDevID() {
    uint8_t result = 0;
    result = I2C_ReadReg(ITG3200_WRITE, 0);
    return result;
}

uint8_t ITG3200_ReadRegister(uint8_t reg) {
    uint8_t result = 0;
    result = I2C_ReadReg(ITG3200_WRITE, reg);
    return result;
}

/* Reads sensors from ITG3200. First three values are XYZ data,
 * the fourth is a die temperature
 *
 */
void ITG3200_GetMeasurements(int16_t *values) {
    uint8_t temp[8];
    I2C_ReadMultiRegs(ITG3200_READ, 0x1B, 8, temp);
    values[0] = (int16_t) (temp[3] + (temp[2] << 8));
    values[1] = (int16_t) (temp[5] + (temp[4] << 8));
    values[2] = (int16_t) (temp[7] + (temp[6] << 8));
    values[3] = (int16_t) (temp[1] + (temp[0] << 8));
}

void ITG3200_WriteRegister( uint8_t reg, uint8_t value) {
    I2C_WriteReg(ITG3200_WRITE, reg, value);
}

void ITG3200_ReadGyroRaw(int *_GyroX, int *_GyroY, int *_GyroZ) {
    int16_t raw_values[4];
    ITG3200_GetMeasurements(raw_values);
    *_GyroX = raw_values[0];
    *_GyroY = raw_values[1];
    *_GyroZ = raw_values[2];
}

void ITG3200_ReadGyroRawCal(int *_GyroX, int *_GyroY, int *_GyroZ) {
    ITG3200_ReadGyroRaw(_GyroX, _GyroY, _GyroZ);
    *_GyroX += offsets[0];
    *_GyroY += offsets[1];
    *_GyroZ += offsets[2];
}

void ITG3200_ReadGyro(float *_GyroX, float *_GyroY, float *_GyroZ) {
    int x, y, z;

    ITG3200_ReadGyroRawCal(&x, &y, &z); // x,y,z will contain calibrated integer values from the sensor
    *_GyroX = (float) ((x * polarities[0] * gains[0]) / 14.375);
    *_GyroY = (float) ((y * polarities[1] * gains[1]) / 14.375);
    *_GyroZ = (float) ((z * polarities[2] * gains[2]) / 14.375);
}

void ITG3200_SetRevPolarity(uint8_t _Xpol, uint8_t _Ypol, uint8_t _Zpol) {
    polarities[0] = _Xpol ? -1 : 1;
    polarities[1] = _Ypol ? -1 : 1;
    polarities[2] = _Zpol ? -1 : 1;
}

void ITG3200_ZeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) {
    int x, y, z;
    int i, j, k;
    int tmpOffsets[] = {0, 0, 0};

    for (i = 0; i < totSamples; i++) {

        for (j = 0; j < sampleDelayMS; j++)
            for (k = 0; k < 1000; k++);
        ITG3200_ReadGyroRaw(&x, &y, &z);
        tmpOffsets[0] += x;
        tmpOffsets[1] += y;
        tmpOffsets[2] += z;
    }
    ITG3200_SetOffsets(-tmpOffsets[0] / totSamples, -tmpOffsets[1] / totSamples, -tmpOffsets[2] / totSamples);
}

void ITG3200_SetGains(float _Xgain, float _Ygain, float _Zgain) {
    gains[0] = _Xgain;
    gains[1] = _Ygain;
    gains[2] = _Zgain;
}

void ITG3200_SetOffsets(int _Xoffset, int _Yoffset, int _Zoffset) {
    offsets[0] = _Xoffset;
    offsets[1] = _Yoffset;
    offsets[2] = _Zoffset;
}

int16_t ITG3200_ReadGyroAxis(AXES_type axis) {
    uint8_t start_reg;
    int *offset;
    if (axis == AXIS_X) {
        offset = offsets;
        start_reg = 0x1D;
    } else if (axis == AXIS_Z) {
        offset = offsets + 2;
        start_reg = 0x21;
    } else {
        offset = offsets + 1;
        start_reg = 0x1F;
    }
    int16_t res;
    uint8_t data[2];
    I2C_ReadMultiRegs(ITG3200_READ, start_reg, 2, data);
    res = (int16_t) (data[1] + (data[0] << 8)) + *offset;
    return res;
}