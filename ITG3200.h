/* 
 * File:   ITG3200.h
 * Author: dccharacter
 *
 * Created on August 2, 2014, 11:28 PM
 */

#ifndef ITG3200_H
#define	ITG3200_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ITG3200_WRITE 0xD0
#define ITG3200_READ 0xD1

typedef enum {
	ITG3200_LPFBandwidth256Hz = 0,
	ITG3200_LPFBandwidth188Hz = 1,
	ITG3200_LPFBandwidth98Hz = 2,
	ITG3200_LPFBandwidth42Hz = 3,
	ITG3200_LPFBandwidth20Hz = 4,
	ITG3200_LPFBandwidth10Hz = 5,
	ITG3200_LPFBandwidth5Hz = 6
}ITG3200_DLP_CFG_TypeDef;

typedef enum {
	ITG3200_CLKInternal_Oscillator = 0,
	ITG3200_CLKLPLLwX_GyroRef = 1,
	ITG3200_CLKLPLLwY_GyroRef = 2,
	ITG3200_CLKLPLLwZ_GyroRef = 3,
	ITG3200_CLKExternal32768 = 4,
	ITG3200_CLKExternal19200kHz = 5
}ITG3200_CLK_SEL_TypeDef;

typedef struct {
	uint8_t ITG3200_SampleRateDivider; /*!<Register 0x15 (21) – Sample Rate Divider
										This register determines the sample rate of
										the ITG-3200 gyros. The gyros outputs are sampled
										internally at either 1kHz or 8kHz, determined by
										the DLPF_CFG setting (see register 22). This
										sampling is then filtered digitally and delivered
										into the sensor registers after the number of
										cycles determined by this register. The sample
										rate is given by the following formula:
										Fsample = Finternal / (divider+1),
										where Finternal is either 1kHz or 8kHz
										As an example, if the internal sampling is
										at 1kHz, then setting this register to 7 would
										give the following:
										Fsample = 1kHz / (7 + 1) = 125Hz, or 8ms per sample*/

	ITG3200_DLP_CFG_TypeDef ITG3200_DigitalLowPassFilterCfg; /*!<Register 0x16 (22) –
										DLPF, Full Scale
										The DLPF_CFG parameter sets the digital
										low pass filter configuration. It also determines
										the internal sampling rate used by the device*/

	uint8_t ITG3200_IrqConfig; /*!<Register 0x17 (23) – Interrupt Configuration
								This register configures the interrupt operation of
								the device. The interrupt output pin (INT) configuration
								can be set, the interrupt latching/clearing method can be
								set, and the triggers for the interrupt can be set.
								Note that if the application requires reading every
								sample of data from the ITG-3200 part, it is best to
								enable the raw data ready interrupt (RAW_RDY_EN). This
								allows the application to know when new sample data is available.
								Parameters:
								ACTL Logic level for INT output pin – 1=active low, 0=active high
								OPEN Drive type for INT output pin – 1=open drain, 0=push-pull
								LATCH_INT_EN Latch mode – 1=latch until interrupt is cleared, 0=50us pulse
								INT_ANYRD_2CLEAR Latch clear method – 1=any register read, 0=status register read only
								ITG_RDY_EN Enable interrupt when device is ready (PLL ready after changing clock source)
								RAW_RDY_EN Enable interrupt when data is available
								0 Load zeros into Bits 1 and 3 of the Interrupt Configuration register.
								Bit7	Bit6	Bit5			Bit4
								ACTL	OPEN	LATCH_INT_EN	INT_ANYRD_2CLEAR
								Bit3	Bit2		Bit1	Bit0
								0		ITG_RDY_EN	0		RAW_RDY_EN
								*/

	uint8_t ITG3200_PowerControl; /*!<Register 0x3E (62) – Power Management
					This register is used to manage the power control,
					select the clock source, and to issue a master reset to the device.
					Setting the SLEEP bit in the register puts the device into very low
					power sleep mode. In this mode, only the serial interface and internal
					registers remain active, allowing for a very low standby current.
					Clearing this bit puts the device back into normal mode. To save
					power, the individual standby selections for each of the gyros should
					be used if any gyro axis is not used by the application.
					The CLK_SEL setting determines the device clock source as follows:
					CLK_SEL CLK_SEL		Clock Source
					0					Internal oscillator
					1					PLL with X Gyro reference
					2					PLL with Y Gyro reference
					3					PLL with Z Gyro reference
					4					PLL with external 32.768kHz reference
					5					PLL with external 19.2MHz reference
					6					Reserved
					7					Reserved
					On power up, the ITG-3200 defaults to the internal oscillator. It is
					highly recommended that the device is configured to use one of the
					gyros (or an external clock) as the clock reference, due to the
					improved stability.
					Bit7	Bit6	Bit5		Bit4		Bit3		|Bit2 Bit1 Bit0
					H_RESET	SLEEP	STBY _XG	STBY _YG	STBY _ZG	|CLK_SEL
					*/

} ITG3200_InitTypeDef;

uint8_t ITG3200_ReadDevID();
uint8_t ITG3200_ReadRegister(uint8_t RegisterToRead);
void ITG3200_WriteRegister(uint8_t reg, uint8_t value);
void ITG3200_GetMeasurements(int16_t *values);
void ITG3200_Init(ITG3200_InitTypeDef* ITG3200_InitStructure);
void ITG3200_ReadGyro(float *_GyroX, float *_GyroY, float *_GyroZ);
void ITG3200_SetRevPolarity (uint8_t _Xpol, uint8_t _Ypol, uint8_t _Zpol);
void ITG3200_SetGains(float _Xgain, float _Ygain, float _Zgain);
void ITG3200_SetOffsets(int _Xoffset, int _Yoffset, int _Zoffset);
void ITG3200_ReadGyroRawCal(int *_GyroX, int *_GyroY, int *_GyroZ);
void ITG3200_ReadGyroRaw(int *_GyroX, int *_GyroY, int *_GyroZ);
void ITG3200_ZeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS);

float gains[3];
int offsets[3];
float polarities[3];

#ifdef	__cplusplus
}
#endif

#endif	/* ITG3200_H */

