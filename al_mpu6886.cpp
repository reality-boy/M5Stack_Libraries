#include <math.h>
#include <Arduino.h>

//****FixMe, #include "MahonyAHRS.h"
#include "AL_MPU6886.h"

#define MPU6886_ADDRESS            0x68

#define MPU6886_XG_OFFS_TC_H       0x04
#define MPU6886_XG_OFFS_TC_L       0x05
#define MPU6886_YG_OFFS_TC_H       0x07
#define MPU6886_YG_OFFS_TC_L       0x08
#define MPU6886_ZG_OFFS_TC_H       0x0A
#define MPU6886_ZG_OFFS_TC_L       0x0B
#define MPU6886_SELF_TEST_X_ACCEL  0x0D
#define MPU6886_SELF_TEST_Y_ACCEL  0x0E
#define MPU6886_SELF_TEST_Z_ACCEL  0x0F
#define MPU6886_XG_OFFS_USRH       0x13 // GYRO_OFFSET
#define MPU6886_XG_OFFS_USRL       0x14
#define MPU6886_YG_OFFS_USRH       0x15
#define MPU6886_YG_OFFS_USRL       0x16
#define MPU6886_ZG_OFFS_USRH       0x17
#define MPU6886_ZG_OFFS_USRL       0x18
#define MPU6886_SMPLRT_DIV         0x19
#define MPU6886_CONFIG             0x1A
#define MPU6886_GYRO_CONFIG        0x1B
#define MPU6886_ACCEL_CONFIG       0x1C
#define MPU6886_ACCEL_CONFIG2      0x1D
#define MPU6886_LP_MODE_CFG        0x1E
#define MPU6886_ACCEL_WOM_X_THR    0x20
#define MPU6886_ACCEL_WOM_Y_THR    0x21
#define MPU6886_ACCEL_WOM_Z_THR    0x22
#define MPU6886_FIFO_EN            0x23
#define MPU6886_FSYNC_INT          0x36
#define MPU6886_INT_PIN_CFG        0x37
#define MPU6886_INT_ENABLE         0x38
#define MPU6886_FIFO_WM_INT_STATUS 0x39
#define MPU6886_INT_STATUS         0x3A
#define MPU6886_ACCEL_XOUT_H       0x3B
#define MPU6886_ACCEL_XOUT_L       0x3C
#define MPU6886_ACCEL_YOUT_H       0x3D
#define MPU6886_ACCEL_YOUT_L       0x3E
#define MPU6886_ACCEL_ZOUT_H       0x3F
#define MPU6886_ACCEL_ZOUT_L       0x40
#define MPU6886_TEMP_OUT_H         0x41
#define MPU6886_TEMP_OUT_L         0x42
#define MPU6886_GYRO_XOUT_H        0x43
#define MPU6886_GYRO_XOUT_L        0x44
#define MPU6886_GYRO_YOUT_H        0x45
#define MPU6886_GYRO_YOUT_L        0x46
#define MPU6886_GYRO_ZOUT_H        0x47
#define MPU6886_GYRO_ZOUT_L        0x48
#define MPU6886_SELF_TEST_X_GYRO   0x50
#define MPU6886_SELF_TEST_Y_GYRO   0x51
#define MPU6886_SELF_TEST_Z_GYRO   0x52
#define MPU6886_E_ID0              0x53
#define MPU6886_E_ID1              0x54
#define MPU6886_E_ID2              0x55
#define MPU6886_E_ID3              0x56
#define MPU6886_E_ID4              0x57
#define MPU6886_E_ID5              0x58
#define MPU6886_E_ID6              0x59
#define MPU6886_FIFO_WM_TH1        0x60
#define MPU6886_FIFO_WM_TH2        0x61
#define MPU6886_SIGNAL_PATH_RESET  0x68
#define MPU6886_ACCEL_INTEL_CTRL   0x69
#define MPU6886_USER_CTRL          0x6A
#define MPU6886_PWR_MGMT_1         0x6B
#define MPU6886_PWR_MGMT_2         0x6C
#define MPU6886_I2C_IF             0x70
#define MPU6886_FIFO_COUNTH        0x72
#define MPU6886_FIFO_COUNTL        0x73
#define MPU6886_FIFO_R_W           0x74
#define MPU6886_WHOAMI             0x75
#define MPU6886_XA_OFFSET_H        0x77
#define MPU6886_XA_OFFSET_L        0x78
#define MPU6886_YA_OFFSET_H        0x7A
#define MPU6886_YA_OFFSET_L        0x7B
#define MPU6886_ZA_OFFSET_H        0x7D
#define MPU6886_ZA_OFFSET_L        0x7E

//****FixMe, if these defines are usefull then make them constants in the class
//#define G 9.8
//#define RtA 57.324841
//#define AtR 0.0174533
//#define Gyro_Gr 0.0010653

AL_MPU6886::AL_MPU6886()
{
  for (int i = 0; i < 3; i++)
  {
    m_accelAddCF[i] = 0;
    m_accelMultCF[i] = 1;
  }
}

void AL_MPU6886::I2C_Read_NBytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer)
{
  Wire1.beginTransmission(MPU6886_ADDRESS);
  Wire1.write(start_Addr);
  Wire1.endTransmission(false);

  //! Put read results in the Rx buffer
  Wire1.requestFrom((uint8_t)MPU6886_ADDRESS, number_Bytes);
  uint8_t i = 0;
  while (Wire1.available() && i < number_Bytes)
    read_Buffer[i++] = Wire1.read();
}

void AL_MPU6886::I2C_Write_NBytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer)
{
  Wire1.beginTransmission(MPU6886_ADDRESS);
  Wire1.write(start_Addr);
  Wire1.write(*write_Buffer);
  Wire1.endTransmission();
}

int AL_MPU6886::Init(void)
{
  int ret = -1;
  unsigned char tempdata[1];
  unsigned char regdata;

  Wire1.begin(21, 22);

  I2C_Read_NBytes(MPU6886_WHOAMI, 1, tempdata);
  m_imuId = tempdata[0];
  delay(1);

  // check that the chip initialized correctly
  if (tempdata[0] == 0x19)
  {

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = (0x01 << 7);
    I2C_Write_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = (0x01 << 0);
    I2C_Write_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    // +- 8g
    regdata = 0x10;
    I2C_Write_NBytes(MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(1);

    // +- 2000 dps
    regdata = 0x18;
    I2C_Write_NBytes(MPU6886_GYRO_CONFIG, 1, &regdata);
    delay(1);

    // 1khz output
    regdata = 0x01;
    I2C_Write_NBytes(MPU6886_CONFIG, 1, &regdata);
    delay(1);

    // 2 div, FIFO 500hz out
#if 0
    regdata = 0x05;
#else
    regdata = 0x01;
#endif
    I2C_Write_NBytes(MPU6886_SMPLRT_DIV, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_INT_ENABLE, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ACCEL_CONFIG2, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_USER_CTRL, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_FIFO_EN, 1, &regdata);
    delay(1);

    regdata = 0x22;
    I2C_Write_NBytes(MPU6886_INT_PIN_CFG, 1, &regdata);
    delay(1);

    regdata = 0x01;
    I2C_Write_NBytes(MPU6886_INT_ENABLE, 1, &regdata);

    delay(100);

    setGyroFsr(m_gscale);
    setAccelFsr(m_ascale);

    ret = 0;
  }

  return ret;
}

//****FixMe,
#if 0
void AL_MPU6886::getAhrsData(float *pitch, float *roll, float *yaw)
{
  float accX = 0;
  float accY = 0;
  float accZ = 0;

  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;


  getGyroData(&gyroX, &gyroY, &gyroZ);
  getAccelData(&accX, &accY, &accZ);

  MahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, pitch, roll, yaw);
}
#endif


// Possible gyro scales (and their register bit settings)
void AL_MPU6886::updateGres()
{
  switch (m_gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    case GFS_250DPS:
      m_gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      m_gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      m_gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      m_gRes = 2000.0 / 32768.0;
      break;
  }
}

void AL_MPU6886::updateAres()
{
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
  // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  switch (m_ascale)
  {
    case AFS_2G:
      m_aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      m_aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      m_aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      m_aRes = 16.0 / 32768.0;
      break;
  }
}

void AL_MPU6886::setGyroFsr(Gscale scale)
{
  //return IIC_Write_Byte(MPU_GYRO_CFG_REG,scale<<3);
  unsigned char regdata;
  regdata = (scale << 3);
  I2C_Write_NBytes(MPU6886_GYRO_CONFIG, 1, &regdata);
  delay(10);

  m_gscale = scale;
  updateGres();
}

void AL_MPU6886::setAccelFsr(Ascale scale)
{
  unsigned char regdata;
  regdata = (scale << 3);
  I2C_Write_NBytes(MPU6886_ACCEL_CONFIG, 1, &regdata);
  delay(10);

  m_ascale = scale;
  updateAres();
}

void AL_MPU6886::getAccelData(float* ax, float* ay, float* az)
{
  uint8_t buf[6];
  I2C_Read_NBytes(MPU6886_ACCEL_XOUT_H, 6, buf);

  int16_t accX = ((int16_t)buf[0] << 8) | buf[1];
  int16_t accY = ((int16_t)buf[2] << 8) | buf[3];
  int16_t accZ = ((int16_t)buf[4] << 8) | buf[5];

  *ax = applyAccelCF((float)accX * m_aRes, 0);
  *ay = applyAccelCF((float)accY * m_aRes, 1);
  *az = applyAccelCF((float)accZ * m_aRes, 2);
}

void AL_MPU6886::getGyroData(float* gx, float* gy, float* gz)
{
  uint8_t buf[6];
  I2C_Read_NBytes(MPU6886_GYRO_XOUT_H, 6, buf);

  int16_t gyroX = ((uint16_t)buf[0] << 8) | buf[1];
  int16_t gyroY = ((uint16_t)buf[2] << 8) | buf[3];
  int16_t gyroZ = ((uint16_t)buf[4] << 8) | buf[5];

  *gx = (float)gyroX * m_gRes;
  *gy = (float)gyroY * m_gRes;
  *gz = (float)gyroZ * m_gRes;
}

void AL_MPU6886::setGyroOffset(uint16_t x, uint16_t y, uint16_t z)
{
  uint8_t buf_out[6];
  buf_out[0] = x >> 8;
  buf_out[1] = x & 0xff;
  buf_out[2] = y >> 8;
  buf_out[3] = y & 0xff;
  buf_out[4] = z >> 8;
  buf_out[5] = z & 0xff;
  I2C_Write_NBytes(MPU6886_XG_OFFS_USRH, 6, buf_out);
}

// limited correction factor, applied in chip
void AL_MPU6886::setAccelOffset(float ax, float ay, float az)
{
  //****FixMe, what is the scale here?
  //±16g Offset cancellation in all Full Scale modes, 15 bit 0.98-mg steps.
  uint16_t x = (uint16_t)ax / m_aRes;
  uint16_t y = (uint16_t)ay / m_aRes;
  uint16_t z = (uint16_t)az / m_aRes;

  uint8_t buf_out[6];
  buf_out[0] = x >> 8;
  buf_out[1] = x & 0xff;
  buf_out[2] = y >> 8;
  buf_out[3] = y & 0xff;
  buf_out[4] = z >> 8;
  buf_out[5] = z & 0xff;
  I2C_Write_NBytes(MPU6886_XA_OFFSET_H, 6, buf_out);
}

// more complete correction facter applied after reading
void AL_MPU6886::setAccelCF(const float addCF[3], const float multCF[3])
{
  for (int i = 0; i < 3; i++)
  {
    m_accelAddCF[i] = addCF[i];
    m_accelMultCF[i] = multCF[i];
  }
}

// apply correction to raw signal in g
float AL_MPU6886::applyAccelCF(float raw, int axis)
{
  return (raw + m_accelAddCF[axis]) * m_accelMultCF[axis];
}



void AL_MPU6886::getTempData(float *t)
{
  uint8_t buf[2];
  I2C_Read_NBytes(MPU6886_TEMP_OUT_H, 2, buf);

  int16_t temp = ((uint16_t)buf[0] << 8) | buf[1];

  *t = (float)temp / 326.8 + 25.0;
}

//---

void AL_MPU6886::enableWakeOnMotion(Ascale ascale, uint8_t thresh_num_lsb)
{
  uint8_t regdata;
  /* 5.1 WAKE-ON-MOTION INTERRUPT
      The MPU-6886 provides motion detection capability. A qualifying motion sample is one where the high passed sample
      from any axis has an absolute value exceeding a user-programmable threshold. The following steps explain how to
      configure the Wake-on-Motion Interrupt.
  */

  /* Step 0: this isn't explicitly listed in the steps, but configuring the
     FSR or full-scale-range of the accelerometer is important to setting up
     the accel/motion threshold in Step 4
  */
  regdata = (ascale << 3);
  I2C_Write_NBytes(MPU6886_ACCEL_CONFIG, 1, &regdata);
  delay(10);

  /* Step 1: Ensure that Accelerometer is running
      • In PWR_MGMT_1 register (0x6B) set CYCLE = 0, SLEEP = 0, and GYRO_STANDBY = 0
      • In PWR_MGMT_2 register (0x6C) set STBY_XA = STBY_YA = STBY_ZA = 0, and STBY_XG = STBY_YG = STBY_ZG = 1
  */
  I2C_Read_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);
  regdata = regdata & 0b10001111; // set cyle, sleep, and gyro to standby, i.e. 0
  I2C_Write_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);

  regdata = 0b00000111; // set accel x, y, and z to standby
  I2C_Write_NBytes(MPU6886_PWR_MGMT_2, 1, &regdata);

  /* Step 2: Set Accelerometer LPF bandwidth to 218.1 Hz
      • In ACCEL_CONFIG2 register (0x1D) set ACCEL_FCHOICE_B = 0 and A_DLPF_CFG[2:0] = 1 (b001)
  */
  I2C_Read_NBytes(MPU6886_ACCEL_CONFIG2, 1, &regdata);
  regdata = 0b00100001; // average 32 samples, use 218 Hz DLPF
  I2C_Write_NBytes(MPU6886_ACCEL_CONFIG2, 1, &regdata);

  /* Step 2.5 - active low? */
  I2C_Read_NBytes(MPU6886_INT_PIN_CFG, 1, &regdata);
  regdata =  ((regdata | 0b10000000) & 0b11011111); // configure pin active-low, no latch
  I2C_Write_NBytes(MPU6886_INT_PIN_CFG, 1, &regdata);

  /* Step 3: Enable Motion Interrupt
      • In INT_ENABLE register (0x38) set WOM_INT_EN = 111 to enable motion interrupt
  */
  regdata = 0b11100000; // enable wake-on-motion interrupt for X, Y, and Z axes
  I2C_Write_NBytes(MPU6886_INT_ENABLE, 1, &regdata);

  /* Step 4: Set Motion Threshold
      • Set the motion threshold in ACCEL_WOM_THR register (0x1F)
      NOTE: the data sheet mentions 0x1F, but is probably referring to
            registers 0x20, 0x21, and 0x22 based on empirical tests
  */
  regdata = thresh_num_lsb; // set accel motion threshold for X, Y, and Z axes
  I2C_Write_NBytes(MPU6886_ACCEL_WOM_X_THR, 1, &regdata);
  I2C_Write_NBytes(MPU6886_ACCEL_WOM_Y_THR, 1, &regdata);
  I2C_Write_NBytes(MPU6886_ACCEL_WOM_Z_THR, 1, &regdata);

  /* Step 5: Enable Accelerometer Hardware Intelligence
      • In ACCEL_INTEL_CTRL register (0x69) set ACCEL_INTEL_EN = ACCEL_INTEL_MODE = 1;
        Ensure that bit 0 is set to 0
  */
  regdata = 0b11000010; // enable wake-on-motion if any of X, Y, or Z axes is above threshold
  // WOM_STEP5_ACCEL_INTEL_CTRL_INTEL_EN_1_MODE_1_WOM_TH_MODE_0;
  I2C_Write_NBytes(MPU6886_ACCEL_INTEL_CTRL, 1, &regdata);

  /* Step 7: Set Frequency of Wake-Up
      • In SMPLRT_DIV register (0x19) set SMPLRT_DIV[7:0] = 3.9 Hz – 500 Hz
  */
  // sample_rate = 1e3 / (1 + regdata)
  //   4.0 Hz = 1e3 / (1 + 249)
  //  10.0 Hz = 1e3 / (1 +  99)
  //  20.0 Hz = 1e3 / (1 +  49)
  //  25.0 Hz = 1e3 / (1 +  39)
  //  50.0 Hz = 1e3 / (1 +  19) <----
  // 500.0 Hz = 1e3 / (1 +   1)
  regdata = 19;
  I2C_Write_NBytes(MPU6886_SMPLRT_DIV, 1, &regdata);

  /* Step 8: Enable Cycle Mode (Accelerometer Low-Power Mode)
      • In PWR_MGMT_1 register (0x6B) set CYCLE = 1
  */
  I2C_Read_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);
  regdata = regdata | 0b00100000; // enable accelerometer low-power mode
  I2C_Write_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);
}

void AL_MPU6886::setINTPinActiveLogic(uint8_t level)
{
  uint8_t tempdata;
  I2C_Read_NBytes(MPU6886_INT_PIN_CFG, 1, &tempdata);
  tempdata &= 0x7f;
  tempdata |= level ? 0x00 : (0x01 << 7);
  I2C_Write_NBytes(MPU6886_INT_PIN_CFG, 1, &tempdata);
}

void AL_MPU6886::disableAllIRQ()
{
  uint8_t tempdata = 0x00;
  I2C_Write_NBytes(MPU6886_INT_ENABLE, 1, &tempdata);
  I2C_Read_NBytes(MPU6886_INT_PIN_CFG, 1, &tempdata);
  tempdata |= 0x01 << 6;
  // int pin is configured as open drain
  I2C_Write_NBytes(MPU6886_INT_PIN_CFG, 1, &tempdata);
}

void AL_MPU6886::clearAllIRQ()
{
  uint8_t tempdata = 0x00;
  I2C_Read_NBytes(MPU6886_FIFO_WM_INT_STATUS, 1, &tempdata);
  I2C_Read_NBytes(MPU6886_INT_STATUS, 1, &tempdata);
}

//---

void AL_MPU6886::setFIFOEnable(bool gyroEnable, bool accelEnable)
{
  m_fifoGyroEnabled = gyroEnable;
  m_fifoAccelEnabled = accelEnable;

  // select data registers to write to fifo
  // TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
  const int GYRO_FIFO_EN = 0x10; // 8 bytes
  //ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L
  const int ACCEL_FIFO_EN = 0x08; // 8 bytes
  //****Note, if both enabled // 14 bytes
  // ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L

  uint8_t regdata = 0;
  if (gyroEnable) regdata |= GYRO_FIFO_EN;
  if (accelEnable) regdata |= ACCEL_FIFO_EN;
  I2C_Write_NBytes(MPU6886_FIFO_EN, 1, &regdata);

  // enable fifo and clear buffer
  const int FIFO_EN = 0x40;
  const int FIFO_RST = 0x04;

  regdata = (gyroEnable || accelEnable) ? (FIFO_EN | FIFO_RST) : 0x00;
  I2C_Write_NBytes(MPU6886_USER_CTRL, 1, &regdata);
}

void AL_MPU6886::resetFIFO()
{
  const int FIFO_RST = 0x04;

  uint8_t buf_out;
  I2C_Read_NBytes(MPU6886_USER_CTRL, 1, &buf_out);
  buf_out |= FIFO_RST;
  I2C_Write_NBytes(MPU6886_USER_CTRL, 1, &buf_out);
}

// bulk read from fifo in blocks of 210 (why 210?)
int AL_MPU6886::readFIFOBuff(uint8_t *buff, uint16_t maxLen)
{
  int count = readFIFOCount();

  // dont' read more than will fit in the buffer
  if (count > maxLen)
    count = maxLen;

  //****Note, can't take too long reading data or the fifo can't write in new data
  // tweak this as needed to get a stable reading.
  int maxBlock = getFIFORecordSize() * 15;

  if (count > 0)
  {
    uint8_t number = count / maxBlock;
    for (uint8_t i = 0; i < number; i++)
      I2C_Read_NBytes(MPU6886_FIFO_R_W, maxBlock, &buff[i * maxBlock]);

    I2C_Read_NBytes(MPU6886_FIFO_R_W, count % maxBlock, &buff[number * maxBlock]);
  }

  return count;
}

uint16_t AL_MPU6886::readFIFOCount()
{
  uint8_t buf[2];
  I2C_Read_NBytes(MPU6886_FIFO_COUNTH, 2, buf);
  uint16_t count = ((uint16_t)buf[0] << 8) | buf[1];
  return count;
}

void AL_MPU6886::parseFIFO_accel(const uint8_t *buf, float *ax, float *ay, float *az)
{

  if (m_fifoGyroEnabled && m_fifoAccelEnabled)
  {
    // full 14 byte record
    // ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
    int16_t accX = ((int16_t)buf[0] << 8) | buf[1];
    int16_t accY = ((int16_t)buf[2] << 8) | buf[3];
    int16_t accZ = ((int16_t)buf[4] << 8) | buf[5];
    *ax = (float)accX * m_aRes;
    *ay = (float)accY * m_aRes;
    *az = (float)accZ * m_aRes;
  }
  else if (m_fifoAccelEnabled)
  {
    // truncated 8 byte accel record
    //ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L
    int16_t accX = ((int16_t)buf[0] << 8) | buf[1];
    int16_t accY = ((int16_t)buf[2] << 8) | buf[3];
    int16_t accZ = ((int16_t)buf[4] << 8) | buf[5];
    *ax = (float)accX * m_aRes;
    *ay = (float)accY * m_aRes;
    *az = (float)accZ * m_aRes;
  }
  else
  {
    // only gyro, or no record
    // TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
    *ax = 0;
    *ay = 0;
    *az = 0;
  }
}

void AL_MPU6886::parseFIFO_gyro(const uint8_t *buf, float *gx, float *gy, float *gz)
{
  if (m_fifoGyroEnabled && m_fifoAccelEnabled)
  {
    // full 14 byte record
    // ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
    int16_t gyroX = ((int16_t)buf[8] << 8) | buf[9];
    int16_t gyroY = ((int16_t)buf[10] << 8) | buf[11];
    int16_t gyroZ = ((int16_t)buf[12] << 8) | buf[13];
    *gx = (float)gyroX * m_gRes;
    *gy = (float)gyroY * m_gRes;
    *gz = (float)gyroZ * m_gRes;
  }
  else if (m_fifoAccelEnabled)
  {
    // truncated 8 byte accel record
    //ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L
    *gx = 0;
    *gy = 0;
    *gz = 0;
  }
  else
  {
    // only gyro, or no record
    // TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
    int16_t gyroX = ((int16_t)buf[2] << 8) | buf[3];
    int16_t gyroY = ((int16_t)buf[4] << 8) | buf[5];
    int16_t gyroZ = ((int16_t)buf[6] << 8) | buf[7];
    *gx = (float)gyroX * m_gRes;
    *gy = (float)gyroY * m_gRes;
    *gz = (float)gyroZ * m_gRes;
  }
}

void AL_MPU6886::parseFIFO_temp(const uint8_t *buf, float *t)
{
  if (m_fifoGyroEnabled && m_fifoAccelEnabled)
  {
    // full 14 byte record
    // ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
    int16_t temp = ((int16_t)buf[6] << 8) | buf[7];
    *t = (float)temp / 326.8 + 25.0;
  }
  else if (m_fifoAccelEnabled)
  {
    // truncated 8 byte accel record
    //ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L
    int16_t temp = ((int16_t)buf[6] << 8) | buf[7];
    *t = (float)temp / 326.8 + 25.0;
  }
  else
  {
    // only gyro, or no record
    // TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
    int16_t temp = ((int16_t)buf[0] << 8) | buf[1];
    *t = (float)temp / 326.8 + 25.0;
  }
}

int AL_MPU6886::getFIFORecordSize()
{
  if (m_fifoGyroEnabled && m_fifoAccelEnabled)
    return 14;
  else if (m_fifoGyroEnabled)
    return 8;
  else if (m_fifoAccelEnabled)
    return 8;
  else
    return 0;
}

//---

void AL_MPU6886::speedUpSampleRate()
{
  uint8_t regdata;

  // Boost the i2c update rate
  // Possible speeds
  //  100000 or 100 KHz, i2c and arduino default clock
  //  400000 or 400 KHz, i2c fast mode
  //  1000000 or 1 MHz, i2c fast mode plus
  //  3400000 or 3.4 MHz, i2c high speed mode
  //  10000000 or 10 MHZ, upper limit on MPU6886 chip
  //****Note, esp32 limits us to 1 MHz clock in esp32-hal-i2c.c i2cSetClock() function
  // being able to run faster may allow us to retrive more data from the fifo buffer without having to break up the reads
  Wire1.setClock(1000000); // 1 MHz i2c fast mode plus

  // Try to run at a 1 KHz sample rate
  //****Note, need at least 1 divider to make FIFO stable, not needed if polling
  //****Note, upping the average reduces the peak magnitude when recording a sine wave.

  // up the sample rate
  //regdata = 0x00; // 4 sample average, 1 kHz sample rate, 218 Hz lowpass
  regdata = 0x07; // 4 sample average, 1 kHz sample rate, 420 Hz lowpass
  //regdata = 0x08; // 1 sample average, 4 kHz sample rate, 1100 Hz lowpass
  I2C_Write_NBytes(MPU6886_ACCEL_CONFIG2, 1, &regdata);

  // turn off sample rate divider
  //regdata = 0x03; // /4 divider
  regdata = 0x00; // /1 divider
  I2C_Write_NBytes(MPU6886_SMPLRT_DIV, 1, &regdata);
}
