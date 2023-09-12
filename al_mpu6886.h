// Based on the M5StickC version at https://github.com/m5stack/M5StickC/tree/master/src/utility
// but heavily modified by David Tucker 2022

#ifndef _AL_MPU6886_H_
#define _AL_MPU6886_H_

#include <Wire.h>
#include <Arduino.h>

class AL_MPU6886
{
  public:
    enum Ascale { AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G };
    enum Gscale { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };

  public:
    AL_MPU6886();
    int Init(void);

    void getAccelData(float *ax, float *ay, float *az);
    void getGyroData(float *gx, float *gy, float *gz);
    void getTempData(float *t);

    void setGyroFsr(Gscale scale);
    void setAccelFsr(Ascale scale);

    void setGyroOffset(uint16_t gx, uint16_t gy, uint16_t gz);
    void setAccelOffset(float ax, float ay, float az);
    void setAccelCF(const float addCF[3], const float multCF[3]);

    //void getAhrsData(float *pitch, float *roll, float *yaw);

    void enableWakeOnMotion(Ascale ascale, uint8_t thresh_num_lsb);
    void setINTPinActiveLogic(uint8_t level);
    void disableAllIRQ();
    void clearAllIRQ();

    // init and reset fifo
    void setFIFOEnable(bool gyroEnable, bool accelEnable);
    // clear fifo out
    void resetFIFO();
    // read up to maxLen of data, returns actual count read
    //****Note, we assume this is a multiple of getFIFORecordSize()
    int readFIFOBuff(uint8_t *buf, uint16_t maxLen);
    // returns count of data in fifo
    uint16_t readFIFOCount();
    // how many bytes make up one record in the fifo (14, 8, etc)
    int getFIFORecordSize();
    // parse accelerometer data from fifo record
    void parseFIFO_accel(const uint8_t *buf, float *ax, float *ay, float *az);
    void parseFIFO_gyro(const uint8_t *buf, float *gx, float *gy, float *gz);
    void parseFIFO_temp(const uint8_t *buf, float *t);

    void speedUpSampleRate();

    // maximum amount of data in bytes that the fifo can hold
    const static int MAX_FIFO = 1024;

  private:
    void I2C_Read_NBytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer);
    void I2C_Write_NBytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);
    void updateGres();
    void updateAres();
    float applyAccelCF(float raw, int axis);


    Gscale m_gscale = GFS_2000DPS;
    Ascale m_ascale = AFS_8G;
    float m_aRes;
    float m_gRes;
    float m_imuId;
    bool m_fifoGyroEnabled;
    bool m_fifoAccelEnabled;

    float m_accelAddCF[3];
    float m_accelMultCF[3];
};
#endif //_AL_MPU6886_H_
