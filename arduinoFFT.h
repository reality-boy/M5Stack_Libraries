/*

   FFT library
   Copyright (C) 2010 Didier Longueville
   Copyright (C) 2014 Enrique Condes
   Modified by David Tucker 2022

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef arduinoFFT_h /* Prevent loading library twice */
#define arduinoFFT_h
#ifdef ARDUINO
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h" /* This is where the standard Arduino code lies */
#endif
#else
#include <stdlib.h>
#include <stdio.h>
#ifdef __AVR__
#include <avr/io.h>
#include <avr/pgmspace.h>
#endif
#include <math.h>
#include "defs.h"
#include "types.h"
#endif

#define FFT_LIB_REV 0x14
/* Custom constants */
#define FFT_FORWARD 0x01
#define FFT_REVERSE 0x00

/* Windowing type */
#define FFT_WIN_TYP_RECTANGLE 0x00 /* rectangle (Box car) */
#define FFT_WIN_TYP_HAMMING 0x01 /* hamming */
#define FFT_WIN_TYP_HANN 0x02 /* hann */
#define FFT_WIN_TYP_TRIANGLE 0x03 /* triangle (Bartlett) */
#define FFT_WIN_TYP_NUTTALL 0x04 /* nuttall */
#define FFT_WIN_TYP_BLACKMAN 0x05 /* blackman */
#define FFT_WIN_TYP_BLACKMAN_NUTTALL 0x06 /* blackman nuttall */
#define FFT_WIN_TYP_BLACKMAN_HARRIS 0x07 /* blackman harris*/
#define FFT_WIN_TYP_FLT_TOP 0x08 /* flat top */
#define FFT_WIN_TYP_WELCH 0x09 /* welch */
/*Mathematial constants*/

#ifdef __AVR__
static const float _c1[]PROGMEM = {0.0000000000, 0.7071067812, 0.9238795325, 0.9807852804,
                                   0.9951847267, 0.9987954562, 0.9996988187, 0.9999247018,
                                   0.9999811753, 0.9999952938, 0.9999988235, 0.9999997059,
                                   0.9999999265, 0.9999999816, 0.9999999954, 0.9999999989,
                                   0.9999999997
                                  };
static const float _c2[]PROGMEM = {1.0000000000, 0.7071067812, 0.3826834324, 0.1950903220,
                                   0.0980171403, 0.0490676743, 0.0245412285, 0.0122715383,
                                   0.0061358846, 0.0030679568, 0.0015339802, 0.0007669903,
                                   0.0003834952, 0.0001917476, 0.0000958738, 0.0000479369,
                                   0.0000239684
                                  };
#endif

class arduinoFFT
{
  public:
    arduinoFFT(); // must call updateDataSet() in this case
    arduinoFFT(float *vReal, float *vImag, uint16_t samples, float *windowWeighingFactors = NULL);
    ~arduinoFFT(void);

    void updateDataSet(float *vReal, float *vImag, uint16_t samples, float *windowWeighingFactors = NULL);

    uint8_t Revision(void);

    void DCRemoval(float *vData, uint16_t samples);

    void ComplexToMagnitude();
    void Compute(uint8_t dir);
    void DCRemoval();
    void Windowing(uint8_t windowType, uint8_t dir, bool withCompensation = false);

  private:
    uint16_t _samples;
    float *_vReal;
    float *_vImag;
    uint8_t _power;

    float *_windowWeighingFactors;
    uint8_t _weighingFactorsFFTWindow; // cache of windowType
    bool _weighingFactorsWithCompensation;
    bool _weighingFactorsComputed;

    constexpr static const float twoPi  =  6.28318531f;
    constexpr static const float fourPi = 12.56637061f;
    constexpr static const float sixPi  = 18.84955593f;

    static const float _WindowCompensationFactors[10];

    uint8_t Exponent(uint16_t value);
    void Swap(float *x, float *y);
};

//---------------

arduinoFFT::arduinoFFT()
{
  //****Note, need to call updateDataSet() once to initialize the library
}

arduinoFFT::arduinoFFT(float *vReal, float *vImag, uint16_t samples, float *windowWeighingFactors)
{
  updateDataSet(vReal, vImag, samples, windowWeighingFactors);
}

arduinoFFT::~arduinoFFT(void)
{
  // Destructor
}

void arduinoFFT::updateDataSet(float *vReal, float *vImag, uint16_t samples, float *windowWeighingFactors)
{
  this->_vReal = vReal;
  this->_vImag = vImag;
  this->_samples = samples;
  this->_power = Exponent(samples);

  this->_windowWeighingFactors = windowWeighingFactors;
  this->_weighingFactorsWithCompensation = false;
  this->_weighingFactorsComputed = false;
}

uint8_t arduinoFFT::Revision(void)
{
  return (FFT_LIB_REV);
}

void arduinoFFT::Compute(uint8_t dir)
{ // Computes in-place complex-to-complex FFT /
  // Reverse bits /
  uint16_t j = 0;
  for (uint16_t i = 0; i < (this->_samples - 1); i++)
  {
    if (i < j)
    {
      Swap(&this->_vReal[i], &this->_vReal[j]);
      if (dir == FFT_REVERSE)
        Swap(&this->_vImag[i], &this->_vImag[j]);
    }
    uint16_t k = (this->_samples >> 1);
    while (k <= j)
    {
      j -= k;
      k >>= 1;
    }
    j += k;
  }
  // Compute the FFT  /
#ifdef __AVR__
  uint8_t index = 0;
#endif
  float c1 = -1.0;
  float c2 = 0.0;
  uint16_t l2 = 1;
  for (uint8_t l = 0; (l < this->_power); l++)
  {
    uint16_t l1 = l2;
    l2 <<= 1;
    float u1 = 1.0;
    float u2 = 0.0;
    for (j = 0; j < l1; j++)
    {
      for (uint16_t i = j; i < this->_samples; i += l2)
      {
        uint16_t i1 = i + l1;
        float t1 = u1 * this->_vReal[i1] - u2 * this->_vImag[i1];
        float t2 = u1 * this->_vImag[i1] + u2 * this->_vReal[i1];
        this->_vReal[i1] = this->_vReal[i] - t1;
        this->_vImag[i1] = this->_vImag[i] - t2;
        this->_vReal[i] += t1;
        this->_vImag[i] += t2;
      }
      float z = ((u1 * c1) - (u2 * c2));
      u2 = ((u1 * c2) + (u2 * c1));
      u1 = z;
    }
#ifdef __AVR__
    c2 = pgm_read_float_near(&(_c2[index]));
    c1 = pgm_read_float_near(&(_c1[index]));
    index++;
#else
    c2 = sqrt((1.0 - c1) / 2.0);
    c1 = sqrt((1.0 + c1) / 2.0);
#endif
    if (dir == FFT_FORWARD) {
      c2 = -c2;
    }
  }
  // Scaling for reverse transform /
  if (dir != FFT_FORWARD) {
    for (uint16_t i = 0; i < this->_samples; i++) {
      this->_vReal[i] /= this->_samples;
      this->_vImag[i] /= this->_samples;
    }
  }
}

void arduinoFFT::ComplexToMagnitude()
{
  // vM is half the size of vReal and vImag
  for (uint16_t i = 0; i < this->_samples; i++)
  {
    this->_vReal[i] = sqrt(sq(this->_vReal[i]) + sq(this->_vImag[i])) * 2.0 / this->_samples;

    // above is calculating peak to peak magnitude, and we want amplitude
    this->_vReal[i] /= 2.0;
  }
}

void arduinoFFT::DCRemoval()
{
  // calculate the mean of vData
  float mean = 0;
  for (uint16_t i = 0; i < this->_samples; i++)
  {
    mean += this->_vReal[i];
  }
  mean /= this->_samples;
  // Subtract the mean from vData
  for (uint16_t i = 0; i < this->_samples; i++)
  {
    this->_vReal[i] -= mean;
  }
}

void arduinoFFT::DCRemoval(float *vData, uint16_t samples)
{
  // calculate the mean of vData
  float mean = 0;
  for (uint16_t i = 0; i < samples; i++)
  {
    mean += vData[i];
  }
  mean /= samples;
  // Subtract the mean from vData
  for (uint16_t i = 0; i < samples; i++)
  {
    vData[i] -= mean;
  }
}

void arduinoFFT::Windowing(uint8_t windowType, uint8_t dir, bool withCompensation)
{
  // check if values are already pre-computed for the correct window type and compensation
  if (_windowWeighingFactors && _weighingFactorsComputed &&
      _weighingFactorsFFTWindow == windowType &&
      _weighingFactorsWithCompensation == withCompensation)
  {
    // yes. values are precomputed
    if (dir == FFT_FORWARD)
    {
      for (uint_fast16_t i = 0; i < (this->_samples >> 1); i++)
      {
        this->_vReal[i] *= _windowWeighingFactors[i];
        this->_vReal[this->_samples - (i + 1)] *= _windowWeighingFactors[i];
      }
    }
    else
    {
      for (uint_fast16_t i = 0; i < (this->_samples >> 1); i++)
      {
        this->_vReal[i] /= _windowWeighingFactors[i];
        this->_vReal[this->_samples - (i + 1)] /= _windowWeighingFactors[i];
      }
    }
  }
  else
  {
    // no. values need to be pre-computed or applied
    // Weighing factors are computed once before multiple use of FFT
    // The weighing function is symmetric; half the weighs are recorded
    float samplesMinusOne = (float(this->_samples) - 1.0);
    float compensationFactor = _WindowCompensationFactors[windowType];
    for (uint16_t i = 0; i < (this->_samples >> 1); i++)
    {
      float indexMinusOne = float(i);
      float ratio = (indexMinusOne / samplesMinusOne);
      float weighingFactor = 1.0;
      // Compute and record weighting factor
      switch (windowType)
      {
        case FFT_WIN_TYP_RECTANGLE: // rectangle (box car)
          weighingFactor = 1.0;
          break;
        case FFT_WIN_TYP_HAMMING: // hamming
          weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
          break;
        case FFT_WIN_TYP_HANN: // hann
          weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
          break;
        case FFT_WIN_TYP_TRIANGLE: // triangle (Bartlett)
#if defined(ESP8266) || defined(ESP32)
          weighingFactor = 1.0 - ((2.0 * fabs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
#else
          weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
#endif
          break;
        case FFT_WIN_TYP_NUTTALL: // nuttall
          weighingFactor = 0.355768 - (0.487396 * (cos(twoPi * ratio))) + (0.144232 * (cos(fourPi * ratio))) - (0.012604 * (cos(sixPi * ratio)));
          break;
        case FFT_WIN_TYP_BLACKMAN: // blackman
          weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
          break;
        case FFT_WIN_TYP_BLACKMAN_NUTTALL: // blackman nuttall
          weighingFactor = 0.3635819 - (0.4891775 * (cos(twoPi * ratio))) + (0.1365995 * (cos(fourPi * ratio))) - (0.0106411 * (cos(sixPi * ratio)));
          break;
        case FFT_WIN_TYP_BLACKMAN_HARRIS: // blackman harris
          weighingFactor = 0.35875 - (0.48829 * (cos(twoPi * ratio))) + (0.14128 * (cos(fourPi * ratio))) - (0.01168 * (cos(sixPi * ratio)));
          break;
        case FFT_WIN_TYP_FLT_TOP: // flat top
          weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
          break;
        case FFT_WIN_TYP_WELCH: // welch
          weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
          break;
      }

      if (withCompensation)
      {
        weighingFactor *= compensationFactor;
      }

      if (_windowWeighingFactors)
      {
        _windowWeighingFactors[i] = weighingFactor;
      }

      if (dir == FFT_FORWARD)
      {
        this->_vReal[i] *= weighingFactor;
        this->_vReal[this->_samples - (i + 1)] *= weighingFactor;
      }
      else
      {
        this->_vReal[i] /= weighingFactor;
        this->_vReal[this->_samples - (i + 1)] /= weighingFactor;
      }

      // mark cached values as pre-computed
      _weighingFactorsFFTWindow = windowType;
      _weighingFactorsWithCompensation = withCompensation;
      _weighingFactorsComputed = true;
    }
  }
}

uint8_t arduinoFFT::Exponent(uint16_t value)
{
  // Calculates the base 2 logarithm of a value
  uint8_t result = 0;
  while (((value >> result) & 1) != 1) result++;
  return (result);
}

// Private functions

void arduinoFFT::Swap(float *x, float *y)
{
  float temp = *x;
  *x = *y;
  *y = temp;
}

const float arduinoFFT::_WindowCompensationFactors[10] =
{
  1.0000000000 * 2.0, // rectangle (Box car)
  1.8549343278 * 2.0, // hamming
  1.8554726898 * 2.0, // hann
  2.0039186079 * 2.0, // triangle (Bartlett)
  2.8163172034 * 2.0, // nuttall
  2.3673474360 * 2.0, // blackman
  2.7557840395 * 2.0, // blackman nuttall
  2.7929062517 * 2.0, // blackman harris
  3.5659039231 * 2.0, // flat top
  1.5029392863 * 2.0  // welch
};

#endif
