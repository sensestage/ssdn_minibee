// library modified from original adafruit library
// by nescivi, May 2016

/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#ifndef __BNO055_H__
#define __BNO055_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
 #include <TinyWireM.h>
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif

// #include <Adafruit_Sensor.h>
// #include <utility/imumaths.h>

#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)
#define BNO055_ID        (0xA0)

#define NUM_BNO055_OFFSET_REGISTERS (22)


/////////////// integer vector

template <uint8_t N> class IntVector
{
public:
    IntVector()
    {
        memset(p_vec, 0, sizeof(int)*N);
    }

    IntVector(int a)
    {
        memset(p_vec, 0, sizeof(int)*N);
        p_vec[0] = a;
    }

    IntVector(int a, int b)
    {
        memset(p_vec, 0, sizeof(int)*N);
        p_vec[0] = a;
        p_vec[1] = b;
    }

    IntVector(int a, int b, int c)
    {
        memset(p_vec, 0, sizeof(int)*N);
        p_vec[0] = a;
        p_vec[1] = b;
        p_vec[2] = c;
    }

    IntVector(int a, int b, int c, int d)
    {
        memset(p_vec, 0, sizeof(int)*N);
        p_vec[0] = a;
        p_vec[1] = b;
        p_vec[2] = c;
        p_vec[3] = d;
    }

    IntVector(const IntVector<N> &v)
    {
        for (uint8_t x = 0; x < N; x++)
            p_vec[x] = v.p_vec[x];
    }

    ~IntVector()
    {
    }

    uint8_t n() { return N; }


    IntVector& operator=(const IntVector& v)
    {
        for (uint8_t x = 0; x < N; x++ )
            p_vec[x] = v.p_vec[x];
        return *this;
    }

    int& operator [](uint8_t n)
    {
        return p_vec[n];
    }

    int operator [](uint8_t n) const
    {
        return p_vec[n];
    }

    int& operator ()(uint8_t n)
    {
        return p_vec[n];
    }

    int operator ()(uint8_t n) const
    {
        return p_vec[n];
    }

    IntVector operator+(const IntVector& v) const
    {
        IntVector ret;
        for(uint8_t i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] + v.p_vec[i];
        return ret;
    }

    IntVector operator-(const IntVector& v) const
    {
        IntVector ret;
        for(uint8_t i = 0; i < N; i++)
            ret.p_vec[i] = p_vec[i] - v.p_vec[i];
        return ret;
    }

    int& x() { return p_vec[0]; }
    int& y() { return p_vec[1]; }
    int& z() { return p_vec[2]; }
    int x() const { return p_vec[0]; }
    int y() const { return p_vec[1]; }
    int z() const { return p_vec[2]; }


private:
    int p_vec[N];
};


//// integer quaternion

class Quaternion
{
public:
    Quaternion(): _w(1), _x(0), _y(0), _z(0) {}

    Quaternion(int w, int x, int y, int z):
        _w(w), _x(x), _y(y), _z(z) {}

    Quaternion(int w, IntVector<3> vec):
        _w(w), _x(vec.x()), _y(vec.y()), _z(vec.z()) {}

    int& w()
    {
        return _w;
    }
    int& x()
    {
        return _x;
    }
    int& y()
    {
        return _y;
    }
    int& z()
    {
        return _z;
    }

    int w() const
    {
        return _w;
    }
    int x() const
    {
        return _x;
    }
    int y() const
    {
        return _y;
    }
    int z() const
    {
        return _z;
    }

//     double magnitude() const
//     {
//         return sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
//     }
// 
//     void normalize()
//     {
//         double mag = magnitude();
//         *this = this->scale(1/mag);
//     }

    Quaternion conjugate() const
    {
        return Quaternion(_w, -_x, -_y, -_z);
    }

//     void fromAxisAngle(const IntVector<3>& axis, double theta)
//     {
//         _w = cos(theta/2);
//         //only need to calculate sine of half theta once
//         double sht = sin(theta/2);
//         _x = axis.x() * sht;
//         _y = axis.y() * sht;
//         _z = axis.z() * sht;
//     }

//     void fromMatrix(const Matrix<3>& m)
//     {
//         double tr = m.trace();
// 
//         double S;
//         if (tr > 0)
//         {
//             S = sqrt(tr+1.0) * 2;
//             _w = 0.25 * S;
//             _x = (m(2, 1) - m(1, 2)) / S;
//             _y = (m(0, 2) - m(2, 0)) / S;
//             _z = (m(1, 0) - m(0, 1)) / S;
//         }
//         else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2))
//         {
//             S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
//             _w = (m(2, 1) - m(1, 2)) / S;
//             _x = 0.25 * S;
//             _y = (m(0, 1) + m(1, 0)) / S;
//             _z = (m(0, 2) + m(2, 0)) / S;
//         }
//         else if (m(1, 1) > m(2, 2))
//         {
//             S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
//             _w = (m(0, 2) - m(2, 0)) / S;
//             _x = (m(0, 1) + m(1, 0)) / S;
//             _y = 0.25 * S;
//             _z = (m(1, 2) + m(2, 1)) / S;
//         }
//         else
//         {
//             S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
//             _w = (m(1, 0) - m(0, 1)) / S;
//             _x = (m(0, 2) + m(2, 0)) / S;
//             _y = (m(1, 2) + m(2, 1)) / S;
//             _z = 0.25 * S;
//         }
//     }
// 
//     void toAxisAngle(Vector<3>& axis, double& angle) const
//     {
//         double sqw = sqrt(1-_w*_w);
//         if (sqw == 0) //it's a singularity and divide by zero, avoid
//             return;
// 
//         angle = 2 * acos(_w);
//         axis.x() = _x / sqw;
//         axis.y() = _y / sqw;
//         axis.z() = _z / sqw;
//     }
// 
//     Matrix<3> toMatrix() const
//     {
//         Matrix<3> ret;
//         ret.cell(0, 0) = 1 - 2*_y*_y - 2*_z*_z;
//         ret.cell(0, 1) = 2*_x*_y - 2*_w*_z;
//         ret.cell(0, 2) = 2*_x*_z + 2*_w*_y;
// 
//         ret.cell(1, 0) = 2*_x*_y + 2*_w*_z;
//         ret.cell(1, 1) = 1 - 2*_x*_x - 2*_z*_z;
//         ret.cell(1, 2) = 2*_y*_z - 2*_w*_x;
// 
//         ret.cell(2, 0) = 2*_x*_z - 2*_w*_y;
//         ret.cell(2, 1) = 2*_y*_z + 2*_w*_x;
//         ret.cell(2, 2) = 1 - 2*_x*_x - 2*_y*_y;
//         return ret;
//     }


    // Returns euler angles that represent the quaternion.  Angles are
    // returned in rotation order and right-handed about the specified
    // axes:
    //
    //   v[0] is applied 1st about z (ie, roll)
    //   v[1] is applied 2nd about y (ie, pitch)
    //   v[2] is applied 3rd about x (ie, yaw)
    //
    // Note that this means result.x() is not a rotation about x;
    // similarly for result.z().
    //
//     Vector<3> toEuler() const
//     {
//         Vector<3> ret;
//         double sqw = _w*_w;
//         double sqx = _x*_x;
//         double sqy = _y*_y;
//         double sqz = _z*_z;
// 
//         ret.x() = atan2(2.0*(_x*_y+_z*_w),(sqx-sqy-sqz+sqw));
//         ret.y() = asin(-2.0*(_x*_z-_y*_w)/(sqx+sqy+sqz+sqw));
//         ret.z() = atan2(2.0*(_y*_z+_x*_w),(-sqx-sqy+sqz+sqw));
// 
//         return ret;
//     }
// 
//     Vector<3> toAngularVelocity(double dt) const
//     {
//         Vector<3> ret;
//         Quaternion one(1.0, 0.0, 0.0, 0.0);
//         Quaternion delta = one - *this;
//         Quaternion r = (delta/dt);
//         r = r * 2;
//         r = r * one;
// 
//         ret.x() = r.x();
//         ret.y() = r.y();
//         ret.z() = r.z();
//         return ret;
//     }
// 
//     Vector<3> rotateVector(const Vector<2>& v) const
//     {
//         return rotateVector(Vector<3>(v.x(), v.y()));
//     }
// 
//     Vector<3> rotateVector(const Vector<3>& v) const
//     {
//         Vector<3> qv(_x, _y, _z);
//         Vector<3> t = qv.cross(v) * 2.0;
//         return v + t*_w + qv.cross(t);
//     }
// 

    Quaternion operator*(const Quaternion& q) const
    {
        return Quaternion(
            _w*q._w - _x*q._x - _y*q._y - _z*q._z,
            _w*q._x + _x*q._w + _y*q._z - _z*q._y,
            _w*q._y - _x*q._z + _y*q._w + _z*q._x,
            _w*q._z + _x*q._y - _y*q._x + _z*q._w
        );
    }

    Quaternion operator+(const Quaternion& q) const
    {
        return Quaternion(_w + q._w, _x + q._x, _y + q._y, _z + q._z);
    }

    Quaternion operator-(const Quaternion& q) const
    {
        return Quaternion(_w - q._w, _x - q._x, _y - q._y, _z - q._z);
    }

//     Quaternion operator/(double scalar) const
//     {
//         return Quaternion(_w / scalar, _x / scalar, _y / scalar, _z / scalar);
//     }

    Quaternion operator*(int scalar) const
    {
        return scale(scalar);
    }

    Quaternion scale(int scalar) const
    {
        return Quaternion(_w * scalar, _x * scalar, _y * scalar, _z * scalar);
    }

private:
    int _w, _x, _y, _z;
};


//////////// BNO sensor


typedef struct
{
    uint16_t accel_offset_x;
    uint16_t accel_offset_y;
    uint16_t accel_offset_z;
    uint16_t gyro_offset_x;
    uint16_t gyro_offset_y;
    uint16_t gyro_offset_z;
    uint16_t mag_offset_x;
    uint16_t mag_offset_y;
    uint16_t mag_offset_z;

    uint16_t accel_radius;
    uint16_t mag_radius;
} bno055_offsets_t;

class BNO055
{
  public:
    typedef enum
    {
      /* Page id register definition */
      BNO055_PAGE_ID_ADDR                                     = 0X07,

      /* PAGE0 REGISTER DEFINITION START*/
      BNO055_CHIP_ID_ADDR                                     = 0x00,
      BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
      BNO055_MAG_REV_ID_ADDR                                  = 0x02,
      BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
      BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
      BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
      BNO055_BL_REV_ID_ADDR                                   = 0X06,

      /* Accel data register */
      BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
      BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
      BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
      BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
      BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
      BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,

      /* Mag data register */
      BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
      BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
      BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
      BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
      BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
      BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,

      /* Gyro data registers */
      BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
      BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
      BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
      BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
      BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
      BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,

      /* Euler data registers */
      BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
      BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
      BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
      BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
      BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
      BNO055_EULER_P_MSB_ADDR                                 = 0X1F,

      /* Quaternion data registers */
      BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
      BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
      BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
      BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
      BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
      BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
      BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
      BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,

      /* Linear acceleration data registers */
      BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
      BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
      BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
      BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
      BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
      BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,

      /* Gravity data registers */
      BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
      BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
      BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
      BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
      BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
      BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,

      /* Temperature data register */
      BNO055_TEMP_ADDR                                        = 0X34,

      /* Status registers */
      BNO055_CALIB_STAT_ADDR                                  = 0X35,
      BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
      BNO055_INTR_STAT_ADDR                                   = 0X37,

      BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
      BNO055_SYS_STAT_ADDR                                    = 0X39,
      BNO055_SYS_ERR_ADDR                                     = 0X3A,

      /* Unit selection register */
      BNO055_UNIT_SEL_ADDR                                    = 0X3B,
      BNO055_DATA_SELECT_ADDR                                 = 0X3C,

      /* Mode registers */
      BNO055_OPR_MODE_ADDR                                    = 0X3D,
      BNO055_PWR_MODE_ADDR                                    = 0X3E,

      BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
      BNO055_TEMP_SOURCE_ADDR                                 = 0X40,

      /* Axis remap registers */
      BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
      BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,

      /* SIC registers */
      BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
      BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
      BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
      BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
      BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
      BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
      BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
      BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
      BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
      BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
      BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
      BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
      BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
      BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
      BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
      BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
      BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
      BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54,

      /* Accelerometer Offset registers */
      ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
      ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
      ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
      ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
      ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
      ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,

      /* Magnetometer Offset registers */
      MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
      MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
      MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
      MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
      MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
      MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,

      /* Gyroscope Offset register s*/
      GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
      GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
      GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
      GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
      GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
      GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,

      /* Radius registers */
      ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
      ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
      MAG_RADIUS_LSB_ADDR                                     = 0X69,
      MAG_RADIUS_MSB_ADDR                                     = 0X6A
    } bno055_reg_t;

    typedef enum
    {
      POWER_MODE_NORMAL                                       = 0X00,
      POWER_MODE_LOWPOWER                                     = 0X01,
      POWER_MODE_SUSPEND                                      = 0X02
    } bno055_powermode_t;

    typedef enum
    {
      /* Operation mode settings*/
      OPERATION_MODE_CONFIG                                   = 0X00,
      OPERATION_MODE_ACCONLY                                  = 0X01,
      OPERATION_MODE_MAGONLY                                  = 0X02,
      OPERATION_MODE_GYRONLY                                  = 0X03,
      OPERATION_MODE_ACCMAG                                   = 0X04,
      OPERATION_MODE_ACCGYRO                                  = 0X05,
      OPERATION_MODE_MAGGYRO                                  = 0X06,
      OPERATION_MODE_AMG                                      = 0X07,
      OPERATION_MODE_IMUPLUS                                  = 0X08,
      OPERATION_MODE_COMPASS                                  = 0X09,
      OPERATION_MODE_M4G                                      = 0X0A,
      OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
      OPERATION_MODE_NDOF                                     = 0X0C
    } bno055_opmode_t;

    typedef enum
    {
      REMAP_CONFIG_P0                                         = 0x21,
      REMAP_CONFIG_P1                                         = 0x24, // default
      REMAP_CONFIG_P2                                         = 0x24,
      REMAP_CONFIG_P3                                         = 0x21,
      REMAP_CONFIG_P4                                         = 0x24,
      REMAP_CONFIG_P5                                         = 0x21,
      REMAP_CONFIG_P6                                         = 0x21,
      REMAP_CONFIG_P7                                         = 0x24
    } bno055_axis_remap_config_t;

    typedef enum
    {
      REMAP_SIGN_P0                                           = 0x04,
      REMAP_SIGN_P1                                           = 0x00, // default
      REMAP_SIGN_P2                                           = 0x06,
      REMAP_SIGN_P3                                           = 0x02,
      REMAP_SIGN_P4                                           = 0x03,
      REMAP_SIGN_P5                                           = 0x01,
      REMAP_SIGN_P6                                           = 0x07,
      REMAP_SIGN_P7                                           = 0x05
    } bno055_axis_remap_sign_t;

    typedef struct
    {
      uint8_t  accel_rev;
      uint8_t  mag_rev;
      uint8_t  gyro_rev;
      uint16_t sw_rev;
      uint8_t  bl_rev;
    } bno055_rev_info_t;

    typedef enum
    {
      VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
      VECTOR_MAGNETOMETER  = BNO055_MAG_DATA_X_LSB_ADDR,
      VECTOR_GYROSCOPE     = BNO055_GYRO_DATA_X_LSB_ADDR,
      VECTOR_EULER         = BNO055_EULER_H_LSB_ADDR,
      VECTOR_LINEARACCEL   = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
      VECTOR_GRAVITY       = BNO055_GRAVITY_DATA_X_LSB_ADDR
    } bno_vector_type_t;

#ifdef ARDUINO_SAMD_ZERO
#error "On an arduino Zero, BNO055's ADR pin must be high. Fix that, then delete this line."
    BNO055 ( int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_B );
#else
    BNO055 ( int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A );
#endif
    bool  begin               ( bno055_opmode_t mode = OPERATION_MODE_NDOF );
    void  setMode             ( bno055_opmode_t mode );
    void  getRevInfo          ( bno055_rev_info_t* );
    void  displayRevInfo      ( void );
    void  setExtCrystalUse    ( boolean usextal );
    void  getSystemStatus     ( uint8_t *system_status,
                                uint8_t *self_test_result,
                                uint8_t *system_error);
//     void  displaySystemStatus ( void );
    void  getCalibration      ( uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
    uint8_t getCalibrationRaw();


    IntVector<3>  getVector ( bno_vector_type_t vector_type );
    Quaternion getQuat   ( void );
    int8_t        getTemp   ( void );

    /* Adafruit_Sensor implementation */
//     bool  getEvent  ( sensors_event_t* );
//     void  getSensor ( sensor_t* );

    /* Functions to deal with raw calibration data */
    bool  getSensorOffsets(uint8_t* calibData);
//     bool  getSensorOffsets(bno055_offsets_t &offsets_type);
    void  setSensorOffsets(const uint8_t* calibData);
//     void  setSensorOffsets(const offsets_t &offsets_type);
    bool  isFullyCalibrated(void);

  private:
    byte  read8   ( bno055_reg_t );
    bool  readLen ( bno055_reg_t, byte* buffer, uint8_t len );
    bool  write8  ( bno055_reg_t, byte value );

    uint8_t _address;
    int32_t _sensorID;
    bno055_opmode_t _mode;
};

#endif
