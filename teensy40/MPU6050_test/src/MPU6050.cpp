#include "MPU6050.h"

// ---------- MPU6050 registers ----------
#define MPU_ADDR        0x68
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_FIFO_EN     0x23
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE  0x38
#define REG_FIFO_COUNT  0x72
#define REG_FIFO_RW     0x74
#define REG_USER_CTRL   0x6A
#define REG_PWR_MGMT_1  0x6B

// ---------- Config constants ----------
#define DLPF_CFG          3       // ~44 Hz accel/gyro bandwidth
#define ACCEL_FS          0x08    // ±4 g
#define GYRO_FS           0x10    // ±1000 dps
#define ACC_LSB_PER_G     8192.0f
#define GYRO_LSB_PER_DPS  32.8f
#define FIFO_FRAME_BYTES  12

// ---------- Mahony state ----------
typedef struct {
    float twoKp, twoKi;
    float q0, q1, q2, q3;
    float ix, iy, iz;
} Mahony;

static Mahony mahony;

// ---------- Mahony functions ----------
static void Mahony_init(Mahony *m, float Kp, float Ki) {
    m->twoKp = 2.0f * Kp;
    m->twoKi = 2.0f * Ki;
    m->q0 = 1.0f; m->q1 = m->q2 = m->q3 = 0.0f;
    m->ix = m->iy = m->iz = 0.0f;
}

static void Mahony_update(Mahony *m, float gx, float gy, float gz,
                          float ax, float ay, float az, float dt) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    float vx = 2.0f*(m->q1*m->q3 - m->q0*m->q2);
    float vy = 2.0f*(m->q0*m->q1 + m->q2*m->q3);
    float vz = m->q0*m->q0 - m->q1*m->q1 - m->q2*m->q2 + m->q3*m->q3;

    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    if (m->twoKi > 0.0f) {
        m->ix += m->twoKi*ex*dt;
        m->iy += m->twoKi*ey*dt;
        m->iz += m->twoKi*ez*dt;
    } else {
        m->ix = m->iy = m->iz = 0.0f;
    }

    gx += m->twoKp*ex + m->ix;
    gy += m->twoKp*ey + m->iy;
    gz += m->twoKp*ez + m->iz;

    gx *= 0.5f*dt; gy *= 0.5f*dt; gz *= 0.5f*dt;
    float qa = m->q0, qb = m->q1, qc = m->q2;
    m->q0 += (-qb*gx - qc*gy - m->q3*gz);
    m->q1 += (qa*gx + qc*gz - m->q3*gy);
    m->q2 += (qa*gy - qb*gz + m->q3*gx);
    m->q3 += (qa*gz + qb*gy - qc*gx);

    norm = 1.0f / sqrtf(m->q0*m->q0 + m->q1*m->q1 +
                        m->q2*m->q2 + m->q3*m->q3);
    m->q0*=norm; m->q1*=norm; m->q2*=norm; m->q3*=norm;
}

static void Mahony_getEulerRP(const Mahony *m, float *roll_deg, float *pitch_deg) {
    float roll  = atan2f(2*(m->q0*m->q1 + m->q2*m->q3),
                         1-2*(m->q1*m->q1 + m->q2*m->q2));
    float pitch = asinf(2*(m->q0*m->q2 - m->q3*m->q1));
    *roll_deg  = roll  * 180.0f / PI;
    *pitch_deg = pitch * 180.0f / PI;
}

// ---------- I2C helpers ----------
static void i2cWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static void i2cBurstRead(uint8_t reg, uint8_t* buf, uint16_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, (int)len);
    for (uint16_t i=0;i<len && Wire.available();++i) buf[i] = Wire.read();
}

static uint16_t readU16BE(uint8_t high, uint8_t low) {
    return (uint16_t)high<<8 | low;
}

// ---------- Public API ----------
bool mpuInit(void) {
    Mahony_init(&mahony, 0.5f, 0.0f);

    i2cWrite(REG_PWR_MGMT_1, 0x80); delay(100); // reset
    i2cWrite(REG_PWR_MGMT_1, 0x01); delay(10);  // clock = PLL

    i2cWrite(REG_CONFIG, DLPF_CFG);
    i2cWrite(REG_GYRO_CFG,  GYRO_FS);
    i2cWrite(REG_ACCEL_CFG, ACCEL_FS);
    i2cWrite(REG_SMPLRT_DIV, 0x00);   // 1 kHz

    i2cWrite(REG_USER_CTRL, 0x04); delay(10); // FIFO reset
    i2cWrite(REG_USER_CTRL, 0x40);            // FIFO enable
    i2cWrite(REG_FIFO_EN, 0x78);              // accel + gyro

    i2cWrite(REG_INT_PIN_CFG, 0x10);
    i2cWrite(REG_INT_ENABLE,  0x01);

    return true;
}

void mpuUpdate(float dt, float *roll, float *pitch) {
    uint8_t cntBuf[2];
    i2cBurstRead(REG_FIFO_COUNT, cntBuf, 2);
    uint16_t fifo_count = (cntBuf[0]<<8)|cntBuf[1];

    if (fifo_count < FIFO_FRAME_BYTES) return;

    uint8_t raw[12];
    i2cBurstRead(REG_FIFO_RW, raw, sizeof(raw));

    int16_t ax_i = (int16_t)readU16BE(raw[0], raw[1]);
    int16_t ay_i = (int16_t)readU16BE(raw[2], raw[3]);
    int16_t az_i = (int16_t)readU16BE(raw[4], raw[5]);
    int16_t gx_i = (int16_t)readU16BE(raw[6], raw[7]);
    int16_t gy_i = (int16_t)readU16BE(raw[8], raw[9]);
    int16_t gz_i = (int16_t)readU16BE(raw[10], raw[11]);

    float ax = (float)ax_i / ACC_LSB_PER_G;
    float ay = (float)ay_i / ACC_LSB_PER_G;
    float az = (float)az_i / ACC_LSB_PER_G;
    float gx_dps = (float)gx_i / GYRO_LSB_PER_DPS;
    float gy_dps = (float)gy_i / GYRO_LSB_PER_DPS;
    float gz_dps = (float)gz_i / GYRO_LSB_PER_DPS;

    const float DEG2RAD = PI / 180.0f;
    Mahony_update(&mahony,
                  gx_dps*DEG2RAD, gy_dps*DEG2RAD, gz_dps*DEG2RAD,
                  ax, ay, az, dt);

    Mahony_getEulerRP(&mahony, roll, pitch);
}
