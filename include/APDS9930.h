/**
 * @file    APDS-9930.h
 * @brief   Library for the SparkFun APDS-9930 breakout board
 * @author  Shawn Hymel (SparkFun Electronics)
 *
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the Avago APDS-9930 to Arduino over I2C. The library
 * relies on the Arduino Wire (I2C) library. to use the library, instantiate an
 * APDS9930 object, call init(), and call the appropriate functions.
 */

#ifndef APDS9930_H
#define APDS9930_H

#include <stdint.h>
#include <air001xx_hal_i2c.h>

/* On/Off definitions */
#define OFF 0
#define ON 1

/* Acceptable parameters for setMode */
#define POWER 0
#define AMBIENT_LIGHT 1
#define PROXIMITY 2
#define WAIT 3
#define AMBIENT_LIGHT_INT 4
#define PROXIMITY_INT 5
#define SLEEP_AFTER_INT 6
#define ALL 7

/* LED Drive values */
#define LED_DRIVE_100MA 0
#define LED_DRIVE_50MA 1
#define LED_DRIVE_25MA 2
#define LED_DRIVE_12_5MA 3

/* Proximity Gain (PGAIN) values */
#define PGAIN_1X 0
#define PGAIN_2X 1
#define PGAIN_4X 2
#define PGAIN_8X 3

/* ALS Gain (AGAIN) values */
#define AGAIN_1X 0
#define AGAIN_8X 1
#define AGAIN_16X 2
#define AGAIN_120X 3

/* State definitions */
enum
{
    NOTAVAILABLE_STATE,
    NEAR_STATE,
    FAR_STATE,
    ALL_STATE
};

#ifdef _AVR_IO_H_
// Do not use this alias as it's deprecated
#define NA_STATE NOTAVAILABLE_STATE
#endif

/* APDS9930 Class */
class APDS9930
{
public:
    /* Initialization methods */
    APDS9930();
    ~APDS9930();
    bool init(GPIO_TypeDef *port, uint32_t sdaPin, uint32_t sclPin, uint32_t afNum);
    uint8_t getMode();
    bool setMode(uint8_t mode, uint8_t enable);

    /* Turn the APDS-9930 on and off */
    bool enablePower();
    bool disablePower();

    /* Enable or disable specific sensors */
    bool enableLightSensor(bool interrupts = false);
    bool disableLightSensor();
    bool enableProximitySensor(bool interrupts = false);
    bool disableProximitySensor();

    /* LED drive strength control */
    uint8_t getLEDDrive();
    bool setLEDDrive(uint8_t drive);
    // uint8_t getGestureLEDDrive();
    // bool setGestureLEDDrive(uint8_t drive);

    /* Gain control */
    uint8_t getAmbientLightGain();
    bool setAmbientLightGain(uint8_t gain);
    uint8_t getProximityGain();
    bool setProximityGain(uint8_t gain);
    bool setProximityDiode(uint8_t drive);
    uint8_t getProximityDiode();

    /* Get and set light interrupt thresholds */
    bool getLightIntLowThreshold(uint16_t &threshold);
    bool setLightIntLowThreshold(uint16_t threshold);
    bool getLightIntHighThreshold(uint16_t &threshold);
    bool setLightIntHighThreshold(uint16_t threshold);

    /* Proximity Interrupt Threshold */
    uint16_t getProximityIntLowThreshold();
    bool setProximityIntLowThreshold(uint16_t threshold);
    uint16_t getProximityIntHighThreshold();
    bool setProximityIntHighThreshold(uint16_t threshold);

    /* Get and set interrupt enables */
    uint8_t getAmbientLightIntEnable();
    bool setAmbientLightIntEnable(uint8_t enable);
    uint8_t getProximityIntEnable();
    bool setProximityIntEnable(uint8_t enable);

    /* Clear interrupts */
    bool clearAmbientLightInt();
    bool clearProximityInt();
    bool clearAllInts();

    /* Proximity methods */
    bool readProximity(uint16_t &val);

    /* Ambient light methods */
    bool readAmbientLightLux(float &val);
    bool readAmbientLightLux(unsigned long &val);
    float floatAmbientToLux(uint16_t Ch0, uint16_t Ch1);
    unsigned long ulongAmbientToLux(uint16_t Ch0, uint16_t Ch1);
    bool readCh0Light(uint16_t &val);
    bool readCh1Light(uint16_t &val);

private:
    I2C_HandleTypeDef i2cHandle;

    /* Raw I2C Commands */
    bool wireWriteByte(uint8_t val);
    bool wireWriteDataByte(uint8_t reg, uint8_t val);
    bool wireWriteDataBlock(uint8_t reg, uint8_t *val, unsigned int len);
    bool wireReadDataByte(uint8_t reg, uint8_t &val);
    int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len);
};

#endif
