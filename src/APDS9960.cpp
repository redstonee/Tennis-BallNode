/**
 * @file    SparkFun_APDS-9960.cpp
 * @brief   Library for the SparkFun APDS-9960 breakout board
 * @author  Shawn Hymel (SparkFun Electronics)
 *
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the Avago APDS-9960 to Arduino over I2C. The library
 * relies on the Arduino Wire (I2C) library. to use the library, instantiate an
 * APDS9960 object, call init(), and call the appropriate functions.
 *
 * APDS-9960 current draw tests (default parameters):
 *   Off:                   1mA
 *   Waiting for gesture:   14mA
 *   Gesture in progress:   35mA
 */

#include "APDS9960.h"
#include <cmath>
#include <air001xx_hal.h>

/* APDS-9930 I2C address */
#define APDS9930_I2C_ADDR       0x39

/* Command register modes */
#define REPEATED_BYTE           0x80
#define AUTO_INCREMENT          0xA0
#define SPECIAL_FN              0xE0

/* Error code for returned values */
#define ERROR                   0xFF

/* Acceptable device IDs */
#define APDS9930_ID_1           0x12
#define APDS9930_ID_2           0x39

/* Misc parameters */
#define FIFO_PAUSE_TIME         30      // Wait period (ms) between FIFO reads

/* APDS-9930 register addresses */
#define APDS9930_ENABLE         0x00
#define APDS9930_ATIME          0x01
#define APDS9930_PTIME          0x02
#define APDS9930_WTIME          0x03
#define APDS9930_AILTL          0x04
#define APDS9930_AILTH          0x05
#define APDS9930_AIHTL          0x06
#define APDS9930_AIHTH          0x07
#define APDS9930_PILTL          0x08
#define APDS9930_PILTH          0x09
#define APDS9930_PIHTL          0x0A
#define APDS9930_PIHTH          0x0B
#define APDS9930_PERS           0x0C
#define APDS9930_CONFIG         0x0D
#define APDS9930_PPULSE         0x0E
#define APDS9930_CONTROL        0x0F
#define APDS9930_ID             0x12
#define APDS9930_STATUS         0x13
#define APDS9930_Ch0DATAL       0x14
#define APDS9930_Ch0DATAH       0x15
#define APDS9930_Ch1DATAL       0x16
#define APDS9930_Ch1DATAH       0x17
#define APDS9930_PDATAL         0x18
#define APDS9930_PDATAH         0x19
#define APDS9930_POFFSET        0x1E


/* Bit fields */
#define APDS9930_PON            0b00000001
#define APDS9930_AEN            0b00000010
#define APDS9930_PEN            0b00000100
#define APDS9930_WEN            0b00001000
#define APSD9930_AIEN           0b00010000
#define APDS9930_PIEN           0b00100000
#define APDS9930_SAI            0b01000000

/* On/Off definitions */
#define OFF                     0
#define ON                      1

/* Acceptable parameters for setMode */
#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define SLEEP_AFTER_INT         6
#define ALL                     7

/* LED Drive values */
#define LED_DRIVE_100MA         0
#define LED_DRIVE_50MA          1
#define LED_DRIVE_25MA          2
#define LED_DRIVE_12_5MA        3

/* Proximity Gain (PGAIN) values */
#define PGAIN_1X                0
#define PGAIN_2X                1
#define PGAIN_4X                2
#define PGAIN_8X                3

/* ALS Gain (AGAIN) values */
#define AGAIN_1X                0
#define AGAIN_8X                1
#define AGAIN_16X               2
#define AGAIN_120X              3

/* Interrupt clear values */
#define CLEAR_PROX_INT          0xE5
#define CLEAR_ALS_INT           0xE6
#define CLEAR_ALL_INTS          0xE7

/* Default values */
#define DEFAULT_ATIME           0xED
#define DEFAULT_WTIME           0xFF
#define DEFAULT_PTIME           0xFF
#define DEFAULT_PPULSE          0x08
#define DEFAULT_POFFSET         0       // 0 offset
#define DEFAULT_CONFIG          0
#define DEFAULT_PDRIVE          LED_DRIVE_100MA
#define DEFAULT_PDIODE          2
#define DEFAULT_PGAIN           PGAIN_8X
#define DEFAULT_AGAIN           AGAIN_1X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x22    // 2 consecutive prox or ALS for int.

/* ALS coefficients */
#define DF                      52
#define GA                      0.49
#define ALS_B                       1.862
#define ALS_C                       0.746
#define ALS_D                       1.291

/* APDS-9960 I2C address */
#define APDS9960_I2C_ADDR (0x39 << 1)

/* Gesture parameters */
#define GESTURE_THRESHOLD_OUT 10
#define GESTURE_SENSITIVITY_1 50
#define GESTURE_SENSITIVITY_2 20

/* Error code for returned values */
#define ERROR 0xFF

/* Acceptable device IDs */
#define APDS9960_ID_1 0xAB
#define APDS9960_ID_2 0x9C
#define APDS9960_ID_3 0xA8

/* Misc parameters */
#define FIFO_PAUSE_TIME 30 // Wait period (ms) between FIFO reads

/* APDS-9960 register addresses */
#define APDS9960_ENABLE 0x80
#define APDS9960_ATIME 0x81
#define APDS9960_WTIME 0x83
#define APDS9960_AILTL 0x84
#define APDS9960_AILTH 0x85
#define APDS9960_AIHTL 0x86
#define APDS9960_AIHTH 0x87
#define APDS9960_PILT 0x89
#define APDS9960_PIHT 0x8B
#define APDS9960_PERS 0x8C
#define APDS9960_CONFIG1 0x8D
#define APDS9960_PPULSE 0x8E
#define APDS9960_CONTROL 0x8F
#define APDS9960_CONFIG2 0x90
#define APDS9960_ID 0x92
#define APDS9960_STATUS 0x93
#define APDS9960_CDATAL 0x94
#define APDS9960_CDATAH 0x95
#define APDS9960_RDATAL 0x96
#define APDS9960_RDATAH 0x97
#define APDS9960_GDATAL 0x98
#define APDS9960_GDATAH 0x99
#define APDS9960_BDATAL 0x9A
#define APDS9960_BDATAH 0x9B
#define APDS9960_PDATA 0x9C
#define APDS9960_POFFSET_UR 0x9D
#define APDS9960_POFFSET_DL 0x9E
#define APDS9960_CONFIG3 0x9F
#define APDS9960_GPENTH 0xA0
#define APDS9960_GEXTH 0xA1
#define APDS9960_GCONF1 0xA2
#define APDS9960_GCONF2 0xA3
#define APDS9960_GOFFSET_U 0xA4
#define APDS9960_GOFFSET_D 0xA5
#define APDS9960_GOFFSET_L 0xA7
#define APDS9960_GOFFSET_R 0xA9
#define APDS9960_GPULSE 0xA6
#define APDS9960_GCONF3 0xAA
#define APDS9960_GCONF4 0xAB
#define APDS9960_GFLVL 0xAE
#define APDS9960_GSTATUS 0xAF
#define APDS9960_IFORCE 0xE4
#define APDS9960_PICLEAR 0xE5
#define APDS9960_CICLEAR 0xE6
#define APDS9960_AICLEAR 0xE7
#define APDS9960_GFIFO_U 0xFC
#define APDS9960_GFIFO_D 0xFD
#define APDS9960_GFIFO_L 0xFE
#define APDS9960_GFIFO_R 0xFF

/* Bit fields */
#define APDS9960_PON 0b00000001
#define APDS9960_AEN 0b00000010
#define APDS9960_PEN 0b00000100
#define APDS9960_WEN 0b00001000
#define APSD9960_AIEN 0b00010000
#define APDS9960_PIEN 0b00100000
#define APDS9960_GEN 0b01000000
#define APDS9960_GVALID 0b00000001

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
#define GESTURE 6
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
#define AGAIN_4X 1
#define AGAIN_16X 2
#define AGAIN_64X 3

/* Gesture Gain (GGAIN) values */
#define GGAIN_1X 0
#define GGAIN_2X 1
#define GGAIN_4X 2
#define GGAIN_8X 3

/* LED Boost values */
#define LED_BOOST_100 0
#define LED_BOOST_150 1
#define LED_BOOST_200 2
#define LED_BOOST_300 3

/* Gesture wait time values */
#define GWTIME_0MS 0
#define GWTIME_2_8MS 1
#define GWTIME_5_6MS 2
#define GWTIME_8_4MS 3
#define GWTIME_14_0MS 4
#define GWTIME_22_4MS 5
#define GWTIME_30_8MS 6
#define GWTIME_39_2MS 7

/* Default values */
#define DEFAULT_ATIME 219           // 103ms
#define DEFAULT_WTIME 246           // 27ms
#define DEFAULT_PROX_PPULSE 0x87    // 16us, 8 pulses
#define DEFAULT_GESTURE_PPULSE 0x89 // 16us, 10 pulses
#define DEFAULT_POFFSET_UR 0        // 0 offset
#define DEFAULT_POFFSET_DL 0        // 0 offset
#define DEFAULT_CONFIG1 0x60        // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE LED_DRIVE_100MA
#define DEFAULT_PGAIN PGAIN_4X
#define DEFAULT_AGAIN AGAIN_4X
#define DEFAULT_PILT 0      // Low proximity threshold
#define DEFAULT_PIHT 50     // High proximity threshold
#define DEFAULT_AILT 0xFFFF // Force interrupt for calibration
#define DEFAULT_AIHT 0
#define DEFAULT_PERS 0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2 0x01 // No saturation interrupts or LED boost
#define DEFAULT_CONFIG3 0    // Enable all photodiodes, no SAI
#define DEFAULT_GPENTH 40    // Threshold for entering gesture mode
#define DEFAULT_GEXTH 30     // Threshold for exiting gesture mode
#define DEFAULT_GCONF1 0x40  // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN GGAIN_4X
#define DEFAULT_GLDRIVE LED_DRIVE_100MA
#define DEFAULT_GWTIME GWTIME_2_8MS
#define DEFAULT_GOFFSET 0   // No offset scaling for gesture mode
#define DEFAULT_GPULSE 0xC9 // 32us, 10 pulses
#define DEFAULT_GCONF3 0    // All photodiodes active during gesture
#define DEFAULT_GIEN 0      // Disable gesture interrupts

/**
 * @brief Constructor - Instantiates SparkFun_APDS9960 object
 */
SparkFun_APDS9960::SparkFun_APDS9960()
{

    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;

    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;

    gesture_near_count_ = 0;
    gesture_far_count_ = 0;

    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}

/**
 * @brief Destructor
 */
SparkFun_APDS9960::~SparkFun_APDS9960()
{
}

/**
 * @brief Configures I2C communications and initializes registers to defaults
 *
 * @return True if initialized successfully. False otherwise.
 */
bool SparkFun_APDS9960::init(GPIO_TypeDef *port, uint32_t sdaPin, uint32_t sclPin, uint32_t afNum)
{
    /* Initialize I2C */
    __HAL_RCC_I2C_CLK_ENABLE();
    __HAL_RCC_I2C_FORCE_RESET();
    __HAL_RCC_I2C_RELEASE_RESET();

    GPIO_InitTypeDef GPIO_InitStruct{
        .Pin = sdaPin | sclPin,
        .Mode = GPIO_MODE_AF_OD, // Open-drain mode for I2C
        .Pull = GPIO_NOPULL,     // No pull-up or pull-down resistors
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = afNum, // Alternate function number
    };
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    i2cHandle = {
        .Instance = I2C1, // Use I2C1
        .Init = {
            .ClockSpeed = 100000, // 100kHz
            .DutyCycle = I2C_DUTYCYCLE_16_9,
            .OwnAddress1 = 0x00, // No own address
            .GeneralCallMode = I2C_GENERALCALL_DISABLE,
            .NoStretchMode = I2C_NOSTRETCH_DISABLE,
        }};
    if (HAL_I2C_Init(&i2cHandle) != HAL_OK)
    {
        return false; // Initialization failed
    }

    uint8_t id;

    /* Read ID register and check against known values for APDS-9960 */
    if (!wireReadDataByte(APDS9960_ID, id))
    {
        return false;
    }
    if (!(id == APDS9960_ID_1 || id == APDS9960_ID_2 || id == APDS9960_ID_3))
    {
        return false;
    }

    /* Set ENABLE register to 0 (disable all features) */
    if (!setMode(ALL, OFF))
    {
        return false;
    }

    /* Set default values for ambient light and proximity registers */
    if (!wireWriteDataByte(APDS9960_ATIME, DEFAULT_ATIME))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_WTIME, DEFAULT_WTIME))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_CONFIG1, DEFAULT_CONFIG1))
    {
        return false;
    }
    if (!setLEDDrive(DEFAULT_LDRIVE))
    {
        return false;
    }
    if (!setProximityGain(DEFAULT_PGAIN))
    {
        return false;
    }
    if (!setAmbientLightGain(DEFAULT_AGAIN))
    {
        return false;
    }
    if (!setProxIntLowThresh(DEFAULT_PILT))
    {
        return false;
    }
    if (!setProxIntHighThresh(DEFAULT_PIHT))
    {
        return false;
    }
    if (!setLightIntLowThreshold(DEFAULT_AILT))
    {
        return false;
    }
    if (!setLightIntHighThreshold(DEFAULT_AIHT))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_PERS, DEFAULT_PERS))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_CONFIG2, DEFAULT_CONFIG2))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_CONFIG3, DEFAULT_CONFIG3))
    {
        return false;
    }

    /* Set default values for gesture sense registers */
    if (!setGestureEnterThresh(DEFAULT_GPENTH))
    {
        return false;
    }
    if (!setGestureExitThresh(DEFAULT_GEXTH))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_GCONF1, DEFAULT_GCONF1))
    {
        return false;
    }
    if (!setGestureGain(DEFAULT_GGAIN))
    {
        return false;
    }
    if (!setGestureLEDDrive(DEFAULT_GLDRIVE))
    {
        return false;
    }
    if (!setGestureWaitTime(DEFAULT_GWTIME))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_GPULSE, DEFAULT_GPULSE))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_GCONF3, DEFAULT_GCONF3))
    {
        return false;
    }
    if (!setGestureIntEnable(DEFAULT_GIEN))
    {
        return false;
    }

#if 0
    /* Gesture config register dump */
    uint8_t reg;
    uint8_t val;

    for(reg = 0x80; reg <= 0xAF; reg++) {
        if( (reg != 0x82) && \
            (reg != 0x8A) && \
            (reg != 0x91) && \
            (reg != 0xA8) && \
            (reg != 0xAC) && \
            (reg != 0xAD) )
        {
            wireReadDataByte(reg, val);
            Serial.print(reg, HEX);
            Serial.print(": 0x");
            Serial.println(val, HEX);
        }
    }

    for(reg = 0xE4; reg <= 0xE7; reg++) {
        wireReadDataByte(reg, val);
        Serial.print(reg, HEX);
        Serial.print(": 0x");
        Serial.println(val, HEX);
    }
#endif

    return true;
}

/*******************************************************************************
 * Public methods for controlling the APDS-9960
 ******************************************************************************/

/**
 * @brief Reads and returns the contents of the ENABLE register
 *
 * @return Contents of the ENABLE register. 0xFF if error.
 */
uint8_t SparkFun_APDS9960::getMode()
{
    uint8_t enable_value;

    /* Read current ENABLE register */
    if (!wireReadDataByte(APDS9960_ENABLE, enable_value))
    {
        return ERROR;
    }

    return enable_value;
}

/**
 * @brief Enables or disables a feature in the APDS-9960
 *
 * @param[in] mode which feature to enable
 * @param[in] enable ON (1) or OFF (0)
 * @return True if operation success. False otherwise.
 */
bool SparkFun_APDS9960::setMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = getMode();
    if (reg_val == ERROR)
    {
        return false;
    }

    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if (mode <= 6)
    {
        if (enable)
        {
            reg_val |= (1 << mode);
        }
        else
        {
            reg_val &= ~(1 << mode);
        }
    }
    else if (mode == ALL)
    {
        if (enable)
        {
            reg_val = 0x7F;
        }
        else
        {
            reg_val = 0x00;
        }
    }

    /* Write value back to ENABLE register */
    if (!wireWriteDataByte(APDS9960_ENABLE, reg_val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Starts the light (R/G/B/Ambient) sensor on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware interrupt on high or low light
 * @return True if sensor enabled correctly. False on error.
 */
bool SparkFun_APDS9960::enableLightSensor(bool interrupts)
{

    /* Set default gain, interrupts, enable power, and enable sensor */
    if (!setAmbientLightGain(DEFAULT_AGAIN))
    {
        return false;
    }
    if (interrupts)
    {
        if (!setAmbientLightIntEnable(1))
        {
            return false;
        }
    }
    else
    {
        if (!setAmbientLightIntEnable(0))
        {
            return false;
        }
    }
    if (!enablePower())
    {
        return false;
    }
    if (!setMode(AMBIENT_LIGHT, 1))
    {
        return false;
    }

    return true;
}

/**
 * @brief Ends the light sensor on the APDS-9960
 *
 * @return True if sensor disabled correctly. False on error.
 */
bool SparkFun_APDS9960::disableLightSensor()
{
    if (!setAmbientLightIntEnable(0))
    {
        return false;
    }
    if (!setMode(AMBIENT_LIGHT, 0))
    {
        return false;
    }

    return true;
}

/**
 * @brief Starts the proximity sensor on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware external interrupt on proximity
 * @return True if sensor enabled correctly. False on error.
 */
bool SparkFun_APDS9960::enableProximitySensor(bool interrupts)
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    if (!setProximityGain(DEFAULT_PGAIN))
    {
        return false;
    }
    if (!setLEDDrive(DEFAULT_LDRIVE))
    {
        return false;
    }
    if (interrupts)
    {
        if (!setProximityIntEnable(1))
        {
            return false;
        }
    }
    else
    {
        if (!setProximityIntEnable(0))
        {
            return false;
        }
    }
    if (!enablePower())
    {
        return false;
    }
    if (!setMode(PROXIMITY, 1))
    {
        return false;
    }

    return true;
}

/**
 * @brief Ends the proximity sensor on the APDS-9960
 *
 * @return True if sensor disabled correctly. False on error.
 */
bool SparkFun_APDS9960::disableProximitySensor()
{
    if (!setProximityIntEnable(0))
    {
        return false;
    }
    if (!setMode(PROXIMITY, 0))
    {
        return false;
    }

    return true;
}

/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware external interrupt on gesture
 * @return True if engine enabled correctly. False on error.
 */
bool SparkFun_APDS9960::enableGestureSensor(bool interrupts)
{

    /* Enable gesture mode
       Set ENABLE to 0 (power off)
       Set WTIME to 0xFF
       Set AUX to LED_BOOST_300
       Enable PON, WEN, PEN, GEN in ENABLE
    */
    resetGestureParameters();
    if (!wireWriteDataByte(APDS9960_WTIME, 0xFF))
    {
        return false;
    }
    if (!wireWriteDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE))
    {
        return false;
    }
    if (!setLEDBoost(LED_BOOST_300))
    {
        return false;
    }
    if (interrupts)
    {
        if (!setGestureIntEnable(1))
        {
            return false;
        }
    }
    else
    {
        if (!setGestureIntEnable(0))
        {
            return false;
        }
    }
    if (!setGestureMode(1))
    {
        return false;
    }
    if (!enablePower())
    {
        return false;
    }
    if (!setMode(WAIT, 1))
    {
        return false;
    }
    if (!setMode(PROXIMITY, 1))
    {
        return false;
    }
    if (!setMode(GESTURE, 1))
    {
        return false;
    }

    return true;
}

/**
 * @brief Ends the gesture recognition engine on the APDS-9960
 *
 * @return True if engine disabled correctly. False on error.
 */
bool SparkFun_APDS9960::disableGestureSensor()
{
    resetGestureParameters();
    if (!setGestureIntEnable(0))
    {
        return false;
    }
    if (!setGestureMode(0))
    {
        return false;
    }
    if (!setMode(GESTURE, 0))
    {
        return false;
    }

    return true;
}

/**
 * @brief Determines if there is a gesture available for reading
 *
 * @return True if gesture available. False otherwise.
 */
bool SparkFun_APDS9960::isGestureAvailable()
{
    uint8_t val;

    /* Read value from GSTATUS register */
    if (!wireReadDataByte(APDS9960_GSTATUS, val))
    {
        return ERROR;
    }

    /* Shift and mask out GVALID bit */
    val &= APDS9960_GVALID;

    /* Return true/false based on GVALID bit */
    if (val == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Processes a gesture event and returns best guessed gesture
 *
 * @return Number corresponding to gesture. -1 on error.
 */
int SparkFun_APDS9960::readGesture()
{
    uint8_t fifo_level = 0;
    uint8_t fifo_data[128];
    uint8_t gstatus;
    int bytes_read = 0;
    int motion;
    int i;

    /* Make sure that power and gesture is on and data is valid */
    if (!isGestureAvailable() || !(getMode() & 0b01000001))
    {
        return DIR_NONE;
    }

    /* Keep looping as long as gesture data is valid */
    while (1)
    {

        /* Wait some time to collect next batch of FIFO data */
        HAL_Delay(FIFO_PAUSE_TIME);

        /* Get the contents of the STATUS register. Is data still valid? */
        if (!wireReadDataByte(APDS9960_GSTATUS, gstatus))
        {
            return ERROR;
        }

        /* If we have valid data, read in FIFO */
        if ((gstatus & APDS9960_GVALID) == APDS9960_GVALID)
        {

            /* Read the current FIFO level */
            if (!wireReadDataByte(APDS9960_GFLVL, fifo_level))
            {
                return ERROR;
            }

#if DEBUG
            Serial.print("FIFO Level: ");
            Serial.println(fifo_level);
#endif

            /* If there's stuff in the FIFO, read it into our data block */
            if (fifo_level > 0)
            {
                bytes_read = wireReadDataBlock(APDS9960_GFIFO_U,
                                               (uint8_t *)fifo_data,
                                               (fifo_level * 4));
                if (bytes_read == -1)
                {
                    return ERROR;
                }
#if DEBUG
                Serial.print("FIFO Dump: ");
                for (i = 0; i < bytes_read; i++)
                {
                    Serial.print(fifo_data[i]);
                    Serial.print(" ");
                }
                Serial.println();
#endif

                /* If at least 1 set of data, sort the data into U/D/L/R */
                if (bytes_read >= 4)
                {
                    for (i = 0; i < bytes_read; i += 4)
                    {
                        gesture_data_.u_data[gesture_data_.index] =
                            fifo_data[i + 0];
                        gesture_data_.d_data[gesture_data_.index] =
                            fifo_data[i + 1];
                        gesture_data_.l_data[gesture_data_.index] =
                            fifo_data[i + 2];
                        gesture_data_.r_data[gesture_data_.index] =
                            fifo_data[i + 3];
                        gesture_data_.index++;
                        gesture_data_.total_gestures++;
                    }

#if DEBUG
                    Serial.print("Up Data: ");
                    for (i = 0; i < gesture_data_.total_gestures; i++)
                    {
                        Serial.print(gesture_data_.u_data[i]);
                        Serial.print(" ");
                    }
                    Serial.println();
#endif

                    /* Filter and process gesture data. Decode near/far state */
                    if (processGestureData())
                    {
                        if (decodeGesture())
                        {
                            //***TODO: U-Turn Gestures
#if DEBUG
                            // Serial.println(gesture_motion_);
#endif
                        }
                    }

                    /* Reset data */
                    gesture_data_.index = 0;
                    gesture_data_.total_gestures = 0;
                }
            }
        }
        else
        {

            /* Determine best guessed gesture and clean up */
            HAL_Delay(FIFO_PAUSE_TIME);
            decodeGesture();
            motion = gesture_motion_;
#if DEBUG
            Serial.print("END: ");
            Serial.println(gesture_motion_);
#endif
            resetGestureParameters();
            return motion;
        }
    }
}

/**
 * Turn the APDS-9960 on
 *
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::enablePower()
{
    if (!setMode(POWER, 1))
    {
        return false;
    }

    return true;
}

/**
 * Turn the APDS-9960 off
 *
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::disablePower()
{
    if (!setMode(POWER, 0))
    {
        return false;
    }

    return true;
}

/*******************************************************************************
 * Ambient light and color sensor controls
 ******************************************************************************/

/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::readAmbientLight(uint16_t &val)
{
    uint8_t val_byte;
    val = 0;

    /* Read value from clear channel, low byte register */
    if (!wireReadDataByte(APDS9960_CDATAL, val_byte))
    {
        return false;
    }
    val = val_byte;

    /* Read value from clear channel, high byte register */
    if (!wireReadDataByte(APDS9960_CDATAH, val_byte))
    {
        return false;
    }
    val = val + ((uint16_t)val_byte << 8);

    return true;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::readRedLight(uint16_t &val)
{
    uint8_t val_byte;
    val = 0;

    /* Read value from clear channel, low byte register */
    if (!wireReadDataByte(APDS9960_RDATAL, val_byte))
    {
        return false;
    }
    val = val_byte;

    /* Read value from clear channel, high byte register */
    if (!wireReadDataByte(APDS9960_RDATAH, val_byte))
    {
        return false;
    }
    val = val + ((uint16_t)val_byte << 8);

    return true;
}

/**
 * @brief Reads the green light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::readGreenLight(uint16_t &val)
{
    uint8_t val_byte;
    val = 0;

    /* Read value from clear channel, low byte register */
    if (!wireReadDataByte(APDS9960_GDATAL, val_byte))
    {
        return false;
    }
    val = val_byte;

    /* Read value from clear channel, high byte register */
    if (!wireReadDataByte(APDS9960_GDATAH, val_byte))
    {
        return false;
    }
    val = val + ((uint16_t)val_byte << 8);

    return true;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::readBlueLight(uint16_t &val)
{
    uint8_t val_byte;
    val = 0;

    /* Read value from clear channel, low byte register */
    if (!wireReadDataByte(APDS9960_BDATAL, val_byte))
    {
        return false;
    }
    val = val_byte;

    /* Read value from clear channel, high byte register */
    if (!wireReadDataByte(APDS9960_BDATAH, val_byte))
    {
        return false;
    }
    val = val + ((uint16_t)val_byte << 8);

    return true;
}

/*******************************************************************************
 * Proximity sensor controls
 ******************************************************************************/

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @param[out] val value of the proximity sensor.
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::readProximity(uint8_t &val)
{
    val = 0;

    /* Read value from proximity data register */
    if (!wireReadDataByte(APDS9960_PDATA, val))
    {
        return false;
    }

    return true;
}

/*******************************************************************************
 * High-level gesture controls
 ******************************************************************************/

/**
 * @brief Resets all the parameters in the gesture data member
 */
void SparkFun_APDS9960::resetGestureParameters()
{
    gesture_data_.index = 0;
    gesture_data_.total_gestures = 0;

    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;

    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;

    gesture_near_count_ = 0;
    gesture_far_count_ = 0;

    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}

/**
 * @brief Processes the raw gesture data to determine swipe direction
 *
 * @return True if near or far state seen. False otherwise.
 */
bool SparkFun_APDS9960::processGestureData()
{
    uint8_t u_first = 1;
    uint8_t d_first = 1;
    uint8_t l_first = 1;
    uint8_t r_first = 1;
    uint8_t u_last = 1;
    uint8_t d_last = 1;
    uint8_t l_last = 1;
    uint8_t r_last = 1;
    int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;

    /* If we have less than 4 total gestures, that's not enough */
    if (gesture_data_.total_gestures <= 4)
    {
        return false;
    }

    /* Check to make sure our data isn't out of bounds */
    if ((gesture_data_.total_gestures <= 32) &&
        (gesture_data_.total_gestures > 0))
    {

        /* Find the first value in U/D/L/R above the threshold */
        for (i = 0; i < gesture_data_.total_gestures; i++)
        {
            if (((gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                 (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT)) ||
                ((gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                 (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT)))
            {

                u_first = gesture_data_.u_data[i];
                d_first = gesture_data_.d_data[i];
                l_first = gesture_data_.l_data[i];
                r_first = gesture_data_.r_data[i];
                break;
            }
        }

        /* Find the last value in U/D/L/R above the threshold */
        for (i = gesture_data_.total_gestures - 1; i >= 0; i--)
        {
#if DEBUG
            Serial.print(F("Finding last: "));
            Serial.print(F("U:"));
            Serial.print(gesture_data_.u_data[i]);
            Serial.print(F(" D:"));
            Serial.print(gesture_data_.d_data[i]);
            Serial.print(F(" L:"));
            Serial.print(gesture_data_.l_data[i]);
            Serial.print(F(" R:"));
            Serial.println(gesture_data_.r_data[i]);
#endif
            if (((gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                 (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT)) ||
                ((gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                 (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT)))
            {

                u_last = gesture_data_.u_data[i];
                d_last = gesture_data_.d_data[i];
                l_last = gesture_data_.l_data[i];
                r_last = gesture_data_.r_data[i];
                break;
            }
        }
    }

    /* Calculate the first vs. last ratio of up/down and left/right */
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);

#if DEBUG
    Serial.print(F("Last Values: "));
    Serial.print(F("U:"));
    Serial.print(u_last);
    Serial.print(F(" D:"));
    Serial.print(d_last);
    Serial.print(F(" L:"));
    Serial.print(l_last);
    Serial.print(F(" R:"));
    Serial.println(r_last);

    Serial.print(F("Ratios: "));
    Serial.print(F("UD Fi: "));
    Serial.print(ud_ratio_first);
    Serial.print(F(" UD La: "));
    Serial.print(ud_ratio_last);
    Serial.print(F(" LR Fi: "));
    Serial.print(lr_ratio_first);
    Serial.print(F(" LR La: "));
    Serial.println(lr_ratio_last);
#endif

    /* Determine the difference between the first and last ratios */
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;

#if DEBUG
    Serial.print("Deltas: ");
    Serial.print("UD: ");
    Serial.print(ud_delta);
    Serial.print(" LR: ");
    Serial.println(lr_delta);
#endif

    /* Accumulate the UD and LR delta values */
    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;

#if DEBUG
    Serial.print("Accumulations: ");
    Serial.print("UD: ");
    Serial.print(gesture_ud_delta_);
    Serial.print(" LR: ");
    Serial.println(gesture_lr_delta_);
#endif

    /* Determine U/D gesture */
    if (gesture_ud_delta_ >= GESTURE_SENSITIVITY_1)
    {
        gesture_ud_count_ = 1;
    }
    else if (gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1)
    {
        gesture_ud_count_ = -1;
    }
    else
    {
        gesture_ud_count_ = 0;
    }

    /* Determine L/R gesture */
    if (gesture_lr_delta_ >= GESTURE_SENSITIVITY_1)
    {
        gesture_lr_count_ = 1;
    }
    else if (gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1)
    {
        gesture_lr_count_ = -1;
    }
    else
    {
        gesture_lr_count_ = 0;
    }

    /* Determine Near/Far gesture */
    if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == 0))
    {
        if ((abs(ud_delta) < GESTURE_SENSITIVITY_2) &&
            (abs(lr_delta) < GESTURE_SENSITIVITY_2))
        {

            if ((ud_delta == 0) && (lr_delta == 0))
            {
                gesture_near_count_++;
            }
            else if ((ud_delta != 0) || (lr_delta != 0))
            {
                gesture_far_count_++;
            }

            if ((gesture_near_count_ >= 10) && (gesture_far_count_ >= 2))
            {
                if ((ud_delta == 0) && (lr_delta == 0))
                {
                    gesture_state_ = NEAR_STATE;
                }
                else if ((ud_delta != 0) && (lr_delta != 0))
                {
                    gesture_state_ = FAR_STATE;
                }
                return true;
            }
        }
    }
    else
    {
        if ((abs(ud_delta) < GESTURE_SENSITIVITY_2) &&
            (abs(lr_delta) < GESTURE_SENSITIVITY_2))
        {

            if ((ud_delta == 0) && (lr_delta == 0))
            {
                gesture_near_count_++;
            }

            if (gesture_near_count_ >= 10)
            {
                gesture_ud_count_ = 0;
                gesture_lr_count_ = 0;
                gesture_ud_delta_ = 0;
                gesture_lr_delta_ = 0;
            }
        }
    }

#if DEBUG
    Serial.print("UD_CT: ");
    Serial.print(gesture_ud_count_);
    Serial.print(" LR_CT: ");
    Serial.print(gesture_lr_count_);
    Serial.print(" NEAR_CT: ");
    Serial.print(gesture_near_count_);
    Serial.print(" FAR_CT: ");
    Serial.println(gesture_far_count_);
    Serial.println("----------");
#endif

    return false;
}

/**
 * @brief Determines swipe direction or near/far state
 *
 * @return True if near/far event. False otherwise.
 */
bool SparkFun_APDS9960::decodeGesture()
{
    /* Return if near or far event is detected */
    if (gesture_state_ == NEAR_STATE)
    {
        gesture_motion_ = DIR_NEAR;
        return true;
    }
    else if (gesture_state_ == FAR_STATE)
    {
        gesture_motion_ = DIR_FAR;
        return true;
    }

    /* Determine swipe direction */
    if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == 0))
    {
        gesture_motion_ = DIR_UP;
    }
    else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == 0))
    {
        gesture_motion_ = DIR_DOWN;
    }
    else if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == 1))
    {
        gesture_motion_ = DIR_RIGHT;
    }
    else if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == -1))
    {
        gesture_motion_ = DIR_LEFT;
    }
    else if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == 1))
    {
        if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_))
        {
            gesture_motion_ = DIR_UP;
        }
        else
        {
            gesture_motion_ = DIR_RIGHT;
        }
    }
    else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == -1))
    {
        if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_))
        {
            gesture_motion_ = DIR_DOWN;
        }
        else
        {
            gesture_motion_ = DIR_LEFT;
        }
    }
    else if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == -1))
    {
        if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_))
        {
            gesture_motion_ = DIR_UP;
        }
        else
        {
            gesture_motion_ = DIR_LEFT;
        }
    }
    else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == 1))
    {
        if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_))
        {
            gesture_motion_ = DIR_DOWN;
        }
        else
        {
            gesture_motion_ = DIR_RIGHT;
        }
    }
    else
    {
        return false;
    }

    return true;
}

/*******************************************************************************
 * Getters and setters for register values
 ******************************************************************************/

/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @return lower threshold
 */
uint8_t SparkFun_APDS9960::getProxIntLowThresh()
{
    uint8_t val;

    /* Read value from PILT register */
    if (!wireReadDataByte(APDS9960_PILT, val))
    {
        val = 0;
    }

    return val;
}

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] threshold the lower proximity threshold
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setProxIntLowThresh(uint8_t threshold)
{
    if (!wireWriteDataByte(APDS9960_PILT, threshold))
    {
        return false;
    }

    return true;
}

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @return high threshold
 */
uint8_t SparkFun_APDS9960::getProxIntHighThresh()
{
    uint8_t val;

    /* Read value from PIHT register */
    if (!wireReadDataByte(APDS9960_PIHT, val))
    {
        val = 0;
    }

    return val;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setProxIntHighThresh(uint8_t threshold)
{
    if (!wireWriteDataByte(APDS9960_PIHT, threshold))
    {
        return false;
    }

    return true;
}

/**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the value of the LED drive strength. 0xFF on failure.
 */
uint8_t SparkFun_APDS9960::getLEDDrive()
{
    uint8_t val;

    /* Read value from CONTROL register */
    if (!wireReadDataByte(APDS9960_CONTROL, val))
    {
        return ERROR;
    }

    /* Shift and mask out LED drive bits */
    val = (val >> 6) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setLEDDrive(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if (!wireReadDataByte(APDS9960_CONTROL, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 6;
    val &= 0b00111111;
    val |= drive;

    /* Write register value back into CONTROL register */
    if (!wireWriteDataByte(APDS9960_CONTROL, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the value of the proximity gain. 0xFF on failure.
 */
uint8_t SparkFun_APDS9960::getProximityGain()
{
    uint8_t val;

    /* Read value from CONTROL register */
    if (!wireReadDataByte(APDS9960_CONTROL, val))
    {
        return ERROR;
    }

    /* Shift and mask out PDRIVE bits */
    val = (val >> 2) & 0b00000011;

    return val;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setProximityGain(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if (!wireReadDataByte(APDS9960_CONTROL, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 2;
    val &= 0b11110011;
    val |= drive;

    /* Write register value back into CONTROL register */
    if (!wireWriteDataByte(APDS9960_CONTROL, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @return the value of the ALS gain. 0xFF on failure.
 */
uint8_t SparkFun_APDS9960::getAmbientLightGain()
{
    uint8_t val;

    /* Read value from CONTROL register */
    if (!wireReadDataByte(APDS9960_CONTROL, val))
    {
        return ERROR;
    }

    /* Shift and mask out ADRIVE bits */
    val &= 0b00000011;

    return val;
}

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setAmbientLightGain(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if (!wireReadDataByte(APDS9960_CONTROL, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    val &= 0b11111100;
    val |= drive;

    /* Write register value back into CONTROL register */
    if (!wireWriteDataByte(APDS9960_CONTROL, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Get the current LED boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @return The LED boost value. 0xFF on failure.
 */
uint8_t SparkFun_APDS9960::getLEDBoost()
{
    uint8_t val;

    /* Read value from CONFIG2 register */
    if (!wireReadDataByte(APDS9960_CONFIG2, val))
    {
        return ERROR;
    }

    /* Shift and mask out LED_BOOST bits */
    val = (val >> 4) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setLEDBoost(uint8_t boost)
{
    uint8_t val;

    /* Read value from CONFIG2 register */
    if (!wireReadDataByte(APDS9960_CONFIG2, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    boost &= 0b00000011;
    boost = boost << 4;
    val &= 0b11001111;
    val |= boost;

    /* Write register value back into CONFIG2 register */
    if (!wireWriteDataByte(APDS9960_CONFIG2, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets proximity gain compensation enable
 *
 * @return 1 if compensation is enabled. 0 if not. 0xFF on error.
 */
uint8_t SparkFun_APDS9960::getProxGainCompEnable()
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if (!wireReadDataByte(APDS9960_CONFIG3, val))
    {
        return ERROR;
    }

    /* Shift and mask out PCMP bits */
    val = (val >> 5) & 0b00000001;

    return val;
}

/**
 * @brief Sets the proximity gain compensation enable
 *
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setProxGainCompEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if (!wireReadDataByte(APDS9960_CONFIG3, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 5;
    val &= 0b11011111;
    val |= enable;

    /* Write register value back into CONFIG3 register */
    if (!wireWriteDataByte(APDS9960_CONFIG3, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the current mask for enabled/disabled proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @return Current proximity mask for photodiodes. 0xFF on error.
 */
uint8_t SparkFun_APDS9960::getProxPhotoMask()
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if (!wireReadDataByte(APDS9960_CONFIG3, val))
    {
        return ERROR;
    }

    /* Mask out photodiode enable mask bits */
    val &= 0b00001111;

    return val;
}

/**
 * @brief Sets the mask for enabling/disabling proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] mask 4-bit mask value
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setProxPhotoMask(uint8_t mask)
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if (!wireReadDataByte(APDS9960_CONFIG3, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    mask &= 0b00001111;
    val &= 0b11110000;
    val |= mask;

    /* Write register value back into CONFIG3 register */
    if (!wireWriteDataByte(APDS9960_CONFIG3, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the entry proximity threshold for gesture sensing
 *
 * @return Current entry proximity threshold.
 */
uint8_t SparkFun_APDS9960::getGestureEnterThresh()
{
    uint8_t val;

    /* Read value from GPENTH register */
    if (!wireReadDataByte(APDS9960_GPENTH, val))
    {
        val = 0;
    }

    return val;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to start gesture mode
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setGestureEnterThresh(uint8_t threshold)
{
    if (!wireWriteDataByte(APDS9960_GPENTH, threshold))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the exit proximity threshold for gesture sensing
 *
 * @return Current exit proximity threshold.
 */
uint8_t SparkFun_APDS9960::getGestureExitThresh()
{
    uint8_t val;

    /* Read value from GEXTH register */
    if (!wireReadDataByte(APDS9960_GEXTH, val))
    {
        val = 0;
    }

    return val;
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to end gesture mode
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setGestureExitThresh(uint8_t threshold)
{
    if (!wireWriteDataByte(APDS9960_GEXTH, threshold))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the current photodiode gain. 0xFF on error.
 */
uint8_t SparkFun_APDS9960::getGestureGain()
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if (!wireReadDataByte(APDS9960_GCONF2, val))
    {
        return ERROR;
    }

    /* Shift and mask out GGAIN bits */
    val = (val >> 5) & 0b00000011;

    return val;
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setGestureGain(uint8_t gain)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if (!wireReadDataByte(APDS9960_GCONF2, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    gain &= 0b00000011;
    gain = gain << 5;
    val &= 0b10011111;
    val |= gain;

    /* Write register value back into GCONF2 register */
    if (!wireWriteDataByte(APDS9960_GCONF2, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the drive current of the LED during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the LED drive current value. 0xFF on error.
 */
uint8_t SparkFun_APDS9960::getGestureLEDDrive()
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if (!wireReadDataByte(APDS9960_GCONF2, val))
    {
        return ERROR;
    }

    /* Shift and mask out GLDRIVE bits */
    val = (val >> 3) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setGestureLEDDrive(uint8_t drive)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if (!wireReadDataByte(APDS9960_GCONF2, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 3;
    val &= 0b11100111;
    val |= drive;

    /* Write register value back into GCONF2 register */
    if (!wireWriteDataByte(APDS9960_GCONF2, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @return the current wait time between gestures. 0xFF on error.
 */
uint8_t SparkFun_APDS9960::getGestureWaitTime()
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if (!wireReadDataByte(APDS9960_GCONF2, val))
    {
        return ERROR;
    }

    /* Mask out GWTIME bits */
    val &= 0b00000111;

    return val;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setGestureWaitTime(uint8_t time)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if (!wireReadDataByte(APDS9960_GCONF2, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    time &= 0b00000111;
    val &= 0b11111000;
    val |= time;

    /* Write register value back into GCONF2 register */
    if (!wireWriteDataByte(APDS9960_GCONF2, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the low threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::getLightIntLowThreshold(uint16_t &threshold)
{
    uint8_t val_byte;
    threshold = 0;

    /* Read value from ambient light low threshold, low byte register */
    if (!wireReadDataByte(APDS9960_AILTL, val_byte))
    {
        return false;
    }
    threshold = val_byte;

    /* Read value from ambient light low threshold, high byte register */
    if (!wireReadDataByte(APDS9960_AILTH, val_byte))
    {
        return false;
    }
    threshold = threshold + ((uint16_t)val_byte << 8);

    return true;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setLightIntLowThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if (!wireWriteDataByte(APDS9960_AILTL, val_low))
    {
        return false;
    }

    /* Write high byte */
    if (!wireWriteDataByte(APDS9960_AILTH, val_high))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the high threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::getLightIntHighThreshold(uint16_t &threshold)
{
    uint8_t val_byte;
    threshold = 0;

    /* Read value from ambient light high threshold, low byte register */
    if (!wireReadDataByte(APDS9960_AIHTL, val_byte))
    {
        return false;
    }
    threshold = val_byte;

    /* Read value from ambient light high threshold, high byte register */
    if (!wireReadDataByte(APDS9960_AIHTH, val_byte))
    {
        return false;
    }
    threshold = threshold + ((uint16_t)val_byte << 8);

    return true;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setLightIntHighThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if (!wireWriteDataByte(APDS9960_AIHTL, val_low))
    {
        return false;
    }

    /* Write high byte */
    if (!wireWriteDataByte(APDS9960_AIHTH, val_high))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the low threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::getProximityIntLowThreshold(uint8_t &threshold)
{
    threshold = 0;

    /* Read value from proximity low threshold register */
    if (!wireReadDataByte(APDS9960_PILT, threshold))
    {
        return false;
    }

    return true;
}

/**
 * @brief Sets the low threshold for proximity interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setProximityIntLowThreshold(uint8_t threshold)
{

    /* Write threshold value to register */
    if (!wireWriteDataByte(APDS9960_PILT, threshold))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets the high threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::getProximityIntHighThreshold(uint8_t &threshold)
{
    threshold = 0;

    /* Read value from proximity low threshold register */
    if (!wireReadDataByte(APDS9960_PIHT, threshold))
    {
        return false;
    }

    return true;
}

/**
 * @brief Sets the high threshold for proximity interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setProximityIntHighThreshold(uint8_t threshold)
{

    /* Write threshold value to register */
    if (!wireWriteDataByte(APDS9960_PIHT, threshold))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets if ambient light interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t SparkFun_APDS9960::getAmbientLightIntEnable()
{
    uint8_t val;

    /* Read value from ENABLE register */
    if (!wireReadDataByte(APDS9960_ENABLE, val))
    {
        return ERROR;
    }

    /* Shift and mask out AIEN bit */
    val = (val >> 4) & 0b00000001;

    return val;
}

/**
 * @brief Turns ambient light interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setAmbientLightIntEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if (!wireReadDataByte(APDS9960_ENABLE, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 4;
    val &= 0b11101111;
    val |= enable;

    /* Write register value back into ENABLE register */
    if (!wireWriteDataByte(APDS9960_ENABLE, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t SparkFun_APDS9960::getProximityIntEnable()
{
    uint8_t val;

    /* Read value from ENABLE register */
    if (!wireReadDataByte(APDS9960_ENABLE, val))
    {
        return ERROR;
    }

    /* Shift and mask out PIEN bit */
    val = (val >> 5) & 0b00000001;

    return val;
}

/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setProximityIntEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if (!wireReadDataByte(APDS9960_ENABLE, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 5;
    val &= 0b11011111;
    val |= enable;

    /* Write register value back into ENABLE register */
    if (!wireWriteDataByte(APDS9960_ENABLE, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Gets if gesture interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t SparkFun_APDS9960::getGestureIntEnable()
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if (!wireReadDataByte(APDS9960_GCONF4, val))
    {
        return ERROR;
    }

    /* Shift and mask out GIEN bit */
    val = (val >> 1) & 0b00000001;

    return val;
}

/**
 * @brief Turns gesture-related interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setGestureIntEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if (!wireReadDataByte(APDS9960_GCONF4, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 1;
    val &= 0b11111101;
    val |= enable;

    /* Write register value back into GCONF4 register */
    if (!wireWriteDataByte(APDS9960_GCONF4, val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Clears the ambient light interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
bool SparkFun_APDS9960::clearAmbientLightInt()
{
    uint8_t throwaway;
    if (!wireReadDataByte(APDS9960_AICLEAR, throwaway))
    {
        return false;
    }

    return true;
}

/**
 * @brief Clears the proximity interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
bool SparkFun_APDS9960::clearProximityInt()
{
    uint8_t throwaway;
    if (!wireReadDataByte(APDS9960_PICLEAR, throwaway))
    {
        return false;
    }

    return true;
}

/**
 * @brief Tells if the gesture state machine is currently running
 *
 * @return 1 if gesture state machine is running, 0 if not. 0xFF on error.
 */
uint8_t SparkFun_APDS9960::getGestureMode()
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if (!wireReadDataByte(APDS9960_GCONF4, val))
    {
        return ERROR;
    }

    /* Mask out GMODE bit */
    val &= 0b00000001;

    return val;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 *
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return True if operation successful. False otherwise.
 */
bool SparkFun_APDS9960::setGestureMode(uint8_t mode)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if (!wireReadDataByte(APDS9960_GCONF4, val))
    {
        return false;
    }

    /* Set bits in register to given value */
    mode &= 0b00000001;
    val &= 0b11111110;
    val |= mode;

    /* Write register value back into GCONF4 register */
    if (!wireWriteDataByte(APDS9960_GCONF4, val))
    {
        return false;
    }

    return true;
}

/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

/**
 * @brief Writes a single byte to the I2C device (no register)
 *
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool SparkFun_APDS9960::wireWriteByte(uint8_t val)
{
    return (HAL_I2C_Master_Transmit(&i2cHandle, APDS9960_I2C_ADDR,
                                    &val, 1, 1000) == HAL_OK);
}

/**
 * @brief Writes a single byte to the I2C device and specified register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool SparkFun_APDS9960::wireWriteDataByte(uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(&i2cHandle, APDS9960_I2C_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1, 1000) == HAL_OK);
}

/**
 * @brief Writes a block (array) of bytes to the I2C device and register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val pointer to the beginning of the data byte array
 * @param[in] len the length (in bytes) of the data to write
 * @return True if successful write operation. False otherwise.
 */
bool SparkFun_APDS9960::wireWriteDataBlock(uint8_t reg,
                                           uint8_t *val,
                                           unsigned int len)
{
    // Unused
    return (HAL_I2C_Mem_Write(&i2cHandle, APDS9960_I2C_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, val, len, 1000) == HAL_OK);
}

/**
 * @brief Reads a single byte from the I2C device and specified register
 *
 * @param[in] reg the register to read from
 * @param[out] the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
bool SparkFun_APDS9960::wireReadDataByte(uint8_t reg, uint8_t &val)
{
    return (HAL_I2C_Mem_Read(&i2cHandle, APDS9960_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 1000) == HAL_OK);
}

/**
 * @brief Reads a block (array) of bytes from the I2C device and register
 *
 * @param[in] reg the register to read from
 * @param[out] val pointer to the beginning of the data
 * @param[in] len number of bytes to read
 * @return Number of bytes read. -1 on read error.
 */
int SparkFun_APDS9960::wireReadDataBlock(uint8_t reg,
                                         uint8_t *val,
                                         unsigned int len)
{
    if (HAL_I2C_Mem_Read(&i2cHandle, APDS9960_I2C_ADDR, reg,
                         I2C_MEMADD_SIZE_8BIT, val, len, 1000) == HAL_OK)
        return -1;

    return len;
}
