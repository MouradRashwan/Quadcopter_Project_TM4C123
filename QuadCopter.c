/*
 *
 * QuadCopter Main Program
 *
 */

#include <math.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

#include "driverlib/eeprom.h"

#include "MCAL_layer/COMMON_driver/COMMON_driver.h"
#include "MCAL_layer/EEPROM_driver/EEPROM_driver.h"

#include "HAL_layer/MPU_driver/MPU_driver.h"
#include "HAL_layer/MOTOR_driver/MOTOR_driver.h"
#include "HAL_layer/DEBUG_driver/DEBUG_driver.h"
#include "HAL_layer/REMOTE_driver/REMOTE_driver.h"

#include "APP_layer/PID_driver/PID_driver.h"
#include "APP_layer/SensorFusion_driver/SensorFusion_driver.h"

/*
 *
 * Constants Definitions
 *
 */
#define ALTITUDE_BMP280_ENABLE      0
#define ORIENTATION_MPU9250_ENABLE  0

#define CALIBRATION_ENABLE          0
#define FILTERING_MODE              3 /* 1 -> Mahony [Less Accurate Less Power] || 2 -> Madgwick [More Accurate More Power] || 3 -> Mahony & Madgwick */

#define DEBUG_ENABLE                1
#define DEBUG_MAIN_WAIT_LOOPS       10

#define MOTOR_DELTA_MICROS          100U /* for Increase/Decrease Motor speed in us */

#define DEBUG_UART_BUADRATE     115200U
#define REMOTE_UART_BUADRATE    115200U

#define EEPROM_FLAG_ADDRESS     0
#define EEPROM_DATA_ADDRESS     4U
#define EEPROM_FLAG_VAL         0xABCDEF00U /* Change this value to write the below PID values to the EEPROM */

#define ROLL_PID_KP             (1.1f)
#define ROLL_PID_KI             (2.2f)
#define ROLL_PID_KD             (3.3f)
#define ROLL_PID_MODE           (MODE_PID)
#define ROLL_PID_OUT_MIN        (-25000.0f)
#define ROLL_PID_OUT_MAX        (25000.0f)

#define PITCH_PID_KP            (1.1f)
#define PITCH_PID_KI            (2.2f)
#define PITCH_PID_KD            (3.3f)
#define PITCH_PID_MODE          (MODE_PID)
#define PITCH_PID_OUT_MIN       (-25000.0f)
#define PITCH_PID_OUT_MAX       (25000.0f)

#define YAW_PID_KP              (1.1f)
#define YAW_PID_KI              (2.2f)
#define YAW_PID_KD              (3.3f)
#define YAW_PID_MODE            (MODE_PID)
#define YAW_PID_OUT_MIN         (-25000.0f)
#define YAW_PID_OUT_MAX         (25000.0f)

#define ALTITUDE_PID_KP         (1.1f)
#define ALTITUDE_PID_KI         (2.2f)
#define ALTITUDE_PID_KD         (3.3f)
#define ALTITUDE_PID_MODE       (MODE_PID)
#define ALTITUDE_PID_OUT_MIN    (-25000.0f)
#define ALTITUDE_PID_OUT_MAX    (25000.0f)

/*
 *
 * Macro Functions
 *
 */
#define SYS_GET_SECONDS_FROM_TICKS(TICKS)   ((TICKS) * (1.0f / SysCtlClockGet()))

/*
 *
 * Structure Definitions
 *
 */
typedef struct IntGain
{
    uint32_t ui32KP;
    uint32_t ui32KI;
    uint32_t ui32KD;
} IntGain_t;

typedef struct IntGains
{
    IntGain_t tIntGainROLL;
    IntGain_t tIntGainPITCH;
    IntGain_t tIntGainYAW;
    IntGain_t tIntGainALTITUDE;
} IntGains_t; /* Size of 48 Bytes */

typedef struct FloatGains
{
    PIDgain_t tPIDgainROLL;
    PIDgain_t tPIDgainPITCH;
    PIDgain_t tPIDgainYAW;
    PIDgain_t tPIDgainALTITUDE;
} FloatGains_t; /* Size of 48 Bytes */

/*
 *
 * APIs Declarations
 *
 */
void Tiva_init(void);
void LEDs_init(void);
void WTimer0A_init(void);
void EEPROM_initReadWrite(IntGains_t * ptIntGains, FloatGains_t * ptFloatGains);
void MOTOR_init(void);
void MPU_init(void);

void LED_setOFF(void);
void LED_setRED(void);
void LED_setBLUE(void);
void LED_setGREEN(void);
void LED_toggleGREEN(void);
uint64_t WTimer0A_getCurrent(void);

void GAIN_FloatToInt(IntGains_t *ptIntGains, FloatGains_t *tpPIDgains);
void GAIN_IntToFloat(FloatGains_t *ptFloatGains, IntGains_t *ptIntGains);
void REMOTE_process(const int32_t i32Command);
DataStatus_t MPU_readRawData(MPU9AxisRawData_t * const ptMPU9AxisRawData,
                             BMP280Press_t * const ptBMP280Press);
void PID_updateInput(const YawPitchRoll_t * const ptActualYPR,
                     const BMP280AltitudeInM_t * const ptActualAltitude);
void PID_updateOutput(const float dPeriodInSec);
void QUAD_updateOutput(void);

/*
 *
 * Operations Status - Desired Altitude - Desired Orientation - PID Objects - PID Gains
 *
 */
uint32_t g_ui32OperationFlag = 0;
BMP280AltitudeInM_t g_tDesiredAltitude = { 1.0f };
YawPitchRoll_t g_tDesiredYPR = { 0.0f, 0.0f, 0.0f };
PIDobject_t g_tPIDobjectROLL, g_tPIDobjectPITCH, g_tPIDobjectYAW,
        g_tPIDobjectALTITUDE;
IntGains_t g_tIntGains;
FloatGains_t g_tFloatGains = { { ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD }, {
        PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD },
                               { YAW_PID_KP, YAW_PID_KI, YAW_PID_KD }, {
                                       ALTITUDE_PID_KP, ALTITUDE_PID_KI,
                                       ALTITUDE_PID_KD } };

/************************************* MAIN-PROGRAM ************************************************/
void main(void)
{
    uint32_t ui32WaitLoops = 0;
    int32_t i32Command = -1;
    PIgain_t tPIgain = { 10.0f, 0.0f };
    float dPeriodInSec, dGyroMeasError;
    uint64_t ui64Current, ui64Last = UINT64_MAX;
    MPU9AxisRawData_t tMPU9AxisRawData;
    MPU9AxisInUnits_t tMPU9AxisInUnits1, tMPU9AxisInUnits2;
    BMP280Press_t tPressRawData;
    BMP280PressInMBar_t tBMP280PressInMBar;
    YawPitchRoll_t tActualYPR, tActualYPRMahony, tActualYPRMadgwick;
    BMP280AltitudeInM_t tActualAltitude = { 0 };
    Quaternion_t tQuaternionMahony, tQuaternionMadgwick;

    PIDmode_t tPIDmodeROLL = ROLL_PID_MODE, tPIDmodePITCH = PITCH_PID_MODE,
            tPIDmodeYAW = YAW_PID_MODE, tPIDmodeALTITUDE = ALTITUDE_PID_MODE;

    PIDoutBound_t tPIDoutBoundROLL = { ROLL_PID_OUT_MIN, ROLL_PID_OUT_MAX },
            tPIDoutBoundPITCH = { PITCH_PID_OUT_MIN, PITCH_PID_OUT_MAX },
            tPIDoutBoundYAW = { YAW_PID_OUT_MIN, YAW_PID_OUT_MAX },
            tPIDoutBoundALTITUDE =
                    { ALTITUDE_PID_OUT_MIN, ALTITUDE_PID_OUT_MAX };

    /* Initialize System clock to 25 MHz */
    Tiva_init();

    /* Initialize PORTF pins F1 & F2 & F3 OUTPUTS */
    LEDs_init();

    /* Initialize Debug port UART0 Buadrate to 115200 */
    DEBUG_init(DEBUG_UART_BUADRATE);

#if(DEBUG_ENABLE == 1)
    DEBUG_print("SYSTEM STARTED ... \n\n");
    delayMillis(1000);
#endif

    /* Initialize MOTOR module */
    MOTOR_init();

    /* Initialize REMOTE module UART3 Buadrate to 115200 */
    REMOTE_init(REMOTE_UART_BUADRATE);

#if(ORIENTATION_MPU9250_ENABLE == 1)
    /* Initialize, Self-test and Calibrate all MPUs */
    MPU_init();
#endif

    /* Get gyroscope measure error to be used in MPU Fusion algorithm */
    dGyroMeasError = MPU9250_getGyroMeasError();

    /* Initialize Quaternion */
#if(FILTERING_MODE == 1)
    SensorFusion_initQuaternion(&tQuaternionMahony);
#elif(FILTERING_MODE == 2)
    SensorFusion_initQuaternion(&tQuaternionMadgwick);
#elif(FILTERING_MODE == 3)
    SensorFusion_initQuaternion(&tQuaternionMahony);
    SensorFusion_initQuaternion(&tQuaternionMadgwick);
#else
#error("Please Set a Correct Filtering Mode of [1 or 2 or 3] ");
#endif

    /* Initialize EEPROM, Check If erased then write the default PID gains | If not erased then check the first word flag then read/write PID gains */
    EEPROM_initReadWrite(&g_tIntGains, &g_tFloatGains);

    /* Initialize PID for ROLL & PITCH & YAW */
    PID_init(&g_tPIDobjectROLL, tPIDmodeROLL, g_tFloatGains.tPIDgainROLL,
             tPIDoutBoundROLL);
    PID_init(&g_tPIDobjectPITCH, tPIDmodePITCH, g_tFloatGains.tPIDgainPITCH,
             tPIDoutBoundPITCH);
    PID_init(&g_tPIDobjectYAW, tPIDmodeYAW, g_tFloatGains.tPIDgainYAW,
             tPIDoutBoundYAW);
    PID_init(&g_tPIDobjectALTITUDE, tPIDmodeALTITUDE,
             g_tFloatGains.tPIDgainALTITUDE, tPIDoutBoundALTITUDE);

    /* Initialize and Start 64-bit WTimer0 that will never reset */
    WTimer0A_init();

    /************************************************* EMBEDDED-LOOP ***************************************************/
    while (1)
    {
        /* Get period in seconds within loop */
        ui64Current = WTimer0A_getCurrent();
        dPeriodInSec = SYS_GET_SECONDS_FROM_TICKS(ui64Last - ui64Current);
        ui64Last = ui64Current;

        i32Command = REMOTE_receiveByte();

        REMOTE_process(i32Command);

#if(ALTITUDE_BMP280_ENABLE == 1)
        BMP280_getPressInMBar(&tBMP280PressInMBar, &tPressRawData);
        BMP280_getAltitudeInM(&tActualAltitude, &tBMP280PressInMBar);
#endif

#if(ORIENTATION_MPU9250_ENABLE == 1)
        while (MPU_readRawData(&tMPU9AxisRawData, &tPressRawData) == DATA_BUSY)
        {

        }

        MPU9250_get9AxisInUnits(&tMPU9AxisInUnits1, &tMPU9AxisRawData);
        MPU9250_get9AxisInUnitsModified(&tMPU9AxisInUnits2, &tMPU9AxisInUnits1);
#endif

#if(FILTERING_MODE == 1)
        SensorFusion_MahonyUpdate(&tQuaternionMahony,
                (SensorFusion9Axis_t *) &tMPU9AxisInUnits2,
                tPIgain, dPeriodInSec);
        SensorFusion_getYawPitchRoll(&tActualYPRMahony, &tQuaternionMahony);
        tActualYPR = tActualYPRMahony;
#elif(FILTERING_MODE == 2)
        SensorFusion_MadgwickUpdate(&tQuaternionMadgwick,
                (SensorFusion9Axis_t *) &tMPU9AxisInUnits2,
                dGyroMeasError, dPeriodInSec);
        SensorFusion_getYawPitchRoll(&tActualYPRMadgwick, &tQuaternionMadgwick);
        tActualYPR = tActualYPRMadgwick;
#elif(FILTERING_MODE == 3)
        SensorFusion_MahonyUpdate(&tQuaternionMahony,
                                  (SensorFusion9Axis_t *) &tMPU9AxisInUnits2,
                                  tPIgain, dPeriodInSec);
        SensorFusion_MadgwickUpdate(&tQuaternionMadgwick,
                                    (SensorFusion9Axis_t *) &tMPU9AxisInUnits2,
                                    dGyroMeasError, dPeriodInSec);
        SensorFusion_getYawPitchRoll(&tActualYPRMahony, &tQuaternionMahony);
        SensorFusion_getYawPitchRoll(&tActualYPRMadgwick, &tQuaternionMadgwick);
        tActualYPR.dYaw = (tActualYPRMahony.dYaw + tActualYPRMadgwick.dYaw) / 2;
        tActualYPR.dPitch =
                (tActualYPRMahony.dPitch + tActualYPRMadgwick.dPitch) / 2;
        tActualYPR.dRoll = (tActualYPRMahony.dRoll + tActualYPRMadgwick.dRoll)
                / 2;
#else
#error("Please Set a Correct Filtering Mode of [1 or 2 or 3]");
#endif

        /************************************* OPERATION-SECTION *******************************************************/
        if (g_ui32OperationFlag == 1)
        {
            PID_updateInput(&tActualYPR, &tActualAltitude);

            PID_updateOutput(dPeriodInSec);

            QUAD_updateOutput();
        }
        /************************************* END OF OPERATION-SECTION ************************************************/

        /************************************* DEBUG-MSG-SECTION *******************************************************/
        ui32WaitLoops += 1;
        if (ui32WaitLoops == DEBUG_MAIN_WAIT_LOOPS)
        {
            ui32WaitLoops = 0;

            /* Heart beat GREEN_LED */
            LED_toggleGREEN();

            /* Print Angle Values */
#if(DEBUG_ENABLE == 1)
            DEBUG_print(
                    "ROLL: %d | PITCH: %d | YAW: %d | ALTITUDE: %u m | PERIOD: %u ms \n\n",
                    lrintf(tActualYPR.dRoll), lrintf(tActualYPR.dPitch),
                    lrintf(tActualYPR.dYaw), lrintf(tActualAltitude.dAltitude),
                    lrintf(dPeriodInSec * 1000));
#endif
        }
        /************************************* END OF DEBUG-MSG-SECTION ************************************************/
    }
    /************************************* END OF EMBEDDED-LOOP ********************************************************/
}
/************************************* END OF MAIN-PROGRAM *************************************************************/

/*
 *
 * Initialize Tiva MCU
 *
 */
void Tiva_init(void)
{
    SysCtlClockSet(
    SYSCTL_SYSDIV_8 | SYSCTL_XTAL_16MHZ | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN); /* 25 MHz */

    FPULazyStackingEnable();
    FPUEnable();
}

/*
 *
 * Initialize LEDs
 *
 */
void LEDs_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
    GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                          GPIO_PIN_TYPE_STD_WPD);
}

/*
 *
 * Initialize WideTimer 64-bit
 *
 */
void WTimer0A_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0))
    {
    }
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_PERIODIC); // full-width periodic with timeout interrupt for timer3A & in disabled state
    TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_SYSTEM); // set the input clock to the system clock
    TimerControlStall(WTIMER0_BASE, TIMER_A, false); // continue counting while in debug mode
    TimerLoadSet64(WTIMER0_BASE, UINT64_MAX); // 1 tick = 12.5 ns // so // 80000000 ticks = 1 sec
    TimerEnable(WTIMER0_BASE, TIMER_A);
}

void EEPROM_initReadWrite(IntGains_t * ptIntGains, FloatGains_t * ptFloatGains)
{
    uint32_t ui32EEPROMflag;

    if (EEPROM_init() != EEPROM_SUCCESS)
    {
        LED_setRED();

        DEBUG_print("*** ERROR *** [EEPROM INIT failed] at line: %u. \n\n",
        __LINE__);
        while (1)
            ;
    }

    if (EEPROM_isErased(EEPROM_FLAG_ADDRESS,
                        (sizeof(FloatGains_t) + sizeof(uint32_t))) == true)
    {
        ui32EEPROMflag = EEPROM_FLAG_VAL;
        if (EEPROM_writeData(EEPROM_FLAG_ADDRESS, &ui32EEPROMflag,
                             sizeof(ui32EEPROMflag)) != EEPROM_SUCCESS)
        {
            LED_setRED();

            DEBUG_print("*** ERROR *** [EEPROM WRITE failed] at line: %u. \n\n",
            __LINE__);
            while (1)
                ;
        }

        GAIN_FloatToInt(ptIntGains, ptFloatGains);
        if (EEPROM_writeData(EEPROM_DATA_ADDRESS, ptIntGains,
                             sizeof(*ptIntGains)) != EEPROM_SUCCESS)
        {
            LED_setRED();

            DEBUG_print("*** ERROR *** [EEPROM WRITE failed] at line: %u. \n\n",
            __LINE__);
            while (1)
                ;
        }
    }
    else
    {
        if (EEPROM_readData(EEPROM_FLAG_ADDRESS, &ui32EEPROMflag,
                            sizeof(ui32EEPROMflag)) != EEPROM_SUCCESS)
        {
            LED_setRED();

            DEBUG_print("*** ERROR *** [EEPROM READ failed] at line: %u. \n\n",
            __LINE__);
            while (1)
                ;
        }

        if (ui32EEPROMflag == EEPROM_FLAG_VAL)
        {
            if (EEPROM_readData(EEPROM_DATA_ADDRESS, ptIntGains,
                                sizeof(*ptIntGains)) != EEPROM_SUCCESS)
            {
                LED_setRED();

                DEBUG_print(
                        "*** ERROR *** [EEPROM READ failed] at line: %u. \n\n",
                        __LINE__);
                while (1)
                    ;
            }

            GAIN_IntToFloat(ptFloatGains, ptIntGains);
        }
        else
        {
            ui32EEPROMflag = EEPROM_FLAG_VAL;
            if (EEPROM_writeData(EEPROM_FLAG_ADDRESS, &ui32EEPROMflag,
                                 sizeof(ui32EEPROMflag)) != EEPROM_SUCCESS)
            {
                LED_setRED();

                DEBUG_print(
                        "*** ERROR *** [EEPROM WRITE failed] at line: %u. \n\n",
                        __LINE__);
                while (1)
                    ;
            }

            GAIN_FloatToInt(ptIntGains, ptFloatGains);
            if (EEPROM_writeData(EEPROM_DATA_ADDRESS, ptIntGains,
                                 sizeof(*ptIntGains)) != EEPROM_SUCCESS)
            {
                LED_setRED();

                DEBUG_print(
                        "*** ERROR *** [EEPROM WRITE failed] at line: %u. \n\n",
                        __LINE__);
                while (1)
                    ;
            }
        }
    }
}

/*
 *
 * Initialize Motors
 *
 */
void MOTOR_init(void)
{
    if (MOTOR_PWM_init() != MOTOR_SUCCESS)
    {
        LED_setRED();

        DEBUG_print(
                "*** ERROR *** [Setting PWM frequency failed] at line: %u. \n\n",
                __LINE__);

        while (1)
            ;
    }

    /* Set QuadPWM to 2000 us */
    MOTOR_setSpeedFULL();

    delayMillis(4000);

    MOTOR_setSpeedOFF();
}

/*
 *
 * Initialize MPU Sensors
 *
 */
void MPU_init(void)
{
    float adDeviations[6];
    uint8_t ui8ID, ui8DevicesNum = 0;
    MPUCalibParam_t tMPUCalibParam;

    ui8DevicesNum = MPU_I2C_initScan();

#if(DEBUG_ENABLE == 1)
    DEBUG_print("MPU Start with %u devices. \n\n", ui8DevicesNum);
    delayMillis(1000);
#endif

    if (MPU9250_getID(&ui8ID) != MPU_SUCCESS)
    {
        LED_setRED();

        DEBUG_print("*** ERROR *** [MPU9250 disconnected] at line: %u. \n\n",
        __LINE__);
        while (1)
            ;
    }

#if(DEBUG_ENABLE == 1)
    DEBUG_print("[MPU9250 connected with ID: 0x%02X]. \n\n", ui8ID);
    delayMillis(1000);
#endif

    if (MPU9250_SelfTest(adDeviations) != MPU_SUCCESS)
    {
        LED_setRED();

        DEBUG_print("*** ERROR *** [MPU9250 SelfTest failed] at line: %u. \n\n",
        __LINE__);
        while (1)
            ;
    }

#if(DEBUG_ENABLE == 1)
    DEBUG_print("[MPU9250 SelfTest done]. \n\n");
    delayMillis(1000);
#endif

#if (CALIBRATION_ENABLE == 1)
    if (MPU9250_calibrateAccelGyro() != MPU_SUCCESS)
    {
        LED_setRED();

        DEBUG_print(
                "*** ERROR *** [MPU9250 Calibration failed] at line: %u. \n\n",
                __LINE__);
        while (1)
        ;
    }

#if(DEBUG_ENABLE == 1)
    DEBUG_print("[MPU9250 Calibration done]. \n\n");
    delayMillis(1000);
#endif
#endif

    if (MPU9250_init() != MPU_SUCCESS)
    {
        LED_setRED();

        DEBUG_print(
                "*** ERROR *** [MPU9250 Initialization failed] at line: %u. \n\n",
                __LINE__);
        while (1)
            ;
    }

#if(DEBUG_ENABLE == 1)
    DEBUG_print("[MPU9250 Initialization done]. \n\n");
    delayMillis(1000);
#endif

    if (AK8963_getID(&ui8ID) != MPU_SUCCESS)
    {
        LED_setRED();

        DEBUG_print("*** ERROR *** [AK8963 disconnected] at line: %u. \n\n",
        __LINE__);
        while (1)
            ;
    }

#if(DEBUG_ENABLE == 1)
    DEBUG_print("[AK8963 connected with ID: 0x%02X]. \n\n", ui8ID);
    delayMillis(1000);
#endif

    if (AK8963_init() != MPU_SUCCESS)
    {
        LED_setRED();

        DEBUG_print(
                "*** ERROR *** [AK8963 Initialization failed] at line: %u. \n\n",
                __LINE__);
        while (1)
            ;
    }

#if(DEBUG_ENABLE == 1)
    DEBUG_print("[AK8963 Initialization done]. \n\n");
    delayMillis(1000);
#endif

#if (CALIBRATION_ENABLE == 1)
    LED_setBLUE();
#if(DEBUG_ENABLE == 1)
    DEBUG_print("[AK8963 Magnetometer Started (Move it in 8 Shape) ...]. \n\n");
#endif
    if (AK8963_calibrateMag() != MPU_SUCCESS)
    {
        LED_setRED();

        DEBUG_print(
                "*** ERROR *** [AK8963 Calibration failed] at line: %u. \n\n",
                __LINE__);
        while (1)
        ;
    }
    LED_setOFF();

#if(DEBUG_ENABLE == 1)
    MPU9250_getCalibParam(&tMPUCalibParam);
    DEBUG_print("[AK8963 Calibration done]. \n\n");
    delayMillis(1000);
#endif
#endif

#if(ALTITUDE_BMP280_ENABLE == 1)
    if (BMP280_getID(&ui8ID) != MPU_SUCCESS)
    {
        LED_setRED();

        DEBUG_print("*** ERROR *** [BMP280 disconnected] at line: %u. \n\n",
                __LINE__);
        while (1)
        ;
    }

#if(DEBUG_ENABLE == 1)
    DEBUG_print("[BMP280 connected with ID: 0x%02X]. \n\n", ui8ID);
    delayMillis(1000);
#endif

    if (BMP280_init() != MPU_SUCCESS)
    {
        LED_setRED();

        DEBUG_print(
                "*** ERROR *** [BMP280 Initialization failed] at line: %u. \n\n",
                __LINE__);
        while (1)
        ;
    }

#if(DEBUG_ENABLE == 1)
    DEBUG_print("[BMP280 Initialization done]. \n\n");
    delayMillis(1000);
#endif
#endif
}

/*
 *
 * Turn OFF LEDs
 *
 */
void LED_setOFF(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
}

/*
 *
 * Turn ON RED_LED
 *
 */
void LED_setRED(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
}

/*
 *
 * Turn ON BLUE_LED
 *
 */
void LED_setBLUE(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
}

/*
 *
 * Turn ON GREEN_LED
 *
 */
void LED_setGREEN(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

/*
 *
 * Toggle GREEN_LED
 *
 */
void LED_toggleGREEN(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3,
                 ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3));
}

/*
 *
 * Read WideTimer Value
 *
 */
uint64_t WTimer0A_getCurrent(void)
{
    return TimerValueGet64(WTIMER0_BASE);
}

/*
 *
 * Read MPU Sensors RawData
 *
 */
DataStatus_t MPU_readRawData(MPU9AxisRawData_t * const ptMPU9AxisRawData,
                             BMP280Press_t * const ptBMP280Press)
{
    DataStatus_t tMPUDataStatus = DATA_BUSY;

    if (MPU9250_isDataReady() == DATA_READY)
    {
        if (MPU9250_readAccelGyroRawData(ptMPU9AxisRawData) != MPU_SUCCESS)
        {
            LED_setRED();

            DEBUG_print(
                    "*** ERROR *** [MPU9250 reading failed] at line: %u. \n\n",
                    __LINE__);

            while (1)
                ;
        }

        if (AK8963_isDataReady() == DATA_READY)
        {
            if (AK8963_readMagRawData(ptMPU9AxisRawData) != MPU_SUCCESS)
            {
                LED_setRED();

                DEBUG_print(
                        "*** ERROR *** [AK8963 reading failed] at line: %u. \n\n",
                        __LINE__);

                while (1)
                    ;
            }

            tMPUDataStatus = DATA_READY;
        }

#if(ALTITUDE_BMP280_ENABLE == 1)
        if (BMP280_readPressRawData(ptBMP280Press) != MPU_SUCCESS)
        {
            LED_setRED();

            DEBUG_print("*** ERROR *** [BMP280 reading failed] at line: %u. \n\n",
                    __LINE__);

            while (1)
            ;
        }
#endif
    }

    return tMPUDataStatus;
}

/*
 *
 * Convert FLoat gains to Integer by Multiplying * 1000
 *
 */
void GAIN_FloatToInt(IntGains_t *ptIntGains, FloatGains_t *ptFloatGains)
{
    uint32_t i;

    for (i = 0; i < (sizeof(FloatGains_t) / sizeof(float)); i++)
    {
        ((uint32_t *) ptIntGains)[i] = ((float *) ptFloatGains)[i] * 1000U;
    }
}

/*
 *
 * Convert Integer gains to Float by Dividing / 1000
 *
 */
void GAIN_IntToFloat(FloatGains_t *ptFloatGains, IntGains_t *ptIntGains)
{
    uint32_t i;

    for (i = 0; i < (sizeof(IntGains_t) / sizeof(uint32_t)); i++)
    {
        ((float *) ptFloatGains)[i] = ((uint32_t *) ptIntGains)[i] / 1000.0f;
    }
}

/*
 *
 * Process REMOTE Commands
 *
 */
void REMOTE_process(const int32_t i32Command)
{
    static uint32_t s_ui32PIDTuningFlag = 0;
    static PIDobject_t * s_ptPIDobject = 0;

    if (s_ptPIDobject == 0)
    {
        s_ptPIDobject = &g_tPIDobjectROLL;
    }

    switch (i32Command)
    {
    case 0x000000FF:
        if ((g_ui32OperationFlag == 0) && (s_ui32PIDTuningFlag == 0))
        {
            g_ui32OperationFlag = 1;
            MOTOR_setSpeedON();
        }
        break;

    case 0x00000000:
        if ((g_ui32OperationFlag == 1) && (s_ui32PIDTuningFlag == 0))
        {
            g_ui32OperationFlag = 0;
            MOTOR_setSpeedOFF();
        }
        break;

    case '*':
        if ((s_ui32PIDTuningFlag == 0) && (g_ui32OperationFlag == 0))
        {
            if (EEPROM_readData(EEPROM_DATA_ADDRESS, &g_tIntGains,
                                sizeof(g_tFloatGains)) != EEPROM_SUCCESS)
            {
                LED_setRED();

                DEBUG_print(
                        "*** ERROR *** [EEPROM READ failed] at line: %u. \n\n",
                        __LINE__);
                while (1)
                    ;
            }

            REMOTE_transmitData(&g_tIntGains, sizeof(g_tIntGains));

            s_ui32PIDTuningFlag = 1;
            g_ui32OperationFlag = 1;
            MOTOR_setSpeedON();
        }
        break;

    case '/':
        if ((s_ui32PIDTuningFlag == 1) && (g_ui32OperationFlag == 1))
        {
            g_tFloatGains.tPIDgainROLL = g_tPIDobjectROLL.tPIDgain;
            g_tFloatGains.tPIDgainPITCH = g_tPIDobjectPITCH.tPIDgain;
            g_tFloatGains.tPIDgainYAW = g_tPIDobjectYAW.tPIDgain;
            g_tFloatGains.tPIDgainALTITUDE = g_tPIDobjectALTITUDE.tPIDgain;

            GAIN_FloatToInt(&g_tIntGains, &g_tFloatGains);

            if (EEPROM_writeData(EEPROM_DATA_ADDRESS, &g_tIntGains,
                                 sizeof(g_tIntGains)) != EEPROM_SUCCESS)
            {
                LED_setRED();

                DEBUG_print(
                        "*** ERROR *** [EEPROM WRITE failed] at line: %u. \n\n",
                        __LINE__);
                while (1)
                    ;
            }

            s_ui32PIDTuningFlag = 0;
            g_ui32OperationFlag = 0;
            MOTOR_setSpeedOFF();
        }
        break;

    default:

        if (g_ui32OperationFlag == 1)
        {
            switch (i32Command)
            {
            case '+':
                MOTOR_increaseSpeed(MOTOR_DELTA_MICROS); /* Increase Micros */
                break;
            case '-':
                MOTOR_decreaseSpeed(MOTOR_DELTA_MICROS); /* Decrease Micros */
                break;

            case 0x00000001:
                g_tDesiredYPR.dPitch = 20.0f;
                break;
            case 0x00000002:
                g_tDesiredYPR.dPitch = -20.0f;
                break;
            case 0x00000003:
                g_tDesiredYPR.dPitch = 0.0f;
                break;
            case 0x00000004:
                g_tDesiredYPR.dRoll = 20.0f;
                break;
            case 0x00000005:
                g_tDesiredYPR.dRoll = -20.0f;
                break;
            case 0x00000006:
                g_tDesiredYPR.dRoll = 0.0f;
                break;

            case 'w':
                break;
            case 'W':
                break;
            case 's':
                break;
            case 'S':
                break;
            case 'a':
                break;
            case 'A':
                break;
            case 'd':
                break;
            case 'D':
                break;

            default:
                break;
            }
        }

        if (s_ui32PIDTuningFlag == 1)
        {
            switch (i32Command)
            {
            case 'r':
                s_ptPIDobject = &g_tPIDobjectROLL;
                break;
            case 'p':
                s_ptPIDobject = &g_tPIDobjectPITCH;
                break;
            case 'y':
                s_ptPIDobject = &g_tPIDobjectYAW;
                break;
            case 'h':
                s_ptPIDobject = &g_tPIDobjectALTITUDE;
                break;

            case '4':
                s_ptPIDobject->tPIDgain.dKP += 0.1f;
                break;
            case '1':
                s_ptPIDobject->tPIDgain.dKP -= 0.1f;
                if (s_ptPIDobject->tPIDgain.dKP < 0.0f)
                {
                    s_ptPIDobject->tPIDgain.dKP = 0.0f;
                }
                break;
            case '5':
                s_ptPIDobject->tPIDgain.dKI += 0.001f;
                break;
            case '2':
                s_ptPIDobject->tPIDgain.dKI -= 0.001f;
                if (s_ptPIDobject->tPIDgain.dKI < 0.00f)
                {
                    s_ptPIDobject->tPIDgain.dKI = 0.00f;
                }
                break;
            case '6':
                s_ptPIDobject->tPIDgain.dKD += 0.1f;
                break;
            case '3':
                s_ptPIDobject->tPIDgain.dKD -= 0.1f;
                if (s_ptPIDobject->tPIDgain.dKD < 0.0f)
                {
                    s_ptPIDobject->tPIDgain.dKD = 0.0f;
                }
                break;

            default:
                break;
            }

//            PID_reset(s_ptPIDobject);
        }
        break;
    }
}

/*
 *
 * Update PID Inputs
 *
 */
void PID_updateInput(const YawPitchRoll_t * const ptActualYPR,
                     const BMP280AltitudeInM_t * const ptActualAltitude)
{
    PID_setDesiredVal(&g_tPIDobjectROLL, g_tDesiredYPR.dRoll);
    PID_setActualVal(&g_tPIDobjectROLL, ptActualYPR->dRoll);

    PID_setDesiredVal(&g_tPIDobjectPITCH, g_tDesiredYPR.dPitch);
    PID_setActualVal(&g_tPIDobjectPITCH, ptActualYPR->dPitch);

    PID_setDesiredVal(&g_tPIDobjectYAW, g_tDesiredYPR.dYaw);
    PID_setActualVal(&g_tPIDobjectYAW, ptActualYPR->dYaw);

#if(ALTITUDE_BMP280_ENABLE == 1)
    PID_setDesiredVal(&g_tPIDobjectALTITUDE, g_tDesiredAltitude.dAltitude);
    PID_setActualVal(&g_tPIDobjectALTITUDE, ptActualAltitude->dAltitude);
#endif
}

/*
 *
 * Update PID Outputs
 *
 */
void PID_updateOutput(const float dPeriodInSec)
{
    PID_update(&g_tPIDobjectROLL, dPeriodInSec);
    PID_update(&g_tPIDobjectPITCH, dPeriodInSec);
    PID_update(&g_tPIDobjectYAW, dPeriodInSec);

#if(ALTITUDE_BMP280_ENABLE == 1)
    PID_update(&g_tPIDobjectALTITUDE, dPeriodInSec);
#endif
}

/*
 *
 * Update QUAD Outputs
 *
 */
void QUAD_updateOutput(void)
{
    PulseWidth_t tPulseWidth;
    float dPIDOutROLL, dPIDOutPITCH, dPIDOutYAW, dPIDOutALTITUDE;
    int32_t i32DeltaOutInTicksROLL, i32DeltaOutInTicksPITCH,
            i32DeltaOutInTicksYAW, i32DeltaOutInTicksALTITUDE;

    /* Read the PID output */
    dPIDOutROLL = PID_getOut(&g_tPIDobjectROLL);
    dPIDOutPITCH = PID_getOut(&g_tPIDobjectPITCH);
    dPIDOutYAW = PID_getOut(&g_tPIDobjectYAW);
    dPIDOutALTITUDE = PID_getOut(&g_tPIDobjectALTITUDE);

    /* Map the PID output to the delta output in ticks */
    i32DeltaOutInTicksROLL = ((PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MAX_MICROS)
            - PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MIN_MICROS)) * dPIDOutROLL)
            / ROLL_PID_OUT_MAX;

    i32DeltaOutInTicksPITCH = ((PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MAX_MICROS)
            - PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MIN_MICROS)) * dPIDOutPITCH)
            / PITCH_PID_OUT_MAX;

    i32DeltaOutInTicksYAW = ((PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MAX_MICROS)
            - PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MIN_MICROS)) * dPIDOutYAW)
            / YAW_PID_OUT_MAX;

    i32DeltaOutInTicksALTITUDE = ((PWM_GET_TICKS_FROM_MICROS(
            MOTOR_PULSE_MAX_MICROS)
            - PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MIN_MICROS))
            * dPIDOutALTITUDE) / ALTITUDE_PID_OUT_MAX;

    /* Read the current actuator output */
    QuadPWM_getPulses(&tPulseWidth);

    /*Add the delta output to the current actuator output*/
    tPulseWidth.ui32PulseInTicks1 += i32DeltaOutInTicksROLL; /* Stabilize ROLL */

    tPulseWidth.ui32PulseInTicks2 += i32DeltaOutInTicksPITCH; /* Stabilize PITCH */

    tPulseWidth.ui32PulseInTicks1 += i32DeltaOutInTicksYAW; /* Stabilize YAW */
    tPulseWidth.ui32PulseInTicks3 += i32DeltaOutInTicksYAW;

#if(ALTITUDE_BMP280_ENABLE == 1)
    tPulseWidth.ui32PulseInTicks1 += i32DeltaOutInTicksALTITUDE; /* Stabilize ALTITUDE */
    tPulseWidth.ui32PulseInTicks2 += i32DeltaOutInTicksALTITUDE;
    tPulseWidth.ui32PulseInTicks3 += i32DeltaOutInTicksALTITUDE;
    tPulseWidth.ui32PulseInTicks4 += i32DeltaOutInTicksALTITUDE;
#endif

    /* Get value within boundaries [MOTOR_PULSE_MAX_MICROS & MOTOR_PULSE_MIN_MICROS] */
    getValueWithinLimits(&tPulseWidth.ui32PulseInTicks1,
                         PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MAX_MICROS),
                         PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MIN_MICROS));

    getValueWithinLimits(&tPulseWidth.ui32PulseInTicks2,
                         PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MAX_MICROS),
                         PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MIN_MICROS));

    getValueWithinLimits(&tPulseWidth.ui32PulseInTicks3,
                         PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MAX_MICROS),
                         PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MIN_MICROS));

    getValueWithinLimits(&tPulseWidth.ui32PulseInTicks4,
                         PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MAX_MICROS),
                         PWM_GET_TICKS_FROM_MICROS(MOTOR_PULSE_MIN_MICROS));

    /* Update Motors */
    MOTOR_update(&tPulseWidth);
}

