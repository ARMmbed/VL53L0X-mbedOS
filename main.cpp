#include "mbed.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

#define USE_I2C_2V8

Serial pc(USBTX, USBRX);
DigitalOut led(LED1);


VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;
    
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return Status;
}



int main()
{
    int x=1, measure=0;
    int ave=0, sum=0;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData    = &RangingMeasurementData;
    VL53L0X_Version_t                   Version;
    
    // Initialize Comms
    pMyDevice->I2cDevAddr      = 0x52;
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  400;
    
    
    VL53L0X_ERROR_CONTROL_INTERFACE;
    VL53L0X_RdWord(&MyDevice, VL53L0X_REG_OSC_CALIBRATE_VAL,0);
    VL53L0X_DataInit(&MyDevice); // Data initialization
    Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    
    VL53L0X_StaticInit(pMyDevice); // Device Initialization
    VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads); // Device Initialization
    VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal); // Device Initialization
    VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
    VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536)); //High Accuracy mode, see API PDF
    VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18*65536)); //High Accuracy mode, see API PDF
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 200000); //High Accuracy mode, see API PDF
    VL53L0X_StartMeasurement(pMyDevice);

    while (1){
        while(x<=50){
        WaitMeasurementDataReady(pMyDevice);
        VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);
        measure=pRangingMeasurementData->RangeMilliMeter;
        //printf("In loop measurement %d\n", mea);
        sum=sum+measure;
        led=!led;
        // Clear the interrupt
        VL53L0X_ClearInterruptMask(pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        VL53L0X_PollingDelay(pMyDevice);
        x++;
        }
        ave=sum/x;
        printf("Average is %d\n", ave);
        x=1;
        sum=0;
        wait(20);
    }

    VL53L0X_StopMeasurement(pMyDevice);
    WaitStopCompleted(pMyDevice);
    VL53L0X_ClearInterruptMask(pMyDevice,VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    
}
 
