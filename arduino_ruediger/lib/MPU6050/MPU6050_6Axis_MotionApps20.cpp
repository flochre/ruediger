#include "MPU6050_6Axis_MotionApps20.h"

// uint8_t MPU6050::dmpInitialize() {
//     // reset device
//     DEBUG_PRINTLN(F("\n\nResetting MPU6050..."));
//     reset();
//     delay(30); // wait after reset

//     // enable sleep mode and wake cycle
//     /*Serial.println(F("Enabling sleep mode..."));
//     setSleepEnabled(true);
//     Serial.println(F("Enabling wake cycle..."));
//     setWakeCycleEnabled(true);*/

//     // disable sleep mode
//     DEBUG_PRINTLN(F("Disabling sleep mode..."));
//     setSleepEnabled(false);

//     // get MPU hardware revision
//     DEBUG_PRINTLN(F("Selecting user bank 16..."));
//     setMemoryBank(0x10, true, true);
//     DEBUG_PRINTLN(F("Selecting memory byte 6..."));
//     setMemoryStartAddress(0x06);
//     DEBUG_PRINTLN(F("Checking hardware revision..."));
//     uint8_t hwRevision = readMemoryByte();
//     DEBUG_PRINT(F("Revision @ user[16][6] = "));
//     DEBUG_PRINTLNF(hwRevision, HEX);
//     DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
//     setMemoryBank(0, false, false);

//     // check OTP bank valid
//     DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
//     uint8_t otpValid = getOTPBankValid();
//     DEBUG_PRINT(F("OTP bank is "));
//     DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

//     // get X/Y/Z acc offsets
//     DEBUG_PRINTLN(F("Reading acc offset values..."));
//     int8_t xaOffset = getXAccelOffset();
//     int8_t yaOffset = getYAccelOffset();
//     int8_t zaOffset = getZAccelOffset();
//     DEBUG_PRINT(F("X ACC offset = "));
//     DEBUG_PRINTLN(xaOffset);
//     DEBUG_PRINT(F("Y ACC offset = "));
//     DEBUG_PRINTLN(yaOffset);
//     DEBUG_PRINT(F("Z ACC offset = "));
//     DEBUG_PRINTLN(zaOffset);

//     // get X/Y/Z gyro offsets
//     DEBUG_PRINTLN(F("Reading gyro offset values..."));
//     int8_t xgOffset = getXGyroOffset();
//     int8_t ygOffset = getYGyroOffset();
//     int8_t zgOffset = getZGyroOffset();
//     DEBUG_PRINT(F("X gyro offset = "));
//     DEBUG_PRINTLN(xgOffset);
//     DEBUG_PRINT(F("Y gyro offset = "));
//     DEBUG_PRINTLN(ygOffset);
//     DEBUG_PRINT(F("Z gyro offset = "));
//     DEBUG_PRINTLN(zgOffset);

//     // setup weird slave stuff (?)
//     DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
//     setSlaveAddress(0, 0x7F);
//     DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
//     setI2CMasterModeEnabled(false);
//     DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
//     setSlaveAddress(0, 0x68);
//     DEBUG_PRINTLN(F("Resetting I2C Master control..."));
//     resetI2CMaster();
//     delay(20);

//     // load DMP code into memory banks
//     DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
//     DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
//     DEBUG_PRINTLN(F(" bytes)"));
//     if (writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) {
//         DEBUG_PRINTLN(F("Success! DMP code written and verified."));

//         // write DMP configuration
//         DEBUG_PRINT(F("Writing DMP configuration to MPU memory banks ("));
//         DEBUG_PRINT(MPU6050_DMP_CONFIG_SIZE);
//         DEBUG_PRINTLN(F(" bytes in config def)"));
//         if (writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
//             DEBUG_PRINTLN(F("Success! DMP configuration written and verified."));

//             DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
//             setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

//             DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
//             setIntEnabled(0x12);

//             // DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
//             // setRate(4); // 1khz / (1 + 4) = 200 Hz
//             DEBUG_PRINTLN(F("Setting sample rate to 20Hz..."));
//             setRate(49); // 1khz / (1 + 49) = 20 Hz

//             DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
//             setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

//             DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
//             setDLPFMode(MPU6050_DLPF_BW_42);

//             // DEBUG_PRINTLN(F("Setting Acc sensitivity to +/- 2g..."));
//             // setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

//             DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
//             setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
//             // DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 500 deg/sec..."));
//             // setFullScaleGyroRange(MPU6050_GYRO_FS_500);

//             DEBUG_PRINTLN(F("Setting DMP configuration bytes (function unknown)..."));
//             setDMPConfig1(0x03);
//             setDMPConfig2(0x00);

//             DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
//             setOTPBankValid(false);

//             DEBUG_PRINTLN(F("Setting X/Y/Z Accel offsets to previous values..."));
//             setXAccelOffset(xaOffset);
//             setYAccelOffset(yaOffset);
//             setZAccelOffset(zaOffset);


//             DEBUG_PRINTLN(F("Setting X/Y/Z gyro offsets to previous values..."));
//             setXGyroOffset(xgOffset);
//             setYGyroOffset(ygOffset);
//             setZGyroOffset(zgOffset);

//             DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
//             setXGyroOffsetUser(0);
//             setYGyroOffsetUser(0);
//             setZGyroOffsetUser(0);

//             // DO small calibration
//             uint16_t x = 0;
//             uint16_t num = 1000;
//             long aSum = 0, bSum = 0, cSum = 0;
//             long xSum = 0, ySum = 0, zSum = 0;
//             for(x = 0; x < num; x++)
//             {
//                 // return_value = readData(0x43, i2cData, 6);
//                 I2Cdev::readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
//                 aSum += ( (buffer[0] << 8) | buffer[1] );
//                 bSum += ( (buffer[2] << 8) | buffer[3] );
//                 cSum += ( (buffer[4] << 8) | buffer[5] );
//                 I2Cdev::readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
//                 xSum += ( (buffer[0] << 8) | buffer[1] );
//                 ySum += ( (buffer[2] << 8) | buffer[3] );
//                 zSum += ( (buffer[4] << 8) | buffer[5] );
//                 delay(3);
//             }
            
//             DEBUG_PRINT(F("X acc user offset = "));
//             DEBUG_PRINTLN(aSum  / num);
//             DEBUG_PRINT(F("Y acc user offset = "));
//             DEBUG_PRINTLN(bSum  / num);
//             DEBUG_PRINT(F("Z acc user offset = "));
//             DEBUG_PRINTLN(cSum  / num);

//             // DEBUG_PRINTLN(F("Setting X/Y/Z Accel offsets to new values..."));
//             // setXAccelOffset(aSum  / num);
//             // setYAccelOffset(bSum  / num);
//             // setZAccelOffset(cSum  / num);

//             DEBUG_PRINT(F("X gyro user offset = "));
//             DEBUG_PRINTLN(xSum  / num);
//             DEBUG_PRINT(F("Y gyro user offset = "));
//             DEBUG_PRINTLN(ySum  / num);
//             DEBUG_PRINT(F("Z gyro user offset = "));
//             DEBUG_PRINTLN(zSum  / num);

//             setXGyroOffsetUser(xSum  / num);
//             setYGyroOffsetUser(ySum  / num);
//             setZGyroOffsetUser(zSum  / num);

//             DEBUG_PRINTLN(F("Writing final memory update 1/7 (function unknown)..."));
//             uint8_t dmpUpdate[16], j;
//             uint16_t pos = 0;
//             for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
//             writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//             DEBUG_PRINTLN(F("Writing final memory update 2/7 (function unknown)..."));
//             for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
//             writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//             DEBUG_PRINTLN(F("Resetting FIFO..."));
//             resetFIFO();

//             DEBUG_PRINTLN(F("Reading FIFO count..."));
//             uint8_t fifoCount = getFIFOCount();
//             uint8_t fifoBuffer[128];

//             DEBUG_PRINT(F("Current FIFO count="));
//             DEBUG_PRINTLN(fifoCount);
//             getFIFOBytes(fifoBuffer, fifoCount);

//             DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
//             setMotionDetectionThreshold(2);

//             DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
//             setZeroMotionDetectionThreshold(156);

//             DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
//             setMotionDetectionDuration(80);

//             DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
//             setZeroMotionDetectionDuration(0);

//             DEBUG_PRINTLN(F("Resetting FIFO..."));
//             resetFIFO();

//             DEBUG_PRINTLN(F("Enabling FIFO..."));
//             setFIFOEnabled(true);

//             DEBUG_PRINTLN(F("Enabling DMP..."));
//             setDMPEnabled(true);

//             DEBUG_PRINTLN(F("Resetting DMP..."));
//             resetDMP();

//             DEBUG_PRINTLN(F("Writing final memory update 3/7 (function unknown)..."));
//             for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
//             writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//             DEBUG_PRINTLN(F("Writing final memory update 4/7 (function unknown)..."));
//             for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
//             writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//             DEBUG_PRINTLN(F("Writing final memory update 5/7 (function unknown)..."));
//             for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
//             writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//             DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
//             while ((fifoCount = getFIFOCount()) < 3);

//             DEBUG_PRINT(F("Current FIFO count="));
//             DEBUG_PRINTLN(fifoCount);
//             DEBUG_PRINTLN(F("Reading FIFO data..."));
//             getFIFOBytes(fifoBuffer, fifoCount);

//             DEBUG_PRINTLN(F("Reading interrupt status..."));
//             uint8_t mpuIntStatus = getIntStatus();

//             DEBUG_PRINT(F("Current interrupt status="));
//             DEBUG_PRINTLNF(mpuIntStatus, HEX);

//             DEBUG_PRINTLN(F("Reading final memory update 6/7 (function unknown)..."));
//             for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
//             readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//             DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
//             while ((fifoCount = getFIFOCount()) < 3);

//             DEBUG_PRINT(F("Current FIFO count="));
//             DEBUG_PRINTLN(fifoCount);

//             DEBUG_PRINTLN(F("Reading FIFO data..."));
//             getFIFOBytes(fifoBuffer, fifoCount);

//             DEBUG_PRINTLN(F("Reading interrupt status..."));
//             mpuIntStatus = getIntStatus();

//             DEBUG_PRINT(F("Current interrupt status="));
//             DEBUG_PRINTLNF(mpuIntStatus, HEX);

//             DEBUG_PRINTLN(F("Writing final memory update 7/7 (function unknown)..."));
//             for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
//             writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//             DEBUG_PRINTLN(F("DMP is good to go! Finally."));

//             DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
//             setDMPEnabled(false);

//             DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
//             dmpPacketSize = 42;
//             /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
//                 return 3; // TODO: proper error code for no memory
//             }*/

//             DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
//             resetFIFO();
//             getIntStatus();
//         } else {
//             DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
//             return 2; // configuration block loading failed
//         }
//     } else {
//         DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
//         return 1; // main binary block loading failed
//     }
//     return 0; // success
// }

uint8_t MPU6050::dmpInitialize() {
    // reset device
    DEBUG_PRINTLN(F("\n\nResetting MPU6050..."));
    reset();
    delay(30); // wait after reset

    // enable sleep mode and wake cycle
    /*Serial.println(F("Enabling sleep mode..."));
    setSleepEnabled(true);
    Serial.println(F("Enabling wake cycle..."));
    setWakeCycleEnabled(true);*/

    // disable sleep mode
    DEBUG_PRINTLN(F("Disabling sleep mode..."));
    setSleepEnabled(false);

    // get MPU hardware revision
    DEBUG_PRINTLN(F("Selecting user bank 16..."));
    setMemoryBank(0x10, true, true);
    DEBUG_PRINTLN(F("Selecting memory byte 6..."));
    setMemoryStartAddress(0x06);
    DEBUG_PRINTLN(F("Checking hardware revision..."));
    uint8_t hwRevision = readMemoryByte();
    DEBUG_PRINT(F("Revision @ user[16][6] = "));
    DEBUG_PRINTLNF(hwRevision, HEX);
    DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
    setMemoryBank(0, false, false);

    // check OTP bank valid
    DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
    uint8_t otpValid = getOTPBankValid();
    DEBUG_PRINT(F("OTP bank is "));
    DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

    // get X/Y/Z gyro offsets
    DEBUG_PRINTLN(F("Reading gyro offset values..."));
    int8_t xgOffset = getXGyroOffset();
    int8_t ygOffset = getYGyroOffset();
    int8_t zgOffset = getZGyroOffset();
    DEBUG_PRINT(F("X gyro offset = "));
    DEBUG_PRINTLN(xgOffset);
    DEBUG_PRINT(F("Y gyro offset = "));
    DEBUG_PRINTLN(ygOffset);
    DEBUG_PRINT(F("Z gyro offset = "));
    DEBUG_PRINTLN(zgOffset);

    // setup weird slave stuff (?)
    DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
    setSlaveAddress(0, 0x7F);
    DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
    setI2CMasterModeEnabled(false);
    DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
    setSlaveAddress(0, 0x68);
    DEBUG_PRINTLN(F("Resetting I2C Master control..."));
    resetI2CMaster();
    delay(20);

    // load DMP code into memory banks
    DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
    DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
    DEBUG_PRINTLN(F(" bytes)"));
    if (writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) {
        DEBUG_PRINTLN(F("Success! DMP code written and verified."));

        // write DMP configuration
        DEBUG_PRINT(F("Writing DMP configuration to MPU memory banks ("));
        DEBUG_PRINT(MPU6050_DMP_CONFIG_SIZE);
        DEBUG_PRINTLN(F(" bytes in config def)"));
        if (writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
            DEBUG_PRINTLN(F("Success! DMP configuration written and verified."));

            DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
            setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

            DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
            setIntEnabled(0x12);

            // DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
            // setRate(4); // 1khz / (1 + 4) = 200 Hz

            DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
            setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

            DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
            setDLPFMode(MPU6050_DLPF_BW_42);

            DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
            setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

            DEBUG_PRINTLN(F("Setting DMP configuration bytes (function unknown)..."));
            setDMPConfig1(0x03);
            setDMPConfig2(0x00);

            DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
            setOTPBankValid(false);

            DEBUG_PRINTLN(F("Setting X/Y/Z gyro offsets to previous values..."));
            setXGyroOffset(xgOffset);
            setYGyroOffset(ygOffset);
            setZGyroOffset(zgOffset);

            DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
            setXGyroOffsetUser(0);
            setYGyroOffsetUser(0);
            setZGyroOffsetUser(0);

            DEBUG_PRINTLN(F("Writing final memory update 1/7 (function unknown)..."));
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Writing final memory update 2/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Resetting FIFO..."));
            resetFIFO();

            DEBUG_PRINTLN(F("Reading FIFO count..."));
            uint8_t fifoCount = getFIFOCount();
            uint8_t fifoBuffer[128];

            DEBUG_PRINT(F("Current FIFO count="));
            DEBUG_PRINTLN(fifoCount);
            getFIFOBytes(fifoBuffer, fifoCount);

            DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
            setMotionDetectionThreshold(2);

            DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
            setZeroMotionDetectionThreshold(156);

            DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
            setMotionDetectionDuration(80);

            DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
            setZeroMotionDetectionDuration(0);

            DEBUG_PRINTLN(F("Resetting FIFO..."));
            resetFIFO();

            DEBUG_PRINTLN(F("Enabling FIFO..."));
            setFIFOEnabled(true);

            DEBUG_PRINTLN(F("Enabling DMP..."));
            setDMPEnabled(true);

            DEBUG_PRINTLN(F("Resetting DMP..."));
            resetDMP();

            DEBUG_PRINTLN(F("Writing final memory update 3/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Writing final memory update 4/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Writing final memory update 5/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
            while ((fifoCount = getFIFOCount()) < 3);

            DEBUG_PRINT(F("Current FIFO count="));
            DEBUG_PRINTLN(fifoCount);
            DEBUG_PRINTLN(F("Reading FIFO data..."));
            getFIFOBytes(fifoBuffer, fifoCount);

            DEBUG_PRINTLN(F("Reading interrupt status..."));
            uint8_t mpuIntStatus = getIntStatus();

            DEBUG_PRINT(F("Current interrupt status="));
            DEBUG_PRINTLNF(mpuIntStatus, HEX);

            DEBUG_PRINTLN(F("Reading final memory update 6/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
            while ((fifoCount = getFIFOCount()) < 3);

            DEBUG_PRINT(F("Current FIFO count="));
            DEBUG_PRINTLN(fifoCount);

            DEBUG_PRINTLN(F("Reading FIFO data..."));
            getFIFOBytes(fifoBuffer, fifoCount);

            DEBUG_PRINTLN(F("Reading interrupt status..."));
            mpuIntStatus = getIntStatus();

            DEBUG_PRINT(F("Current interrupt status="));
            DEBUG_PRINTLNF(mpuIntStatus, HEX);

            DEBUG_PRINTLN(F("Writing final memory update 7/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("DMP is good to go! Finally."));

            DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
            setDMPEnabled(false);

            DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
            dmpPacketSize = 42;
            /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
                return 3; // TODO: proper error code for no memory
            }*/

            DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
            resetFIFO();
            getIntStatus();
        } else {
            DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
            return 2; // configuration block loading failed
        }
    } else {
        DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
        return 1; // main binary block loading failed
    }
    return 0; // success
}

bool MPU6050::dmpPacketAvailable() {
    return getFIFOCount() >= dmpGetFIFOPacketSize();
}

// uint8_t MPU6050::dmpSetFIFORate(uint8_t fifoRate);
// uint8_t MPU6050::dmpGetFIFORate();
// uint8_t MPU6050::dmpGetSampleStepSizeMS();
// uint8_t MPU6050::dmpGetSampleFrequency();
// int32_t MPU6050::dmpDecodeTemperature(int8_t tempReg);

//uint8_t MPU6050::dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t MPU6050::dmpUnregisterFIFORateProcess(inv_obj_func func);
//uint8_t MPU6050::dmpRunFIFORateProcesses();

// uint8_t MPU6050::dmpSendQuaternion(uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendPacketNumber(uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

uint8_t MPU6050::dmpGetAccel(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[28] << 24) + (packet[29] << 16) + (packet[30] << 8) + packet[31]);
    data[1] = ((packet[32] << 24) + (packet[33] << 16) + (packet[34] << 8) + packet[35]);
    data[2] = ((packet[36] << 24) + (packet[37] << 16) + (packet[38] << 8) + packet[39]);
    return 0;
}
uint8_t MPU6050::dmpGetAccel(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[28] << 8) + packet[29];
    data[1] = (packet[32] << 8) + packet[33];
    data[2] = (packet[36] << 8) + packet[37];
    return 0;
}
uint8_t MPU6050::dmpGetAccel(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[28] << 8) + packet[29];
    v -> y = (packet[32] << 8) + packet[33];
    v -> z = (packet[36] << 8) + packet[37];
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 24) + (packet[1] << 16) + (packet[2] << 8) + packet[3]);
    data[1] = ((packet[4] << 24) + (packet[5] << 16) + (packet[6] << 8) + packet[7]);
    data[2] = ((packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8) + packet[11]);
    data[3] = ((packet[12] << 24) + (packet[13] << 16) + (packet[14] << 8) + packet[15]);
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) + packet[1]);
    data[1] = ((packet[4] << 8) + packet[5]);
    data[2] = ((packet[8] << 8) + packet[9]);
    data[3] = ((packet[12] << 8) + packet[13]);
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}
// uint8_t MPU6050::dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[16] << 24) + (packet[17] << 16) + (packet[18] << 8) + packet[19]);
    data[1] = ((packet[20] << 24) + (packet[21] << 16) + (packet[22] << 8) + packet[23]);
    data[2] = ((packet[24] << 24) + (packet[25] << 16) + (packet[26] << 8) + packet[27]);
    return 0;
}
uint8_t MPU6050::dmpGetGyro(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[16] << 8) + packet[17];
    data[1] = (packet[20] << 8) + packet[21];
    data[2] = (packet[24] << 8) + packet[25];
    return 0;
}
uint8_t MPU6050::dmpGetGyro(float *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (float) ((packet[16] << 8) + packet[17]) / 131.0;
    data[1] = (float) ((packet[20] << 8) + packet[21]) / 131.0;
    data[2] = (float) ((packet[24] << 8) + packet[25]) / 131.0;
    return 0;
}
// uint8_t MPU6050::dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t MPU6050::dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+2g = +16384 in standard DMP FIFO packet)
    // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
    v -> x = vRaw -> x - gravity -> x*16384;
    v -> y = vRaw -> y - gravity -> y*16384;
    v -> z = vRaw -> z - gravity -> z*16384;
    return 0;
}
uint8_t MPU6050::dmpGetLinearAccel(VectorFloat *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+2g = +16384 in standard DMP FIFO packet)
    // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
    v -> x = vRaw -> x / 16384.0 * 9.8 - gravity->x * 9.8;
    v -> y = vRaw -> y / 16384.0 * 9.8 - gravity->y * 9.8;
    v -> z = vRaw -> z / 16384.0 * 9.8 - gravity->z * 9.8;
    return 0;
}
// uint8_t MPU6050::dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}
// uint8_t MPU6050::dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetGravity(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}
// uint8_t MPU6050::dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t MPU6050::dmpGetEIS(long *data, const uint8_t* packet);

uint8_t MPU6050::dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    // data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    // data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    data[2] = atan2(gravity -> y , gravity -> z);

    if (gravity -> z < 0) {
		if(data[1] > 0) {
			data[1] = PI - data[1];
        } else {
			data[1] = -PI - data[1];
		}
	}

    return 0;
}

// uint8_t MPU6050::dmpGetAccelFloat(float *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t MPU6050::dmpProcessFIFOPacket(const unsigned char *dmpData) {
    /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
        if (dmpData[k] < 0x10) Serial.print("0");
        Serial.print(dmpData[k], HEX);
        Serial.print(" ");
    }
    Serial.print("\n");*/
    //Serial.println((uint16_t)dmpPacketBuffer);
    return 0;
}
uint8_t MPU6050::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        getFIFOBytes(buf, dmpPacketSize);

        // process packet
        if ((status = dmpProcessFIFOPacket(buf)) > 0) return status;

        // increment external process count variable, if supplied
        if (processed != 0) *processed++;
    }
    return 0;
}

// uint8_t MPU6050::dmpSetFIFOProcessedCallback(void (*func) (void));

// uint8_t MPU6050::dmpInitFIFOParam();
// uint8_t MPU6050::dmpCloseFIFO();
// uint8_t MPU6050::dmpSetGyroDataSource(uint_fast8_t source);
// uint8_t MPU6050::dmpDecodeQuantizedAccel();
// uint32_t MPU6050::dmpGetGyroSumOfSquare();
// uint32_t MPU6050::dmpGetAccelSumOfSquare();
// void MPU6050::dmpOverrideQuaternion(long *q);
uint16_t MPU6050::dmpGetFIFOPacketSize() {
    return dmpPacketSize;
}