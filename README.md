# STM32-DFR0971-GP8403-M5DAC-Fix
This issue was solved with help of AI.  

Simple STM32 HAL driver for I2C DAC modules (DFR0971, M5Stack DAC2). Fixes the issue where the output voltage is half of the expected value (e.g., 5V instead of 10V).
The Problem:
According to the official datasheets, the GP8403 chip is 12-bit. However, newer revisions of these modules behave like 16-bit (left-aligned) devices.
Sending 12-bit max (0x0FFF) results in ~0.6V output.
Sending 15-bit max (0x7FFF) results in 5.0V output (half of the range).

The Solution:
To get full 0-10V range, you must treat the device as 16-bit and send data in range 0x0000 to 0xFFFF.
Usage:

// --- I2C ADDRESS DEFINITIONS ---
// Note: STM32 HAL expects the address shifted left by 1 bit.
#define DAC_ADDR_M5STACK  (0x59 << 1) // 0xB2
#define DAC_ADDR_DFROBOT  (0x58 << 1) // 0xB0 (can be changed via DIP)

/**
 * @brief  Sets the output voltage for GP8403/GP8413 I2C DAC modules.
 * @note   CRITICAL FIX: The official datasheet states this is a 12-bit chip (0-4095).
 * However, the hardware behaves like a 16-bit device (left-aligned).
 * Sending 12-bit max (0x0FFF) results in ~0.6V output.
 * Sending 15-bit max (0x7FFF) results in 5.0V output (half range).
 * To get full 0-10V range, we must send 16-bit data (0-65535).
 *
 * @param  hi2c      Pointer to a I2C_HandleTypeDef structure (e.g., &hi2c1).
 * @param  i2c_addr  The 7-bit I2C address shifted left by 1 (e.g., 0xB0 or 0xB2).
 * @param  voltage   Target voltage in Volts (0.0f to 10.0f).
 */
void DAC_SetVoltage(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, float voltage)
{
    //  Clamp voltage to valid range 0V - 10V
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > 10.0f) voltage = 10.0f;

    //  THE FIX: Scale voltage to 16-bit value (0 - 65535)
    // 10.0V = 0xFFFF (65535)
    uint16_t data_raw = (uint16_t)((voltage / 10.0f) * 65535.0f);

    // Prepare I2C data buffer (Little Endian: Low Byte first)
    uint8_t i2c_data[2];
    i2c_data[0] = (uint8_t)(data_raw & 0xFF);      // Low Byte
    i2c_data[1] = (uint8_t)((data_raw >> 8) & 0xFF); // High Byte

    //  Write data to both output channels
    // Register 0x02 = Channel 0
    HAL_I2C_Mem_Write(hi2c, i2c_addr, 0x02, 1, i2c_data, 2, 100);
    // Register 0x04 = Channel 1
    HAL_I2C_Mem_Write(hi2c, i2c_addr, 0x04, 1, i2c_data, 2, 100);
}

// --- EXAMPLE USAGE IN MAIN LOOP ---
/*
void main(void) {
  // ... HAL Init code ...

  while (1) {
      // Example for M5Stack DAC2 Unit
      // No need to worry about hex values, just pass the float voltage.

      DAC_SetVoltage(&hi2c1, DAC_ADDR_M5STACK, 2.5);  // Output: 2.5V
      HAL_Delay(2000);

      DAC_SetVoltage(&hi2c1, DAC_ADDR_M5STACK, 5.0);  // Output: 5.0V
      HAL_Delay(2000);

      DAC_SetVoltage(&hi2c1, DAC_ADDR_M5STACK, 10.0); // Output: 10.0V
      HAL_Delay(2000);
  }
}
*/
  

Tested Hardware:

DFRobot Gravity: I2C 2-Channel DAC (SKU: DFR0971)

M5Stack DAC2 Unit (SKU: U012-V2)

MCU: STM32F446RE (Nucleo-64)
