/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "pololu_lidar.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t writeReg(uint16_t reg, uint8_t value) {
	uint8_t buffer[3];
	buffer[0] = (uint8_t)((reg >> 8) & 0x00FF);
	buffer[1] = (uint8_t)(reg & 0x00FF);
	buffer[2] = value;
	return HAL_I2C_Master_Transmit(&hi2c1, LIDAR_I2C_W_ADDR, &buffer[0], 3, 1000);
}

uint8_t writeReg16Bit(uint16_t reg, uint16_t value) {
	uint8_t buffer[4];
	buffer[0] = (uint8_t)((reg >> 8) & 0x00FF);
	buffer[1] = (uint8_t)(reg & 0x00FF);
	buffer[2] = (uint8_t)((value >> 8) & 0x00FF);
	buffer[3] = (uint8_t)(value & 0x00FF);
	return HAL_I2C_Master_Transmit(&hi2c1, LIDAR_I2C_W_ADDR, &buffer[0], 4, 1000);
}

uint8_t writeReg32Bit(uint16_t reg, uint32_t value) {
	uint8_t buffer[6];
	buffer[0] = (uint8_t)((reg >> 8) & 0x00FF);
	buffer[1] = (uint8_t)(reg & 0x00FF);
	buffer[2] = (uint8_t)((value >> 24) & 0x00FF);
	buffer[3] = (uint8_t)((value >> 16) & 0x00FF);
	buffer[4] = (uint8_t)((value >> 8) & 0x00FF);
	buffer[5] = (uint8_t)(value & 0x00FF);
	return HAL_I2C_Master_Transmit(&hi2c1, LIDAR_I2C_W_ADDR, &buffer[0], 6, 1000);
}

uint8_t readReg(uint16_t reg, uint8_t readData[]) {
	uint8_t buffer[2];
	buffer[0] = (uint8_t)((reg >> 8) & 0x00FF);
	buffer[1] = (uint8_t)(reg & 0x00FF);
	uint8_t error = HAL_I2C_Master_Transmit(&hi2c1, LIDAR_I2C_W_ADDR, &buffer[0], 2, 1000);
	if (error != HAL_OK) return error;
	return HAL_I2C_Master_Receive(&hi2c1, LIDAR_I2C_W_ADDR, &readData[0], 1, 1000);
}

uint8_t readReg16Bit(uint16_t reg, uint8_t readData[]) {
	uint8_t buffer[2];
	buffer[0] = (uint8_t)((reg >> 8) & 0x00FF);
	buffer[1] = (uint8_t)(reg & 0x00FF);
	uint8_t error = HAL_I2C_Master_Transmit(&hi2c1, LIDAR_I2C_W_ADDR, &buffer[0], 2, 1000);
	if (error != HAL_OK) return error;
	return HAL_I2C_Master_Receive(&hi2c1, LIDAR_I2C_W_ADDR, &readData[0], 2, 1000);
}

uint8_t readReg32Bit(uint16_t reg, uint8_t readData[]) {
	uint8_t buffer[2];
	buffer[0] = (uint8_t)((reg >> 8) & 0x00FF);
	buffer[1] = (uint8_t)(reg & 0x00FF);
	uint8_t error = HAL_I2C_Master_Transmit(&hi2c1, LIDAR_I2C_W_ADDR, &buffer[0], 2, 1000);
	if (error != HAL_OK) return error;
	return HAL_I2C_Master_Receive(&hi2c1, LIDAR_I2C_W_ADDR, &readData[0], 4, 1000);
}

uint8_t convert(uint8_t readData[]) {
	return readData[0];
}

uint16_t convert16Bit(uint8_t readData[]) {
	return ((uint16_t)((uint16_t)readData[1]) << 8) + (uint16_t)readData[0];
}

uint32_t convert32Bit(uint8_t readData[]) {
	return ((uint32_t)((uint32_t)readData[3]) << 24) +
			((uint32_t)((uint32_t)readData[2]) << 16) +
			((uint32_t)((uint32_t)readData[1]) << 8) +
			(uint32_t)readData[0];
}

void lidarInit(void) {
	uint8_t ret;
	uint8_t buf[2];
	uint8_t buffer[1];

	// =========================== Added stuff begins

	ret = readReg16Bit(IDENTIFICATION__MODEL_ID, buf);
	if (ret != HAL_OK) printf("Error #-1\n");
	ret = writeReg(SOFT_RESET, 0);
	if (ret != HAL_OK) printf("Error #-2\n");
	ret = writeReg(SOFT_RESET, 1);
	if (ret != HAL_OK) printf("Error #-3\n");
	ret = readReg(FIRMWARE__SYSTEM_STATUS, buffer);
	if (ret != HAL_OK) printf("Error #-4\n");
	ret = readReg(PAD_I2C_HV__EXTSUP_CONFIG, buffer);
	if (ret != HAL_OK) printf("Error #-5\n");
	ret = writeReg(PAD_I2C_HV__EXTSUP_CONFIG, 1);
	if (ret != HAL_OK) printf("Error #-6\n");

	// =========================== Added stuff ends


	ret = readReg16Bit(OSC_MEASURED__FAST_OSC__FREQUENCY, buf);
	  fast_osc_frequency = convert16Bit(buf);
	  ret = readReg16Bit(RESULT__OSC_CALIBRATE_VAL, buf);
	  osc_calibrate_val = convert16Bit(buf);
	  ret = writeReg16Bit(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, 0x0A00); // should already be this value after reset
	  if (ret != HAL_OK) printf("Error #1\n");
	  ret = writeReg(GPIO__TIO_HV_STATUS, 0x02);
	  if (ret != HAL_OK) printf("Error #2\n");
	  ret = writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
	  if (ret != HAL_OK) printf("Error #3\n");
	  ret = writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
	  if (ret != HAL_OK) printf("Error #4\n");
	  ret = writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
	  if (ret != HAL_OK) printf("Error #5\n");
	  ret = writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
	  if (ret != HAL_OK) printf("Error #6\n");
	  ret = writeReg(ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
	  if (ret != HAL_OK) printf("Error #7\n");
	  ret = writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default
	  if (ret != HAL_OK) printf("Error #8\n");
	  // Successful till here (first two pages of dock)
	  // general config
	  ret = writeReg16Bit(SYSTEM__THRESH_RATE_HIGH, 0x0000);
	  if (ret != HAL_OK) printf("Error #9\n");
	  ret = writeReg16Bit(SYSTEM__THRESH_RATE_LOW, 0x0000);
	  if (ret != HAL_OK) printf("Error #10\n");
	  ret = writeReg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);
	  if (ret != HAL_OK) printf("Error #11\n");
	  // timing config
	  // most of these settings will be determined later by distance and timing
	  // budget configuration
	  ret = writeReg16Bit(RANGE_CONFIG__SIGMA_THRESH, 360); // tuning parm default
	  if (ret != HAL_OK) printf("Error #12\n");
	  ret = writeReg16Bit(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default
	  if (ret != HAL_OK) printf("Error #13\n");
	  // dynamic config

	  ret = writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
	  if (ret != HAL_OK) printf("Error #14\n");
	  ret = writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
	  if (ret != HAL_OK) printf("Error #15\n");
	  ret = writeReg(SD_CONFIG__QUANTIFIER, 2); // tuning parm default
	  if (ret != HAL_OK) printf("Error #16\n");
	  // VL53L1_preset_mode_standard_ranging() end
	  // Init works till here
	  // from VL53L1_preset_mode_timed_ranging_*
	  // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
	  // and things don't seem to work if we don't set GPH back to 0 (which the API
	  // does here).
	  ret = writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
	  if (ret != HAL_OK) printf("Error #17\n");
	  ret = writeReg(SYSTEM__SEED_CONFIG, 1); // tuning parm default
	  if (ret != HAL_OK) printf("Error #18\n");
	  // from VL53L1_config_low_power_auto_mode
	  ret = writeReg(SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
	  if (ret != HAL_OK) printf("Error #19\n");
	  ret = writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
	  if (ret != HAL_OK) printf("Error #20\n");
	  ret = writeReg(DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS
	  if (ret != HAL_OK) printf("Error #21\n");
	  // VL53L1_set_preset_mode() end

	  // default to long range, 50 ms timing budget
	  // note that this is different than what the API defaults to

	  setDistanceMode(Long);
	  setMeasurementTimingBudget(50000);
	  //setMeasurementTimingBudget(50000);

	  // VL53L1_StaticInit() end

	  // the API triggers this change in VL53L1_init_and_start_range() once a
	  // measurement is started; assumes MM1 and MM2 are disabled
	  uint8_t reader[2];
	  ret = readReg16Bit(MM_CONFIG__OUTER_OFFSET_MM, reader);
	  if (ret != HAL_OK) printf("Error #22\n");
	  uint16_t val = convert16Bit(reader) * 4;
	  ret = writeReg16Bit(ALGO__PART_TO_PART_RANGE_OFFSET_MM, val);
	  if (ret != HAL_OK) printf("Error #23\n");
}

void shortMode(void) {
	// from VL53L1_preset_mode_standard_ranging_short_range()
	uint8_t ret;
	// timing config
	ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
    if (ret != HAL_OK) printf("Error #24\n");
    ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
    if (ret != HAL_OK) printf("Error #25\n");
    ret = writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
    if (ret != HAL_OK) printf("Error #26\n");
    // dynamic config
    ret = writeReg(SD_CONFIG__WOI_SD0, 0x07);
    if (ret != HAL_OK) printf("Error #27\n");
    ret = writeReg(SD_CONFIG__WOI_SD1, 0x05);
    if (ret != HAL_OK) printf("Error #28\n");
    ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
    if (ret != HAL_OK) printf("Error #29\n");
    ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default
    if (ret != HAL_OK) printf("Error #30\n");
}

void mediumMode(void) {
	// from VL53L1_preset_mode_standard_ranging()
	uint8_t ret;
	  // timing config
	ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
	  if (ret != HAL_OK) printf("Error #31\n");
	  ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
	  if (ret != HAL_OK) printf("Error #32\n");
	  ret = writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);
	  if (ret != HAL_OK) printf("Error #33\n");

	  // dynamic config
	  ret = writeReg(SD_CONFIG__WOI_SD0, 0x0B);
	  if (ret != HAL_OK) printf("Error #34\n");
	  ret = writeReg(SD_CONFIG__WOI_SD1, 0x09);
	  if (ret != HAL_OK) printf("Error #35\n");
	  ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
	  if (ret != HAL_OK) printf("Error #36\n");
	  ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default
	  if (ret != HAL_OK) printf("Error #37\n");
}

void longMode(void) {
    // from VL53L1_preset_mode_standard_ranging_long_range()
	uint8_t ret;
    // timing config
	ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
    if (ret != HAL_OK) printf("Error #38\n");
    ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
    if (ret != HAL_OK) printf("Error #39\n");
    ret = writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
    if (ret != HAL_OK) printf("Error #40\n");

    // dynamic config
    ret = writeReg(SD_CONFIG__WOI_SD0, 0x0F);
    if (ret != HAL_OK) printf("Error #41\n");
    ret = writeReg(SD_CONFIG__WOI_SD1, 0x0D);
    if (ret != HAL_OK) printf("Error #42\n");
    ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
    if (ret != HAL_OK) printf("Error #43\n");
    ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default
    if (ret != HAL_OK) printf("Error #44\n");
}

// set distance mode to Short, Medium, or Long
// based on VL53L1_SetDistanceMode()
uint8_t setDistanceMode(uint8_t mode)
{
  // save existing timing budget
  uint32_t budget_us = getMeasurementTimingBudget();
  uint8_t ret;
  switch (mode)
  {
    case Short:
    	// from VL53L1_preset_mode_standard_ranging_short_range()
    	// timing config
    	ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
        if (ret != HAL_OK) printf("Error #24\n");
        ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
        if (ret != HAL_OK) printf("Error #25\n");
        ret = writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
        if (ret != HAL_OK) printf("Error #26\n");
        // dynamic config
        ret = writeReg(SD_CONFIG__WOI_SD0, 0x07);
        if (ret != HAL_OK) printf("Error #27\n");
        ret = writeReg(SD_CONFIG__WOI_SD1, 0x05);
        if (ret != HAL_OK) printf("Error #28\n");
        ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
        if (ret != HAL_OK) printf("Error #29\n");
        ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default
        if (ret != HAL_OK) printf("Error #30\n");
      break;

    case Medium:
      // from VL53L1_preset_mode_standard_ranging()
    	  // timing config
    	ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
    	  if (ret != HAL_OK) printf("Error #31\n");
    	  ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
    	  if (ret != HAL_OK) printf("Error #32\n");
    	  ret = writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);
    	  if (ret != HAL_OK) printf("Error #33\n");

    	  // dynamic config
    	  ret = writeReg(SD_CONFIG__WOI_SD0, 0x0B);
    	  if (ret != HAL_OK) printf("Error #34\n");
    	  ret = writeReg(SD_CONFIG__WOI_SD1, 0x09);
    	  if (ret != HAL_OK) printf("Error #35\n");
    	  ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
    	  if (ret != HAL_OK) printf("Error #36\n");
    	  ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default
    	  if (ret != HAL_OK) printf("Error #37\n");
    	  break;

    case Long: // long
      // from VL53L1_preset_mode_standard_ranging_long_range()
        // timing config
    	ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
        if (ret != HAL_OK) printf("Error #38\n");
        ret = writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
        if (ret != HAL_OK) printf("Error #39\n");
        ret = writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
        if (ret != HAL_OK) printf("Error #40\n");

        // dynamic config
        ret = writeReg(SD_CONFIG__WOI_SD0, 0x0F);
        if (ret != HAL_OK) printf("Error #41\n");
        ret = writeReg(SD_CONFIG__WOI_SD1, 0x0D);
        if (ret != HAL_OK) printf("Error #42\n");
        ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
        if (ret != HAL_OK) printf("Error #43\n");
        ret = writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default
        if (ret != HAL_OK) printf("Error #44\n");
      break;

    default:
      // unrecognized mode - do nothing
      return 0;
  }

  // reapply timing budget
  setMeasurementTimingBudget(budget_us);

  // save mode so it can be returned by getDistanceMode()
  distance_mode = mode;

  return 1;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate
// measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
uint8_t setMeasurementTimingBudget(uint32_t budget_us)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS
	uint8_t ret;
  if (budget_us <= TimingGuard) { return 0; }

  uint32_t range_config_timeout_us = budget_us -= TimingGuard;
  if (range_config_timeout_us > 1100000) { return 0; } // FDA_MAX_TIMING_BUDGET_US * 2

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values() begin

  uint32_t macro_period_us;

  // "Update Macro Period for Range A VCSEL Period"
  uint8_t buf1[4] = {0, 0, 0, 0};
  ret = readReg(RANGE_CONFIG__VCSEL_PERIOD_A, buf1);
  if (ret != HAL_OK) printf("Error #45\n");
  macro_period_us = calcMacroPeriod(convert32Bit(buf1));

  // "Update Phase timeout - uses Timing A"
  // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg().
  uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
  ret = writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);
  if (ret != HAL_OK) printf("Error #46\n");

  // "Update MM Timing A timeout"
  // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
  // actually ends up with a slightly different value because it gets assigned,
  // retrieved, recalculated with a different macro period, and reassigned,
  // but it probably doesn't matter because it seems like the MM ("mode
  // mitigation"?) sequence steps are disabled in low power auto mode anyway.
  ret = writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));
  if (ret != HAL_OK) printf("Error #47\n");

  // "Update Range Timing A timeout"
  // INCORRECT
  ret = writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
  if (ret != HAL_OK) printf("Error #48\n");
  // "Update Macro Period for Range B VCSEL Period"
  uint8_t buf2[4] = {0, 0, 0, 0};
  ret = readReg(RANGE_CONFIG__VCSEL_PERIOD_B, buf2);
  if (ret != HAL_OK) printf("Error #49\n");
  macro_period_us = calcMacroPeriod(convert32Bit(buf2));

  // "Update MM Timing B timeout"
  // (See earlier comment about MM Timing A timeout.)
  ret = writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));
  if (ret != HAL_OK) printf("Error #50\n");

  // "Update Range Timing B timeout"
  // INCORRECT
  ret = writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
  if (ret != HAL_OK) printf("Error #51\n");

  // VL53L1_calc_timeout_register_values() end

  return 1;
}

// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
uint32_t getMeasurementTimingBudget()
{
	uint8_t ret;
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // "Update Macro Period for Range A VCSEL Period"
  uint8_t buf1[1] = {0};
  ret = readReg(RANGE_CONFIG__VCSEL_PERIOD_A, buf1);
  if (ret != HAL_OK) printf("Error #52\n");
  uint32_t macro_period_us = calcMacroPeriod(convert(buf1));

  // "Get Range Timing A timeout"
  uint8_t buf2[2] = {0, 0};
  ret = readReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A, buf2);
  if (ret != HAL_OK) printf("Error #53\n");
  uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(convert16Bit(buf2)), macro_period_us);

  // VL53L1_get_timeouts_us() end

  return  2 * range_config_timeout_us + TimingGuard;
}

// Set the width and height of the region of interest
// based on VL53L1X_SetROI() from STSW-IMG009 Ultra Lite Driver
//
// ST user manual UM2555 explains ROI selection in detail, so we recommend
// reading that document carefully.
void setROISize(uint8_t width, uint8_t height)
{
	uint8_t ret;
  if ( width > 16) {  width = 16; }
  if (height > 16) { height = 16; }

  // Force ROI to be centered if width or height > 10, matching what the ULD API
  // does. (This can probably be overridden by calling setROICenter()
  // afterwards.)
  if (width > 10 || height > 10)
  {
	  ret = writeReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD, 199);
	  if (ret != HAL_OK) printf("Error #54\n");
  }

  ret = writeReg(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
           (height - 1) << 4 | (width - 1));
  if (ret != HAL_OK) printf("Error #55\n");
}

// Get the width and height of the region of interest (ROI)
// based on VL53L1X_GetROI_XY() from STSW-IMG009 Ultra Lite Driver
void getROISize(uint8_t * width, uint8_t * height)
{
	uint8_t buf1[1] = {0};
	uint8_t ret = readReg(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, buf1);
	if (ret != HAL_OK) printf("Error #56\n");
  uint8_t reg_val = convert(buf1);
  *width = (reg_val & 0xF) + 1;
  *height = (reg_val >> 4) + 1;
}

// Set the center SPAD of the region of interest (ROI)
// based on VL53L1X_SetROICenter() from STSW-IMG009 Ultra Lite Driver
//
// ST user manual UM2555 explains ROI selection in detail, so we recommend
// reading that document carefully. Here is a table of SPAD locations from
// UM2555 (199 is the default/center):
//
// 128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
// 129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
// 130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
// 131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
// 132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
// 133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
// 134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
// 135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255
//
// 127,119,111,103, 95, 87, 79, 71,   63, 55, 47, 39, 31, 23, 15,  7
// 126,118,110,102, 94, 86, 78, 70,   62, 54, 46, 38, 30, 22, 14,  6
// 125,117,109,101, 93, 85, 77, 69,   61, 53, 45, 37, 29, 21, 13,  5
// 124,116,108,100, 92, 84, 76, 68,   60, 52, 44, 36, 28, 20, 12,  4
// 123,115,107, 99, 91, 83, 75, 67,   59, 51, 43, 35, 27, 19, 11,  3
// 122,114,106, 98, 90, 82, 74, 66,   58, 50, 42, 34, 26, 18, 10,  2
// 121,113,105, 97, 89, 81, 73, 65,   57, 49, 41, 33, 25, 17,  9,  1
// 120,112,104, 96, 88, 80, 72, 64,   56, 48, 40, 32, 24, 16,  8,  0 <- Pin 1
//
// This table is oriented as if looking into the front of the sensor (or top of
// the chip). SPAD 0 is closest to pin 1 of the VL53L1X, which is the corner
// closest to the VDD pin on the Pololu VL53L1X carrier board:
//
//   +--------------+
//   |             O| GPIO1
//   |              |
//   |             O|
//   | 128    248   |
//   |+----------+ O|
//   ||+--+  +--+|  |
//   |||  |  |  || O|
//   ||+--+  +--+|  |
//   |+----------+ O|
//   | 120      0   |
//   |             O|
//   |              |
//   |             O| VDD
//   +--------------+
//
// However, note that the lens inside the VL53L1X inverts the image it sees
// (like the way a camera works). So for example, to shift the sensor's FOV to
// sense objects toward the upper left, you should pick a center SPAD in the
// lower right.
void setROICenter(uint8_t spadNumber)
{
	uint8_t ret;
	ret = writeReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD, spadNumber);
	if (ret != HAL_OK) printf("Error #57\n");
}

// Get the center SPAD of the region of interest
// based on VL53L1X_GetROICenter() from STSW-IMG009 Ultra Lite Driver
uint8_t getROICenter()
{
	uint8_t buf1[1] = {0};
	uint8_t ret = readReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD, buf1);
	if (ret != HAL_OK) printf("Error #58\n");
  return convert(buf1);
}

// Start continuous ranging measurements, with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
void startContinuous(uint32_t period_ms)
{
	uint8_t ret;
  // from VL53L1_set_inter_measurement_period_ms()
	ret = writeReg32Bit(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);
	if (ret != HAL_OK) printf("Error #59\n");

	ret = writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
	if (ret != HAL_OK) printf("Error #60\n");
	ret = writeReg(SYSTEM__MODE_START, 0x40); // mode_range__timed
	if (ret != HAL_OK) printf("Error #61\n");
}

// Stop continuous measurements
// based on VL53L1_stop_range()
void stopContinuous()
{
	uint8_t ret;
	ret = writeReg(SYSTEM__MODE_START, 0x80); // mode_range__abort
	if (ret != HAL_OK) printf("Error #62\n");

  // VL53L1_low_power_auto_data_stop_range() begin

  calibrated = 0;

  // "restore vhv configs"
  if (saved_vhv_init != 0)
  {
	  ret = writeReg(VHV_CONFIG__INIT, saved_vhv_init);
	  if (ret != HAL_OK) printf("Error #63\n");
  }
  if (saved_vhv_timeout != 0)
  {
	  ret = writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
	  if (ret != HAL_OK) printf("Error #64\n");
  }

  // "remove phasecal override"
  ret = writeReg(PHASECAL_CONFIG__OVERRIDE, 0x00);
  if (ret != HAL_OK) printf("Error #65\n");

  // VL53L1_low_power_auto_data_stop_range() end
}

// Returns a range reading in millimeters when continuous mode is active. If
// blocking is true (the default), this function waits for a new measurement to
// be available. If blocking is false, it will try to return data immediately.
// (readSingle() also calls this function after starting a single-shot range
// measurement)
uint16_t read(uint8_t blocking)
{
	uint8_t ret;
  if (blocking)
  {
//    startTimeout();
//    while (!dataReady())
//    {
//      if (checkTimeoutExpired())
//      {
//        did_timeout = 1;
//        return 0;
//      }
//    }
  }

  readResults();

  if (!calibrated)
  {
    setupManualCalibration();
    calibrated = 1;
  }

  updateDSS();

  getRangingData();

  ret = writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  if (ret != HAL_OK) printf("Error #66\n");

  return ranging_data.range_mm;
}

uint16_t readSingle(uint8_t blocking)
{
	uint8_t ret;
	ret = writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
	if (ret != HAL_OK) printf("Error #67\n");
	ret = writeReg(SYSTEM__MODE_START, 0x10); // mode_range__single_shot
	if (ret != HAL_OK) printf("Error #68\n");

  if (blocking)
  {
    return read(1);
  }
  else
  {
    return 0;
  }
}

const char* rangeStatusToString(uint8_t status)
{
  switch (status)
  {
    case RangeValid:
      return "range valid";

    case SigmaFail:
      return "sigma fail";

    case SignalFail:
      return "signal fail";

    case RangeValidMinRangeClipped:
      return "range valid, min range clipped";

    case OutOfBoundsFail:
      return "out of bounds fail";

    case HardwareFail:
      return "hardware fail";

    case RangeValidNoWrapCheckFail:
      return "range valid, no wrap check fail";

    case WrapTargetFail:
      return "wrap target fail";

    case XtalkSignalFail:
      return "xtalk signal fail";

    case SynchronizationInt:
      return "synchronization int";

    case MinRangeFail:
      return "min range fail";

    case None:
      return "no update";

    default:
      return "unknown status";
  }
}

uint8_t timeoutOccurred()
{
  uint8_t tmp = did_timeout;
  did_timeout = 0;
  return tmp;
}

void setupManualCalibration()
{

  // "save original vhv configs"
	uint8_t buf1[1] = {0};
	uint8_t ret = readReg(VHV_CONFIG__INIT, buf1);
	if (ret != HAL_OK) printf("Error #69\n");
  saved_vhv_init = convert(buf1);
  uint8_t buf2[1] = {0};
  ret = readReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, buf2);
  if (ret != HAL_OK) printf("Error #70\n");
  saved_vhv_timeout = convert(buf2);

  // "disable VHV init"
  ret = writeReg(VHV_CONFIG__INIT, saved_vhv_init & 0x7F);
  if (ret != HAL_OK) printf("Error #71\n");

  // "set loop bound to tuning param"
  ret = writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
    (saved_vhv_timeout & 0x03) + (3 << 2)); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)
  if (ret != HAL_OK) printf("Error #72\n");

  // "override phasecal"
  ret = writeReg(PHASECAL_CONFIG__OVERRIDE, 0x01);
  if (ret != HAL_OK) printf("Error #73\n");
  uint8_t buf3[1] = {0};
  ret = readReg(PHASECAL_RESULT__VCSEL_START, buf3);
  if (ret != HAL_OK) printf("Error #74\n");
  ret = writeReg(CAL_CONFIG__VCSEL_START, convert(buf3));
  if (ret != HAL_OK) printf("Error #75\n");
}

// TODO LiDAR READ RESULTS FUNCTION

void updateDSS()
{
	uint8_t ret;
  uint16_t spadCount = results.dss_actual_effective_spads_sd0;

  if (spadCount != 0)
  {
    // "Calc total rate per spad"

    uint32_t totalRatePerSpad =
      (uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
      results.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    // "shift up to take advantage of 32 bits"
    totalRatePerSpad <<= 16;

    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0)
    {
      // "get the target rate and shift up by 16"
      uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

      // "clip to 16 bit"
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      // "override DSS config"
      ret = writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
      if (ret != HAL_OK) printf("Error #76\n");
      // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

      return;
    }
  }

  // If we reached this point, it means something above would have resulted in a
  // divide by zero.
  // "We want to gracefully set a spad target, not just exit with an error"

   // "set target to mid point"
  ret = writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
  if (ret != HAL_OK) printf("Error #77\n");
}

void getRangingData()
{
  // VL53L1_copy_sys_and_core_results_to_range_results() begin

  uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

  // "apply correction gain"
  // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
  // Basically, this appears to scale the result by 2011/2048, or about 98%
  // (with the 1024 added for proper rounding).
  ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

  // VL53L1_copy_sys_and_core_results_to_range_results() end

  // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
  // mostly based on ConvertStatusLite()
  switch(results.range_status)
  {
    case 17: // MULTCLIPFAIL
    case 2: // VCSELWATCHDOGTESTFAILURE
    case 1: // VCSELCONTINUITYTESTFAILURE
    case 3: // NOVHVVALUEFOUND
      // from SetSimpleData()
      ranging_data.range_status = HardwareFail;
      break;

    case 13: // USERROICLIP
     // from SetSimpleData()
      ranging_data.range_status = MinRangeFail;
      break;

    case 18: // GPHSTREAMCOUNT0READY
      ranging_data.range_status = SynchronizationInt;
      break;

    case 5: // RANGEPHASECHECK
      ranging_data.range_status =  OutOfBoundsFail;
      break;

    case 4: // MSRCNOTARGET
      ranging_data.range_status = SignalFail;
      break;

    case 6: // SIGMATHRESHOLDCHECK
      ranging_data.range_status = SigmaFail;
      break;

    case 7: // PHASECONSISTENCY
      ranging_data.range_status = WrapTargetFail;
      break;

    case 12: // RANGEIGNORETHRESHOLD
      ranging_data.range_status = XtalkSignalFail;
      break;

    case 8: // MINCLIP
      ranging_data.range_status = RangeValidMinRangeClipped;
      break;

    case 9: // RANGECOMPLETE
      // from VL53L1_copy_sys_and_core_results_to_range_results()
      if (results.stream_count == 0)
      {
        ranging_data.range_status = RangeValidNoWrapCheckFail;
      }
      else
      {
        ranging_data.range_status = RangeValid;
      }
      break;

    default:
      ranging_data.range_status = None;
  }

  // from SetSimpleData()
  ranging_data.peak_signal_count_rate_MCPS =
    countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  ranging_data.ambient_count_rate_MCPS =
    countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}

uint32_t decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t encodeTimeout(uint32_t timeout_mclks)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t calcMacroPeriod(uint8_t vcsel_period)
{
  // from VL53L1_calc_pll_period_us()
  // fast osc frequency in 4.12 format; PLL period in 0.24 format
  uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

  // from VL53L1_decode_vcsel_period()
  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
  uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}

uint16_t readRangeContinuousMillimeters(uint8_t blocking) { return read(blocking); } // alias of read()
uint16_t readRangeSingleMillimeters(uint8_t blocking) { return readSingle(blocking); } // alias of readSingle()
float countRateFixedToFloat(uint16_t count_rate_fixed) { return (float)count_rate_fixed / (1 << 7); }

uint8_t dataReady() {
	uint8_t buf[4] = {0, 0, 0, 0};
	uint8_t ret = readReg32Bit(GPIO__TIO_HV_STATUS, buf);
	if (ret != HAL_OK) printf("Error #78\n");
	return (convert32Bit(buf) & 0x01) == 0;
}

void readResults() {
    uint8_t range[17];
    uint8_t ret;
    uint8_t buffer[2];
    buffer[0] = (uint8_t)((RESULT__RANGE_STATUS >> 8) & 0x00FF);
    buffer[1] = (uint8_t)(RESULT__RANGE_STATUS & 0x00FF);
    ret = HAL_I2C_Master_Transmit(&hi2c1, LIDAR_I2C_W_ADDR, &buffer[0], 2, 1000);
    if (ret != HAL_OK) printf("Error A\n");
    else {
		  HAL_I2C_Master_Receive(&hi2c1, LIDAR_I2C_W_ADDR, &range[0], 17, 1000);
		  if (ret != HAL_OK) {
			printf("Error B\n");
		  }
		  else {
			  results.range_status = range[0];
			  results.stream_count = range[2];
			  results.dss_actual_effective_spads_sd0 = convert16Bit(&range[3]);
			  results.ambient_count_rate_mcps_sd0 = convert16Bit(&range[7]);
			  results.final_crosstalk_corrected_range_mm_sd0 = convert16Bit(&range[13]);
			  results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = convert16Bit(&range[15]);
//			  for (uint8_t i = 0; i < 17; ++i) {
//				  printf ("%x ", range[i]);
//			  }
//			  printf("was the received value.\n");
		  }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    lidarInit();
	setDistanceMode(Short);
	setMeasurementTimingBudget(50000);
    startContinuous(50);
    volatile uint16_t readValue;
    while (1)
    {
  	  // Device Addr is 0x52 for LIDAR
    	//readResults();
    	//TODO read() needs timeout errors fixed
    	//while (dataReady() == 0);
    	readValue = read(1);
    	if (ranging_data.range_status == RangeValid) {
    		printf("The received value was %i.\n", readValue);
    	}
    	else printf("Invalid Range\n");
		HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE11 PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM1_COMP1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

