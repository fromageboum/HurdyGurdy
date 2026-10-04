// Reading and processing of distance sensor values for the 3rd string

#include "Distsensor.h"
#include "Config.h"
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <cmath>
#include <vector>
#include <libraries/Oscillator/Oscillator.h>

// ------------------------------------------------------------------
// Scales available for the 3rd string
// ------------------------------------------------------------------

// Chosen scale: 0 = none (continuous mapping), 1..5 = scales below
static int scaleChoice = 0;

// Frequency references, in Hz
// C4, C#4, D4, D#4, E4, F4, F#4, G4, G#4, A4, A#4, B4
static const std::vector<float> CMajor = {261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392, 415.30, 440, 466.16, 493.88};

static const std::vector<float> fiveEDO = {440, 505.43, 580.58, 666.92, 766.08, 880.00};

static const std::vector<float> eightEDO = {440, 479.82, 523.25, 570.61, 622.25, 678.57, 739.99, 806.96, 880.00};

// 'bem', 'gulu', 'dada', 'pelog', 'lima', 'nem', 'barang' notes, cycling for the additional values
static const std::vector<float> pelog = {262, 282, 303, 362, 384, 409, 451, 524, 544, 564, 585, 644, 660, 702};

static const std::vector<float> centaur = {264, 277.20, 297, 308, 330, 352, 369.60, 396, 410.667, 440, 462, 495, 528}; // not sure yet :v

// Distances (mm) returned by the 24 keys of the hurdy gurdy keyboard
static const std::vector<float> keyValues = {40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270};

// Frequency of each of the 24 keys for the current scale (sized once in setup)
static std::vector<float> tunedKeys;

// ------------------------------------------------------------------
// Port of the Arduino VL53L1X class (Pololu) for Bela
// ------------------------------------------------------------------
class VL53L1X {
public:
	enum DistanceMode { Unknown, Short, Medium, Long };

	enum RangeStatus {
		RangeValid = 0,
		SigmaFail = 1,
		SignalFail = 2,
		RangeValidMinRangeClipped = 3,
		OutOfBoundsFail = 4,
		HardwareFail = 5,
		RangeValidNoWrapCheckFail = 6,
		WrapTargetFail = 7,
		XtalkSignalFail = 9,
		SynchronizationInt = 10,
		MinRangeFail = 13,
		None = 255,
	};

	struct RangingData {
		uint16_t range_mm;
		RangeStatus range_status;
	};

	RangingData ranging_data;

	bool begin(int bus, uint8_t i2cAddress = 0x29)
	{
		char path[32];
		snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
		fd_ = open(path, O_RDWR);
		if(fd_ < 0) {
			fprintf(stderr, "VL53L1X: impossible d'ouvrir %s\n", path);
			return false;
		}
		if(ioctl(fd_, I2C_SLAVE, i2cAddress) < 0) {
			fprintf(stderr, "VL53L1X: adresse I2C %#x refusee\n", i2cAddress);
			return false;
		}
		return init();
	}

	void end()
	{
		if(fd_ >= 0) {
			stopContinuous();
			close(fd_);
			fd_ = -1;
		}
	}

	void startContinuous(uint32_t period_ms)
	{
		writeReg32(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val_);
		writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);
		writeReg(SYSTEM__MODE_START, 0x40); // mode_range__timed
	}

	void stopContinuous()
	{
		writeReg(SYSTEM__MODE_START, 0x80); // mode_range__abort
		calibrated_ = false;
		if(saved_vhv_init_ != 0) writeReg(VHV_CONFIG__INIT, saved_vhv_init_);
		if(saved_vhv_timeout_ != 0) writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout_);
		writeReg(PHASECAL_CONFIG__OVERRIDE, 0x00);
	}

	int read(bool blocking = true)
	{
		if(fd_ < 0) return -1;

		if(blocking) {
			auto start = std::chrono::steady_clock::now();
			while(!dataReady()) {
				usleep(1000);
				auto elapsed = std::chrono::steady_clock::now() - start;
				if(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > 500)
					return -1;
			}
		} else if(!dataReady()) {
			return -1;
		}

		readResults();
		if(!calibrated_) {
			setupManualCalibration();
			calibrated_ = true;
		}
		updateDSS();
		getRangingData();
		writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);

		return ranging_data.range_mm;
	}

	bool setDistanceMode(DistanceMode mode)
	{
		uint32_t budget_us = getMeasurementTimingBudget();
		switch(mode) {
			case Short:
				writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
				writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
				writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
				writeReg(SD_CONFIG__WOI_SD0, 0x07);
				writeReg(SD_CONFIG__WOI_SD1, 0x05);
				writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6);
				writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6);
				break;
			case Medium:
				writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
				writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
				writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);
				writeReg(SD_CONFIG__WOI_SD0, 0x0B);
				writeReg(SD_CONFIG__WOI_SD1, 0x09);
				writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10);
				writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10);
				break;
			case Long:
				writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
				writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
				writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
				writeReg(SD_CONFIG__WOI_SD0, 0x0F);
				writeReg(SD_CONFIG__WOI_SD1, 0x0D);
				writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14);
				writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14);
				break;
			default:
				return false;
		}
		setMeasurementTimingBudget(budget_us);
		distance_mode_ = mode;
		return true;
	}

	bool setMeasurementTimingBudget(uint32_t budget_us)
	{
		if(budget_us <= kTimingGuard) return false;
		uint32_t range_config_timeout_us = budget_us - kTimingGuard;
		if(range_config_timeout_us > 1100000) return false;
		range_config_timeout_us /= 2;

		uint32_t macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));
		uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
		if(phasecal_timeout_mclks > 0xFF) phasecal_timeout_mclks = 0xFF;
		writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

		writeReg16(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));
		writeReg16(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

		macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_B));
		writeReg16(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));
		writeReg16(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

		return true;
	}

	uint32_t getMeasurementTimingBudget()
	{
		uint32_t macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));
		uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(
			decodeTimeout(readReg16(RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);
		return 2 * range_config_timeout_us + kTimingGuard;
	}

	// Field of view (FOV) through the ROI.
	// width/height: from 4 (minimum, ~15 degrees) to 16 (maximum, ~27 degrees, default)
	void setROISize(uint8_t width, uint8_t height)
	{
		if(width > 16) width = 16;
		if(height > 16) height = 16;
		// height in the 4 high bits, width in the 4 low bits of ONE byte (shift by 4, not 8)
		writeReg(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, (uint8_t)(((height - 1) << 4) | (width - 1)));
	}

	// ROI center. 199 = exact center of the SPAD array.
	void setROICenter(uint8_t spadNum)
	{
		writeReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD, spadNum);
	}

private:
	int fd_ = -1;
	bool calibrated_ = false;
	uint8_t saved_vhv_init_ = 0;
	uint8_t saved_vhv_timeout_ = 0;
	DistanceMode distance_mode_ = Unknown;
	uint16_t fast_osc_frequency_ = 0;
	uint16_t osc_calibrate_val_ = 0;

	struct ResultBuffer {
		uint8_t range_status;
		uint16_t stream_count;
		uint16_t dss_actual_effective_spads_sd0;
		uint16_t ambient_count_rate_mcps_sd0;
		uint16_t final_crosstalk_corrected_range_mm_sd0;
		uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
	} results_;

	static const uint16_t kTargetRate = 0x0A00;
	static const uint32_t kTimingGuard = 4528;

	enum : uint16_t {
		SOFT_RESET = 0x0000,
		VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008,
		VHV_CONFIG__INIT = 0x000B,
		ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM = 0x0039,
		ALGO__RANGE_IGNORE_VALID_HEIGHT_MM = 0x003E,
		ALGO__RANGE_MIN_CLIP = 0x003F,
		ALGO__CONSISTENCY_CHECK__TOLERANCE = 0x0040,
		SYSTEM__THRESH_RATE_HIGH = 0x0050,
		SYSTEM__THRESH_RATE_LOW = 0x0052,
		DSS_CONFIG__APERTURE_ATTENUATION = 0x0057,
		MM_CONFIG__TIMEOUT_MACROP_A = 0x005A,
		MM_CONFIG__TIMEOUT_MACROP_B = 0x005C,
		RANGE_CONFIG__TIMEOUT_MACROP_A = 0x005E,
		RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060,
		RANGE_CONFIG__TIMEOUT_MACROP_B = 0x0061,
		RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063,
		RANGE_CONFIG__SIGMA_THRESH = 0x0064,
		RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066,
		RANGE_CONFIG__VALID_PHASE_HIGH = 0x0069,
		SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C,
		SYSTEM__GROUPED_PARAMETER_HOLD_0 = 0x0071,
		SD_CONFIG__QUANTIFIER = 0x007E,
		ROI_CONFIG__USER_ROI_CENTRE_SPAD = 0x007F,
		ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080,
		SYSTEM__SEED_CONFIG = 0x0077,
		SD_CONFIG__WOI_SD0 = 0x0078,
		SD_CONFIG__WOI_SD1 = 0x0079,
		SD_CONFIG__INITIAL_PHASE_SD0 = 0x007A,
		SD_CONFIG__INITIAL_PHASE_SD1 = 0x007B,
		SYSTEM__GROUPED_PARAMETER_HOLD_1 = 0x007C,
		SYSTEM__SEQUENCE_CONFIG = 0x0081,
		SYSTEM__GROUPED_PARAMETER_HOLD = 0x0082,
		SYSTEM__INTERRUPT_CLEAR = 0x0086,
		SYSTEM__MODE_START = 0x0087,
		RESULT__RANGE_STATUS = 0x0089,
		GPIO__TIO_HV_STATUS = 0x0031,
		PHASECAL_CONFIG__TIMEOUT_MACROP = 0x004B,
		PHASECAL_CONFIG__OVERRIDE = 0x004D,
		CAL_CONFIG__VCSEL_START = 0x0047,
		PHASECAL_RESULT__VCSEL_START = 0x00D8,
		DSS_CONFIG__TARGET_TOTAL_RATE_MCPS = 0x0024,
		DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT = 0x0054,
		DSS_CONFIG__ROI_MODE_CONTROL = 0x004F,
		SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS = 0x0036,
		SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS = 0x0037,
		OSC_MEASURED__FAST_OSC__FREQUENCY = 0x0006,
		RESULT__OSC_CALIBRATE_VAL = 0x00DE,
		FIRMWARE__SYSTEM_STATUS = 0x00E5,
		IDENTIFICATION__MODEL_ID = 0x010F,
		ALGO__PART_TO_PART_RANGE_OFFSET_MM = 0x001E,
		MM_CONFIG__OUTER_OFFSET_MM = 0x0022,
	};

	bool init()
	{
		if(readReg16(IDENTIFICATION__MODEL_ID) != 0xEACC) {
			fprintf(stderr, "VL53L1X: capteur non detecte (mauvais MODEL_ID)\n");
			return false;
		}

		writeReg(SOFT_RESET, 0x00);
		usleep(100);
		writeReg(SOFT_RESET, 0x01);
		usleep(1000);

		auto start = std::chrono::steady_clock::now();
		while((readReg(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0) {
			usleep(1000);
			auto elapsed = std::chrono::steady_clock::now() - start;
			if(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > 1000) {
				fprintf(stderr, "VL53L1X: timeout au boot\n");
				return false;
			}
		}

		fast_osc_frequency_ = readReg16(OSC_MEASURED__FAST_OSC__FREQUENCY);
		osc_calibrate_val_ = readReg16(RESULT__OSC_CALIBRATE_VAL);

		writeReg16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, kTargetRate);
		writeReg(GPIO__TIO_HV_STATUS, 0x02);
		writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);
		writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16);
		writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
		writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
		writeReg(ALGO__RANGE_MIN_CLIP, 0);
		writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2);

		writeReg16(SYSTEM__THRESH_RATE_HIGH, 0x0000);
		writeReg16(SYSTEM__THRESH_RATE_LOW, 0x0000);
		writeReg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

		writeReg16(RANGE_CONFIG__SIGMA_THRESH, 360);
		writeReg16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192);

		writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
		writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
		writeReg(SD_CONFIG__QUANTIFIER, 2);

		writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
		writeReg(SYSTEM__SEED_CONFIG, 1);

		writeReg(SYSTEM__SEQUENCE_CONFIG, 0x8B);
		writeReg16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
		writeReg(DSS_CONFIG__ROI_MODE_CONTROL, 2);

		setDistanceMode(Short); // short range of 0-30 centimeters
		setMeasurementTimingBudget(20000); // minimum measurement budget for the short mode

		writeReg16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, readReg16(MM_CONFIG__OUTER_OFFSET_MM) * 4);

		return true;
	}

	bool dataReady()
	{
		return (readReg(GPIO__TIO_HV_STATUS) & 0x01) == 0;
	}

	void setupManualCalibration()
	{
		saved_vhv_init_ = readReg(VHV_CONFIG__INIT);
		saved_vhv_timeout_ = readReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);
		writeReg(VHV_CONFIG__INIT, saved_vhv_init_ & 0x7F);
		writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (saved_vhv_timeout_ & 0x03) + (3 << 2));
		writeReg(PHASECAL_CONFIG__OVERRIDE, 0x01);
		writeReg(CAL_CONFIG__VCSEL_START, readReg(PHASECAL_RESULT__VCSEL_START));
	}

	void readResults()
	{
		uint8_t addr[2] = { (uint8_t)(RESULT__RANGE_STATUS >> 8), (uint8_t)(RESULT__RANGE_STATUS & 0xFF) };
		writeRaw(addr, 2);
		uint8_t buf[17];
		readRaw(buf, 17);

		results_.range_status = buf[0];
		results_.stream_count = buf[2];
		results_.dss_actual_effective_spads_sd0 = ((uint16_t)buf[3] << 8) | buf[4];
		results_.ambient_count_rate_mcps_sd0 = ((uint16_t)buf[7] << 8) | buf[8];
		results_.final_crosstalk_corrected_range_mm_sd0 = ((uint16_t)buf[13] << 8) | buf[14];
		results_.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = ((uint16_t)buf[15] << 8) | buf[16];
	}

	void updateDSS()
	{
		uint16_t spadCount = results_.dss_actual_effective_spads_sd0;
		if(spadCount != 0) {
			uint32_t totalRatePerSpad = (uint32_t)results_.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
			                            results_.ambient_count_rate_mcps_sd0;
			if(totalRatePerSpad > 0xFFFF) totalRatePerSpad = 0xFFFF;
			totalRatePerSpad <<= 16;
			totalRatePerSpad /= spadCount;
			if(totalRatePerSpad != 0) {
				uint32_t requiredSpads = ((uint32_t)kTargetRate << 16) / totalRatePerSpad;
				if(requiredSpads > 0xFFFF) requiredSpads = 0xFFFF;
				writeReg16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
				return;
			}
		}
		writeReg16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
	}

	void getRangingData()
	{
		uint16_t range = results_.final_crosstalk_corrected_range_mm_sd0;
		ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

		switch(results_.range_status) {
			case 17: case 2: case 1: case 3:
				ranging_data.range_status = HardwareFail; break;
			case 13:
				ranging_data.range_status = MinRangeFail; break;
			case 18:
				ranging_data.range_status = SynchronizationInt; break;
			case 5:
				ranging_data.range_status = OutOfBoundsFail; break;
			case 4:
				ranging_data.range_status = SignalFail; break;
			case 6:
				ranging_data.range_status = SigmaFail; break;
			case 7:
				ranging_data.range_status = WrapTargetFail; break;
			case 12:
				ranging_data.range_status = XtalkSignalFail; break;
			case 8:
				ranging_data.range_status = RangeValidMinRangeClipped; break;
			case 9:
				ranging_data.range_status = (results_.stream_count == 0) ? RangeValidNoWrapCheckFail : RangeValid;
				break;
			default:
				ranging_data.range_status = None;
		}
	}

	static uint32_t decodeTimeout(uint16_t reg_val)
	{
		return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
	}

	static uint16_t encodeTimeout(uint32_t timeout_mclks)
	{
		uint32_t ls_byte = 0;
		uint16_t ms_byte = 0;
		if(timeout_mclks > 0) {
			ls_byte = timeout_mclks - 1;
			while((ls_byte & 0xFFFFFF00) > 0) { ls_byte >>= 1; ms_byte++; }
			return (ms_byte << 8) | (ls_byte & 0xFF);
		}
		return 0;
	}

	static uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
	{
		return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
	}

	static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
	{
		return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
	}

	uint32_t calcMacroPeriod(uint8_t vcsel_period)
	{
		uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency_;
		uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;
		uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
		macro_period_us >>= 6;
		macro_period_us *= vcsel_period_pclks;
		macro_period_us >>= 6;
		return macro_period_us;
	}

	void writeRaw(const uint8_t* buf, size_t len)
	{
		if(write(fd_, buf, len) != (ssize_t)len)
			fprintf(stderr, "VL53L1X: echec ecriture I2C (%s)\n", strerror(errno));
	}

	void readRaw(uint8_t* buf, size_t len)
	{
		if(::read(fd_, buf, len) != (ssize_t)len)
			fprintf(stderr, "VL53L1X: echec lecture I2C (%s)\n", strerror(errno));
	}

	void writeReg(uint16_t reg, uint8_t val)
	{
		uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val };
		writeRaw(buf, 3);
	}

	void writeReg16(uint16_t reg, uint16_t val)
	{
		uint8_t buf[4] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
		writeRaw(buf, 4);
	}

	void writeReg32(uint16_t reg, uint32_t val)
	{
		uint8_t buf[6] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
		                    (uint8_t)(val >> 24), (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
		writeRaw(buf, 6);
	}

	uint8_t readReg(uint16_t reg)
	{
		uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
		writeRaw(addr, 2);
		uint8_t val = 0;
		readRaw(&val, 1);
		return val;
	}

	uint16_t readReg16(uint16_t reg)
	{
		uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
		writeRaw(addr, 2);
		uint8_t val[2] = { 0, 0 };
		readRaw(val, 2);
		return ((uint16_t)val[0] << 8) | val[1];
	}
};

// ------------------------------------------------------------------
// Bela integration
// ------------------------------------------------------------------
static VL53L1X gSensor;
static AuxiliaryTask gReadSensorTask;
volatile int gLatestDistanceMM = -1;

// Smoothing (exponential moving average) to stabilise the measurements
static float gSmoothedDistance = -1;
static const float kSmoothingFactor = 0.2f;

// "Drone" oscillator driven by the distance
static Oscillator gOscillator;
static float gFrequency = 300.0f;

// Continuous mode: distance (mm) -> frequency (Hz)
static const float kMinDistanceMM = 20.0f;
static const float kMaxDistanceMM = 300.0f;
static const float kMinFrequencyHz = 100.0f;
static const float kMaxFrequencyHz = 400.0f;

// Last distance used to compute gFrequency (-2 forces a recompute)
static int lastDistanceMM = -2;

// Drone amplitude, controlled by a dedicated potentiometer (analog pin 3),
// independent from the main volume
static const unsigned int droneVolumePin = 3;
static float gDroneAmplitude = 0.15f; // default value before the first pot reading

// Enable/disable button, on digital pin 8
static const unsigned int distanceEnablePin = 8;
bool gDistanceSensorEnabled = false; // disabled by default at startup

// Debounce of the toggle button (independent from the other buttons)
static bool enableCandidateState = true;
static int enableCandidateCounter = 0;
static bool enableStableState = true;
static bool previousEnableStableState = true;

// --- Waveform selection buttons (one per waveform) ---
static const unsigned int sineButtonPin = 2;
static const unsigned int triangleButtonPin = 3;
static const unsigned int squareButtonPin = 4;
static const unsigned int sawtoothButtonPin = 5;

static bool waveButtonState[4];

// Independent debounce for each of the 4 waveform buttons
static bool waveCandidateState[4] = {true, true, true, true};
static int waveCandidateCounter[4] = {0, 0, 0, 0};
static bool waveStableState[4] = {true, true, true, true};
static bool previousWaveStableState[4] = {true, true, true, true};

// Waveform types and their names, in the same order as the buttons
static Oscillator::Type waveType[] = { Oscillator::sine, Oscillator::triangle, Oscillator::square, Oscillator::sawtooth };
static const char* waveNames[] = {"sine", "triangle", "square", "sawtooth"};

// Fills tunedKeys (one frequency per key) from the chosen scale.
// The scale is repeated octave after octave to cover all 24 keys.
static void tuningSynthString()
{
	const std::vector<float>* scale = nullptr;

	switch (scaleChoice) {
		case 1: scale = &CMajor;   break;
		case 2: scale = &fiveEDO;  break;
		case 3: scale = &eightEDO; break;
		case 4: scale = &pelog;    break;
		case 5: scale = &centaur;  break;
		default: return; // 0 = no scale, continuous mode
	}

	// If the last note is already the octave of the first one, it is not part
	// of the cycle (otherwise that note would be played twice in a row)
	size_t period = scale->size();
	if (fabsf(scale->back() - 2.0f * scale->front()) < 0.01f * scale->front()) {
		period--;
	}

	for (size_t n = 0; n < tunedKeys.size(); n++) {
		size_t degree = n % period;
		int octave = (int)(n / period);
		tunedKeys[n] = (*scale)[degree] * powf(2.0f, octave);
	}
}

// Frequency of the key closest to the measured distance
static float frequencyForDistance(int distanceMM)
{
	size_t best = 0;
	float bestDiff = fabsf(distanceMM - keyValues[0]);

	for (size_t i = 1; i < keyValues.size(); i++) {
		float diff = fabsf(distanceMM - keyValues[i]);
		if (diff < bestDiff) {
			bestDiff = diff;
			best = i;
		}
	}
	return tunedKeys[best];
}

static void readSensorLoop(void*)
{
	gSensor.startContinuous(20); // inter-measurement period: 20ms
	while(!Bela_stopRequested())
	{
		int distanceMM = gSensor.read(true); // blocking, waits for each new measurement
		if(distanceMM >= 0) {
			if(gSmoothedDistance < 0) {
				gSmoothedDistance = distanceMM;
			} else {
				gSmoothedDistance = kSmoothingFactor * distanceMM + (1 - kSmoothingFactor) * gSmoothedDistance;
			}
			gLatestDistanceMM = (int)gSmoothedDistance;
		}
	}
	gSensor.stopContinuous();
}

bool distanceSensorSetup(BelaContext *context)
{
	const int kI2cBus = 1; // same bus as the Trill sensors (different addresses)
	if(!gSensor.begin(kI2cBus, 0x29)) {
		fprintf(stderr, "Echec de l'initialisation du VL53L1X\n");
		return false;
	}

	// Intermediate ROI (8x8, ~20 degrees): compromise between aiming
	// precision and detection reliability if the target moves sideways
	gSensor.setROISize(8, 8);
	gSensor.setROICenter(199);

	gOscillator.setup(context->audioSampleRate, Oscillator::sine);

	// Sized once here, never in render() (no memory allocation in the audio thread)
	tunedKeys.assign(keyValues.size(), 0.0f);

	pinMode(context, 0, distanceEnablePin, INPUT);
	pinMode(context, 0, sineButtonPin, INPUT);
	pinMode(context, 0, triangleButtonPin, INPUT);
	pinMode(context, 0, squareButtonPin, INPUT);
	pinMode(context, 0, sawtoothButtonPin, INPUT);

	gReadSensorTask = Bela_createAuxiliaryTask(readSensorLoop, 50, "read-vl53l1x");
	Bela_scheduleAuxiliaryTask(gReadSensorTask);

	return true;
}

void distanceSensorReadVolume(BelaContext *context, int analogFrameIndex)
{
	gDroneAmplitude = analogRead(context, analogFrameIndex, droneVolumePin); // already between 0 and 1
}

void distanceSensorSetScale(int scale)
{
	if (scale < 0 || scale > 5) return;
	if (scale == scaleChoice) return; // nothing changed, nothing to recompute

	scaleChoice = scale;
	tuningSynthString();
	lastDistanceMM = -2; // forces the frequency to be recomputed
}

float distanceSensorProcessSample(BelaContext *context, int n)
{
	// --- Toggle button debounce ---
	bool rawState = digitalRead(context, n, distanceEnablePin);

	if (rawState == enableCandidateState) {
		enableCandidateCounter++;
	} else {
		enableCandidateState = rawState;
		enableCandidateCounter = 0;
	}

	if (enableCandidateCounter >= debounceSamples && enableCandidateState != enableStableState) {
		previousEnableStableState = enableStableState;
		enableStableState = enableCandidateState;

		// Valid press: toggles enabled / disabled
		if (enableStableState == false && previousEnableStableState == true) {
			gDistanceSensorEnabled = !gDistanceSensorEnabled;
			rt_printf("Capteur de distance : %s\n", gDistanceSensorEnabled ? "active" : "desactive");
		}
	}

	// --- Reading + debounce of the 4 waveform buttons ---
	waveButtonState[0] = digitalRead(context, n, sineButtonPin);
	waveButtonState[1] = digitalRead(context, n, triangleButtonPin);
	waveButtonState[2] = digitalRead(context, n, squareButtonPin);
	waveButtonState[3] = digitalRead(context, n, sawtoothButtonPin);

	for (unsigned int i = 0; i < 4; i++) {
		if (waveButtonState[i] == waveCandidateState[i]) {
			waveCandidateCounter[i]++;
		} else {
			waveCandidateState[i] = waveButtonState[i];
			waveCandidateCounter[i] = 0;
		}

		if (waveCandidateCounter[i] >= debounceSamples && waveCandidateState[i] != waveStableState[i]) {
			previousWaveStableState[i] = waveStableState[i];
			waveStableState[i] = waveCandidateState[i];

			if (waveStableState[i] == false && previousWaveStableState[i] == true) {
				gOscillator.setType(waveType[i]);
				rt_printf("Type d'onde change : %s\n", waveNames[i]);
			}
		}
	}

	if (!gDistanceSensorEnabled) {
		return 0.0f;
	}

	// The frequency is only recomputed when the distance (or the scale) changed
	int distanceMM = gLatestDistanceMM;

	if (distanceMM >= 0 && distanceMM != lastDistanceMM) {
		lastDistanceMM = distanceMM;

		if (scaleChoice > 0) {
			// Scale chosen: snap to the frequency of the closest key
			gFrequency = frequencyForDistance(distanceMM);
		} else {
			// No scale: continuous mapping
			float clamped = distanceMM;
			if (clamped < kMinDistanceMM) clamped = kMinDistanceMM;
			if (clamped > kMaxDistanceMM) clamped = kMaxDistanceMM;

			gFrequency = map(clamped, kMinDistanceMM, kMaxDistanceMM, kMinFrequencyHz, kMaxFrequencyHz);
		}
	}

	return gDroneAmplitude * gOscillator.process(gFrequency);
}

void distanceSensorCleanup()
{
	gSensor.end();
}