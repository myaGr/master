/**
 * todo
 */

#ifndef _GNSS_API_DEF_H_
#define _GNSS_API_DEF_H_

#include <stdio.h>
#include <stdint.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of Measurements in gnss_measurement_callback(). */
#define AGGNSS_MAX_MEASUREMENT   64
/** Maximum number of frequency in each GNSS system that is used in position calculation. */
#define AGGNSS_MAX_FREQ_NUM      2

/**
* Flags to indicate what fields in AGGnss_Measurement are valid.
*/
typedef uint32_t GnssMeasurementFlags_t;
/** A valid 'snr' is stored in the data structure.                      */
#define AGGNSS_MEASUREMENT_HAS_SNR                               (1<<0)
/** A valid 'elevation' is stored in the data structure.                */
#define AGGNSS_MEASUREMENT_HAS_ELEVATION                         (1<<1)
/** A valid 'elevation uncertainty' is stored in the data structure.    */
#define AGGNSS_MEASUREMENT_HAS_ELEVATION_UNCERTAINTY             (1<<2)
/** A valid 'azimuth' is stored in the data structure.                  */
#define AGGNSS_MEASUREMENT_HAS_AZIMUTH                           (1<<3)
/** A valid 'azimuth uncertainty' is stored in the data structure.      */
#define AGGNSS_MEASUREMENT_HAS_AZIMUTH_UNCERTAINTY               (1<<4)
/** A valid 'pseudorange' is stored in the data structure.              */
#define AGGNSS_MEASUREMENT_HAS_PSEUDORANGE                       (1<<5)
/** A valid 'pseudorange uncertainty' is stored in the data structure.  */
#define AGGNSS_MEASUREMENT_HAS_PSEUDORANGE_UNCERTAINTY           (1<<6)
/** A valid 'carrier frequency' is stored in the data structure.        */
#define AGGNSS_MEASUREMENT_HAS_CARRIER_FREQUENCY                 (1<<9)
/** A valid 'carrier cycles' is stored in the data structure.           */
#define AGGNSS_MEASUREMENT_HAS_CARRIER_CYCLES                    (1<<10)
/** A valid 'carrier phase' is stored in the data structure.            */
#define AGGNSS_MEASUREMENT_HAS_CARRIER_PHASE                     (1<<11)
/** A valid 'carrier phase uncertainty' is stored in the data structure. */
#define AGGNSS_MEASUREMENT_HAS_CARRIER_PHASE_UNCERTAINTY         (1<<12)
/** A valid 'doppler shift' is stored in the data structure.            */
#define AGGNSS_MEASUREMENT_HAS_DOPPLER_SHIFT                     (1<<15)
/** A valid 'doppler shift uncertainty' is stored in the data structure. */
#define AGGNSS_MEASUREMENT_HAS_DOPPLER_SHIFT_UNCERTAINTY         (1<<16)
/** A valid 'used in fix' flag is stored in the data structure.         */
#define AGGNSS_MEASUREMENT_HAS_USED_IN_FIX                       (1<<17)


/**
* Constellation type of GnssSvInfo
*/
typedef uint8_t AGGnss_Constellation_t;
#define AGGNSS_CONSTELLATION_UNKNOWN      ((AGGnss_Constellation_t)0)
#define AGGNSS_CONSTELLATION_GPS          ((AGGnss_Constellation_t)1)
#define AGGNSS_CONSTELLATION_SBAS         ((AGGnss_Constellation_t)2)
#define AGGNSS_CONSTELLATION_GLONASS      ((AGGnss_Constellation_t)3)
#define AGGNSS_CONSTELLATION_QZSS         ((AGGnss_Constellation_t)4)
#define AGGNSS_CONSTELLATION_BEIDOU       ((AGGnss_Constellation_t)5)
#define AGGNSS_CONSTELLATION_GALILEO      ((AGGnss_Constellation_t)6)


typedef uint16_t AGGnss_NavigationMessageType_t;
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_UNKNOWN        0
/** GPS L1 C/A message contained in the structure.      */
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_GPS_L1CA       0x0101
/** GPS L2-CNAV message contained in the structure.     */
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_GPS_L2CNAV     0x0102
/** GPS L5-CNAV message contained in the structure.     */
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_GPS_L5CNAV     0x0103
/** GPS CNAV-2 message contained in the structure.      */
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_GPS_CNAV2      0x0104
/** Glonass L1 CA message contained in the structure.   */
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_GLO_L1CA       0x0301
/** Beidou D1 message contained in the structure.       */
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1         0x0501
/** Beidou D2 message contained in the structure.       */
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2         0x0502
/** Galileo I/NAV message contained in the structure.   */
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_GAL_I          0x0601
/** Galileo F/NAV message contained in the structure.   */
#define AGGNSS_NAVIGATION_MESSAGE_TYPE_GAL_F          0x0602

/**
* Flags indicating the GNSS measurement state.
*
* The expected behavior here is for GPS HAL to set all the flags that applies.
* For example, if the state for a satellite is only C/A code locked and bit
* synchronized, and there is still millisecond ambiguity, the state should be
* set as:
*
* GNSS_MEASUREMENT_STATE_CODE_LOCK | GNSS_MEASUREMENT_STATE_BIT_SYNC |
*         GNSS_MEASUREMENT_STATE_MSEC_AMBIGUOUS
*
* If GNSS is still searching for a satellite, the corresponding state should be
* set to GNSS_MEASUREMENT_STATE_UNKNOWN(0).
*/
typedef uint32_t GnssMeasurementState_t;
#define AGGNSS_MEASUREMENT_STATE_UNKNOWN                  (0)
#define AGGNSS_MEASUREMENT_STATE_CODE_LOCK                (1<<0)
#define AGGNSS_MEASUREMENT_STATE_BIT_SYNC                 (1<<1)
#define AGGNSS_MEASUREMENT_STATE_SUBFRAME_SYNC            (1<<2)
#define AGGNSS_MEASUREMENT_STATE_TOW_DECODED              (1<<3)
#define AGGNSS_MEASUREMENT_STATE_MSEC_AMBIGUOUS           (1<<4)
#define AGGNSS_MEASUREMENT_STATE_SYMBOL_SYNC              (1<<5)
#define AGGNSS_MEASUREMENT_STATE_GLO_STRING_SYNC          (1<<6)
#define AGGNSS_MEASUREMENT_STATE_GLO_TOD_DECODED          (1<<7)
#define AGGNSS_MEASUREMENT_STATE_BDS_D2_BIT_SYNC          (1<<8)
#define AGGNSS_MEASUREMENT_STATE_BDS_D2_SUBFRAME_SYNC     (1<<9)
#define AGGNSS_MEASUREMENT_STATE_GAL_E1BC_CODE_LOCK       (1<<10)
#define AGGNSS_MEASUREMENT_STATE_GAL_E1C_2ND_CODE_LOCK    (1<<11)
#define AGGNSS_MEASUREMENT_STATE_GAL_E1B_PAGE_SYNC        (1<<12)
#define AGGNSS_MEASUREMENT_STATE_SBAS_SYNC                (1<<13)
#define AGGNSS_MEASUREMENT_STATE_TOW_KNOWN                (1<<14)
#define AGGNSS_MEASUREMENT_STATE_GLO_TOD_KNOWN            (1<<15)

/**
* Flags indicating the Accumulated Delta Range's states.
*/
typedef uint16_t GnssAccumulatedDeltaRangeState_t;
#define AGGNSS_ADR_STATE_UNKNOWN                      (0)
#define AGGNSS_ADR_STATE_VALID                        (1<<0)
#define AGGNSS_ADR_STATE_RESET                        (1<<1)
#define AGGNSS_ADR_STATE_CYCLE_SLIP                   (1<<2)
#define AGGNSS_ADR_STATE_HALF_CYCLE_RESOLVED          (1<<3)
#define AGGNSS_ADR_STATE_HALF_CYCLE_REPORTED          (1<<4)


/**
* Enumeration of available values for the GNSS Measurement's multipath
* indicator.
*/
typedef uint8_t GnssMultipathIndicator_t;
/** The indicator is not available or unknown.                  */
#define AGGNSS_MULTIPATH_INDICATOR_UNKNOWN                (0)
/** The measurement is indicated to be affected by multipath.   */
#define AGGNSS_MULTIPATH_INDICATOR_PRESENT                (1)
/** The measurement is indicated to be not affected by multipath. */
#define AGGNSS_MULTIPATH_INDICATOR_NOT_PRESENT            (2)

typedef struct {
	/**
    * Time offset at which the measurement was taken in nanoseconds.
    * The reference receiver's time is specified by GpsData::clock::time_ns and should be
    * interpreted in the same way as indicated by GpsClock::type.
    *
    * The sign of time_offset_ns is given by the following equation:
    *      measurement time = GpsClock::time_ns + time_offset_ns
    *
    * It provides an individual time-stamp for the measurement, and allows sub-nanosecond accuracy.
    * This is a mandatory value.
    */
    double time_offset_ns;

    /**
    * Per satellite sync state. It represents the current sync state for the associated satellite.
    * Based on the sync state, the 'received GPS tow' field should be interpreted accordingly.
    *
    * This is a mandatory value.
    */
    GnssMeasurementState_t state;

    /**
    * The received GNSS Time-of-Week at the measurement time, in nanoseconds.
    * Ensure that this field is independent (see comment at top of
    * AGGnss_Measurement struct.)
    *
    * For GPS & QZSS, this is:
    *   Received GPS Time-of-Week at the measurement time, in nanoseconds.
    *   The value is relative to the beginning of the current GPS week.
    *
    *   Given the highest sync state that can be achieved, per each satellite, valid range
    *   for this field can be:
    *     Searching       : [ 0       ]   : GNSS_MEASUREMENT_STATE_UNKNOWN
    *     C/A code lock   : [ 0   1ms ]   : GNSS_MEASUREMENT_STATE_CODE_LOCK is set
    *     Bit sync        : [ 0  20ms ]   : GNSS_MEASUREMENT_STATE_BIT_SYNC is set
    *     Subframe sync   : [ 0    6s ]   : GNSS_MEASUREMENT_STATE_SUBFRAME_SYNC is set
    *     TOW decoded     : [ 0 1week ]   : GNSS_MEASUREMENT_STATE_TOW_DECODED is set
    *
    *   Note well: if there is any ambiguity in integer millisecond,
    *   GNSS_MEASUREMENT_STATE_MSEC_AMBIGUOUS should be set accordingly, in the 'state' field.
    *
    *   This value must be populated if 'state' != GNSS_MEASUREMENT_STATE_UNKNOWN.
    *
    * For Glonass, this is:
    *   Received Glonass time of day, at the measurement time in nanoseconds.
    *
    *   Given the highest sync state that can be achieved, per each satellite, valid range for
    *   this field can be:
    *     Searching       : [ 0       ]   : GNSS_MEASUREMENT_STATE_UNKNOWN
    *     C/A code lock   : [ 0   1ms ]   : GNSS_MEASUREMENT_STATE_CODE_LOCK is set
    *     Symbol sync     : [ 0  10ms ]   : GNSS_MEASUREMENT_STATE_SYMBOL_SYNC is set
    *     Bit sync        : [ 0  20ms ]   : GNSS_MEASUREMENT_STATE_BIT_SYNC is set
    *     String sync     : [ 0    2s ]   : GNSS_MEASUREMENT_STATE_GLO_STRING_SYNC is set
    *     Time of day     : [ 0  1day ]   : GNSS_MEASUREMENT_STATE_GLO_TOD_DECODED is set
    *
    * For Beidou, this is:
    *   Received Beidou time of week, at the measurement time in nanoseconds.
    *
    *   Given the highest sync state that can be achieved, per each satellite, valid range for
    *   this field can be:
    *     Searching    : [ 0       ] : GNSS_MEASUREMENT_STATE_UNKNOWN
    *     C/A code lock: [ 0   1ms ] : GNSS_MEASUREMENT_STATE_CODE_LOCK is set
    *     Bit sync (D2): [ 0   2ms ] : GNSS_MEASUREMENT_STATE_BDS_D2_BIT_SYNC is set
    *     Bit sync (D1): [ 0  20ms ] : GNSS_MEASUREMENT_STATE_BIT_SYNC is set
    *     Subframe (D2): [ 0  0.6s ] : GNSS_MEASUREMENT_STATE_BDS_D2_SUBFRAME_SYNC is set
    *     Subframe (D1): [ 0    6s ] : GNSS_MEASUREMENT_STATE_SUBFRAME_SYNC is set
    *     Time of week : [ 0 1week ] : GNSS_MEASUREMENT_STATE_TOW_DECODED is set
    *
    * For Galileo, this is:
    *   Received Galileo time of week, at the measurement time in nanoseconds.
    *
    *     E1BC code lock   : [ 0   4ms ]   : GNSS_MEASUREMENT_STATE_GAL_E1BC_CODE_LOCK is set
    *     E1C 2nd code lock: [ 0 100ms ]   :
    *     GNSS_MEASUREMENT_STATE_GAL_E1C_2ND_CODE_LOCK is set
    *
    *     E1B page    : [ 0    2s ] : GNSS_MEASUREMENT_STATE_GAL_E1B_PAGE_SYNC is set
    *     Time of week: [ 0 1week ] : GNSS_MEASUREMENT_STATE_TOW_DECODED is set
    *
    * For SBAS, this is:
    *   Received SBAS time, at the measurement time in nanoseconds.
    *
    *   Given the highest sync state that can be achieved, per each satellite,
    *   valid range for this field can be:
    *     Searching    : [ 0     ] : GNSS_MEASUREMENT_STATE_UNKNOWN
    *     C/A code lock: [ 0 1ms ] : GNSS_MEASUREMENT_STATE_CODE_LOCK is set
    *     Symbol sync  : [ 0 2ms ] : GNSS_MEASUREMENT_STATE_SYMBOL_SYNC is set
    *     Message      : [ 0  1s ] : GNSS_MEASUREMENT_STATE_SBAS_SYNC is set
    */
    int64_t received_sv_time_in_ns;

    /**
    * 1-Sigma uncertainty of the Received GPS Time-of-Week in nanoseconds.
    *
    * This value must be populated if 'state' != GPS_MEASUREMENT_STATE_UNKNOWN.
    */
    int64_t received_sv_time_uncertainty_in_ns;

    /**
    * Carrier-to-noise density in dB-Hz, typically in the range [0, 63].
    * It contains the measured C/N0 value for the signal at the antenna port.
    *
    * This is a mandatory value.
    */
    double c_n0_dbhz;

    /**
    * Pseudorange rate at the timestamp in m/s. The correction of a given
    * Pseudorange Rate value includes corrections for receiver and satellite
    * clock frequency errors. Ensure that this field is independent (see
    * comment at top of AGGnss_Measurement struct.)
    *
    * It is mandatory to provide the 'uncorrected' 'pseudorange rate', and provide GpsClock's
    * 'drift' field as well (When providing the uncorrected pseudorange rate, do not apply the
    * corrections described above.)
    *
    * The value includes the 'pseudorange rate uncertainty' in it.
    * A positive 'uncorrected' value indicates that the SV is moving away from the receiver.
    *
    * The sign of the 'uncorrected' 'pseudorange rate' and its relation to the sign of 'doppler
    * shift' is given by the equation:
    *      pseudorange rate = -k * doppler shift   (where k is a constant)
    *
    * This should be the most accurate pseudorange rate available, based on
    * fresh signal measurements from this channel.
    *
    * It is mandatory that this value be provided at typical carrier phase PRR
    * quality (few cm/sec per second of uncertainty, or better) - when signals
    * are sufficiently strong & stable, e.g. signals from a GPS simulator at >=
    * 35 dB-Hz.
    */
    double pseudorange_rate_mps;

    /**
    * 1-Sigma uncertainty of the pseudorange_rate_mps.
    * The uncertainty is represented as an absolute (single sided) value.
    *
    * This is a mandatory value.
    */
    double pseudorange_rate_uncertainty_mps;

    /**
    * Accumulated delta range's state. It indicates whether ADR is reset or there is a cycle slip
    * (indicating loss of lock).
    *
    * This is a mandatory value.
    */
    GnssAccumulatedDeltaRangeState_t accumulated_delta_range_state;

    /**
    * Accumulated delta range since the last channel reset in meters.
    * A positive value indicates that the SV is moving away from the receiver.
    *
    * The sign of the 'accumulated delta range' and its relation to the sign of 'carrier phase'
    * is given by the equation:
    *          accumulated delta range = -k * carrier phase    (where k is a constant)
    *
    * This value must be populated if 'accumulated delta range state' != GPS_ADR_STATE_UNKNOWN.
    * However, it is expected that the data is only accurate when:
    *      'accumulated delta range state' == GPS_ADR_STATE_VALID.
    */
    double accumulated_delta_range_m;

    /**
    * 1-Sigma uncertainty of the accumulated delta range in meters.
    * This value must be populated if 'accumulated delta range state' != GPS_ADR_STATE_UNKNOWN.
    */
    double accumulated_delta_range_uncertainty_m;

    /**
    * Best derived Pseudorange by the chip-set, in meters.
    * The value contains the 'pseudorange uncertainty' in it.
    *
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_PSEUDORANGE.
    */
    double pseudorange_m;

    /**
    * 1-Sigma uncertainty of the pseudorange in meters.
    * The value contains the 'pseudorange' and 'clock' uncertainty in it.
    * The uncertainty is represented as an absolute (single sided) value.
    *
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_PSEUDORANGE_UNCERTAINTY.
    */
    double pseudorange_uncertainty_m;

    /**
    * Carrier frequency at which codes and messages are modulated, it can be L1 or L2.
    * If the field is not set, the carrier frequency is assumed to be L1.
    *
    * If the data is available, 'flags' must contain
    * GNSS_MEASUREMENT_HAS_CARRIER_FREQUENCY.
    */
    float carrier_frequency_hz;

    /**
    * The number of full carrier cycles between the satellite and the receiver.
    * The reference frequency is given by the field 'carrier_frequency_hz'.
    * Indications of possible cycle slips and resets in the accumulation of
    * this value can be inferred from the accumulated_delta_range_state flags.
    *
    * If the data is available, 'flags' must contain
    * GNSS_MEASUREMENT_HAS_CARRIER_CYCLES.
    */
    int64_t carrier_cycles;

    /**
    * The RF phase detected by the receiver, in the range [0.0, 1.0].
    * This is usually the fractional part of the complete carrier phase measurement.
    *
    * The reference frequency is given by the field 'carrier_frequency_hz'.
    * The value contains the 'carrier-phase uncertainty' in it.
    *
    * If the data is available, 'flags' must contain
    * GNSS_MEASUREMENT_HAS_CARRIER_PHASE.
    */
    double carrier_phase;

    /**
    * 1-Sigma uncertainty of the carrier-phase.
    * If the data is available, 'flags' must contain
    * GNSS_MEASUREMENT_HAS_CARRIER_PHASE_UNCERTAINTY.
    */
    double carrier_phase_uncertainty;

    /**
    * Doppler shift in Hz.
    * A positive value indicates that the SV is moving toward the receiver.
    *
    * The reference frequency is given by the field 'carrier_frequency_hz'.
    * The value contains the 'doppler shift uncertainty' in it.
    *
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_DOPPLER_SHIFT.
    */
    double doppler_shift_hz;

    /**
    * 1-Sigma uncertainty of the doppler shift in Hz.
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_DOPPLER_SHIFT_UNCERTAINTY.
    */
    double doppler_shift_uncertainty_hz;

    /**
    * An enumeration that indicates the 'multipath' state of the event.
    *
    * The multipath Indicator is intended to report the presence of overlapping
    * signals that manifest as distorted correlation peaks.
    *
    * - if there is a distorted correlation peak shape, report that multipath
    *   is GNSS_MULTIPATH_INDICATOR_PRESENT.
    * - if there is not a distorted correlation peak shape, report
    *   GNSS_MULTIPATH_INDICATOR_NOT_PRESENT
    * - if signals are too weak to discern this information, report
    *   GNSS_MULTIPATH_INDICATOR_UNKNOWN
    *
    * Example: when doing the standardized overlapping Multipath Performance
    * test (3GPP TS 34.171) the Multipath indicator should report
    * GNSS_MULTIPATH_INDICATOR_PRESENT for those signals that are tracked, and
    * contain multipath, and GNSS_MULTIPATH_INDICATOR_NOT_PRESENT for those
    * signals that are tracked and do not contain multipath.
    */
    GnssMultipathIndicator_t multipath_indicator;

    /**
    * Signal-to-noise ratio at correlator output in dB.
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_SNR.
    * This is the power ratio of the "correlation peak height above the
    * observed noise floor" to "the noise RMS".
    */
    double snr_db;

	/**
	* Loss of lock indicator.
	* 0-7bit: original LLI
	* 8-31: other tracking status
	*/
	int32_t LLI;

} GnssChannelMeas_t;

/**
* Flags to indicate what fields in AGGnss_Clock are valid.
*/
typedef uint16_t GnssClockFlags_t;
/** A valid 'leap second' is stored in the data structure.      */
#define AGGNSS_CLOCK_HAS_LEAP_SECOND               (1<<0)
/** A valid 'time uncertainty' is stored in the data structure. */
#define AGGNSS_CLOCK_HAS_TIME_UNCERTAINTY          (1<<1)
/** A valid 'full bias' is stored in the data structure.        */
#define AGGNSS_CLOCK_HAS_FULL_BIAS                 (1<<2)
/** A valid 'bias' is stored in the data structure. */
#define AGGNSS_CLOCK_HAS_BIAS                      (1<<3)
/** A valid 'bias uncertainty' is stored in the data structure. */
#define AGGNSS_CLOCK_HAS_BIAS_UNCERTAINTY          (1<<4)
/** A valid 'drift' is stored in the data structure. */
#define AGGNSS_CLOCK_HAS_DRIFT                     (1<<5)
/** A valid 'drift uncertainty' is stored in the data structure. */
#define AGGNSS_CLOCK_HAS_DRIFT_UNCERTAINTY         (1<<6)

/**
* Represents an estimate of the GPS clock time.
*/
typedef struct {
    /** set to sizeof(AGGnss_Clock) */
    uint64_t size;

    /**
    * A set of flags indicating the validity of the fields in this data
    * structure.
    */
    GnssClockFlags_t flags;

    /**
    * Leap second data.
    * The sign of the value is defined by the following equation:
    *      utc_time_ns = time_ns - (full_bias_ns + bias_ns) - leap_second *
    *      1,000,000,000
    *
    * If the data is available 'flags' must contain GNSS_CLOCK_HAS_LEAP_SECOND.
    */
    int16_t leap_second;

    /**
    * The GNSS receiver internal clock value. This is the local hardware clock
    * value.
    *
    * For local hardware clock, this value is expected to be monotonically
    * increasing while the hardware clock remains power on. (For the case of a
    * HW clock that is not continuously on, see the
    * hw_clock_discontinuity_count field). The receiver's estimate of GPS time
    * can be derived by substracting the sum of full_bias_ns and bias_ns (when
    * available) from this value.
    *
    * This GPS time is expected to be the best estimate of current GPS time
    * that GNSS receiver can achieve.
    *
    * Sub-nanosecond accuracy can be provided by means of the 'bias_ns' field.
    * The value contains the 'time uncertainty' in it.
    *
    * This field is mandatory.
    */
    int64_t time_ns;

    /**
    * 1-Sigma uncertainty associated with the clock's time in nanoseconds.
    * The uncertainty is represented as an absolute (single sided) value.
    *
    * If the data is available, 'flags' must contain
    * GNSS_CLOCK_HAS_TIME_UNCERTAINTY. This value is effectively zero (it is
    * the reference local clock, by which all other times and time
    * uncertainties are measured.)  (And thus this field can be not provided,
    * per GNSS_CLOCK_HAS_TIME_UNCERTAINTY flag, or provided & set to 0.)
    */
    double time_uncertainty_ns;

    /**
    * The difference between hardware clock ('time' field) inside GPS receiver
    * and the true GPS time since 0000Z, January 6, 1980, in nanoseconds.
    *
    * The sign of the value is defined by the following equation:
    *      local estimate of GPS time = time_ns - (full_bias_ns + bias_ns)
    *
    * This value is mandatory if the receiver has estimated GPS time. If the
    * computed time is for a non-GPS constellation, the time offset of that
    * constellation to GPS has to be applied to fill this value. The error
    * estimate for the sum of this and the bias_ns is the bias_uncertainty_ns,
    * and the caller is responsible for using this uncertainty (it can be very
    * large before the GPS time has been solved for.) If the data is available
    * 'flags' must contain GNSS_CLOCK_HAS_FULL_BIAS.
    */
    int64_t full_bias_ns;

    /**
    * Sub-nanosecond bias.
    * The error estimate for the sum of this and the full_bias_ns is the
    * bias_uncertainty_ns
    *
    * If the data is available 'flags' must contain GNSS_CLOCK_HAS_BIAS. If GPS
    * has computed a position fix. This value is mandatory if the receiver has
    * estimated GPS time.
    */
    double bias_ns;

    /**
    * 1-Sigma uncertainty associated with the local estimate of GPS time (clock
    * bias) in nanoseconds. The uncertainty is represented as an absolute
    * (single sided) value.
    *
    * If the data is available 'flags' must contain
    * GNSS_CLOCK_HAS_BIAS_UNCERTAINTY. This value is mandatory if the receiver
    * has estimated GPS time.
    */
    double bias_uncertainty_ns;

    /**
    * The clock's drift in nanoseconds (per second).
    *
    * A positive value means that the frequency is higher than the nominal
    * frequency, and that the (full_bias_ns + bias_ns) is growing more positive
    * over time.
    *
    * The value contains the 'drift uncertainty' in it.
    * If the data is available 'flags' must contain GNSS_CLOCK_HAS_DRIFT.
    *
    * This value is mandatory if the receiver has estimated GNSS time
    */
    double drift_nsps;

    /**
    * 1-Sigma uncertainty associated with the clock's drift in nanoseconds (per second).
    * The uncertainty is represented as an absolute (single sided) value.
    *
    * If the data is available 'flags' must contain
    * GNSS_CLOCK_HAS_DRIFT_UNCERTAINTY. If GPS has computed a position fix this
    * field is mandatory and must be populated.
    */
    double drift_uncertainty_nsps;

    /**
    * When there are any discontinuities in the HW clock, this field is
    * mandatory.
    *
    * A "discontinuity" is meant to cover the case of a switch from one source
    * of clock to another.  A single free-running crystal oscillator (XO)
    * should generally not have any discontinuities, and this can be set and
    * left at 0.
    *
    * If, however, the time_ns value (HW clock) is derived from a composite of
    * sources, that is not as smooth as a typical XO, or is otherwise stopped &
    * restarted, then this value shall be incremented each time a discontinuity
    * occurs.  (E.g. this value may start at zero at device boot-up and
    * increment each time there is a change in clock continuity. In the
    * unlikely event that this value reaches full scale, rollover (not
    * clamping) is required, such that this value continues to change, during
    * subsequent discontinuity events.)
    *
    * While this number stays the same, between AGGnss_Clock reports, it can be
    * safely assumed that the time_ns value has been running continuously, e.g.
    * derived from a single, high quality clock (XO like, or better, that's
    * typically used during continuous GNSS signal sampling.)
    *
    * It is expected, esp. during periods where there are few GNSS signals
    * available, that the HW clock be discontinuity-free as long as possible,
    * as this avoids the need to use (waste) a GNSS measurement to fully
    * re-solve for the GPS clock bias and drift, when using the accompanying
    * measurements, from consecutive AGGnss_Data reports.
    */
    uint32_t hw_clock_discontinuity_count;

    /**
	* system time tag, millisecond since 1970/1/1 locally.
	*/
    int64_t time_stamp;

} AGGnss_Clock_t;

/**
* Represents a GNSS Measurement, it contains raw and computed information.
*
* Independence - All signal measurement information (e.g. sv_time,
* pseudorange_rate, multipath_indicator) reported in this struct should be
* based on GNSS signal measurements only. You may not synthesize measurements
* by calculating or reporting expected measurements based on known or estimated
* position, velocity, or time.
*/
typedef struct {
    /** set to sizeof(GpsMeasurement) */
    uint64_t size;

    /** A set of flags indicating the validity of the fields in this data structure. */
    GnssMeasurementFlags_t flags;

    /**
    * Pseudo-random number for the SV, or FCN/OSN number for Glonass. The
    * distinction is made by looking at constellation field. Values should be
    * in the range of:
    *
    * - GPS:     1-32
    * - SBAS:    120-151, 183-192
    * - GLONASS: 1-24, the orbital slot number (OSN), if known.  Or, if not:
    *            93-106, the frequency channel number (FCN) (-7 to +6) offset by + 100
    *            i.e. report an FCN of -7 as 93, FCN of 0 as 100, and FCN of +6 as 106.
    * - QZSS:    193-200
    * - Galileo: 1-36
    * - Beidou:  1-37
    */
    int16_t svid;

    /**
    * Defines the constellation of the given SV. Value should be one of those
    * GNSS_CONSTELLATION_* constants
    */
    AGGnss_Constellation_t constellation;

	/**
	* A set of bit-mask indicating the validity of parameter group of each 
	* frequency point. From low bit to high, corresponding to different GNSS system,
	* each bit relates to L1, L2, L5 from GPS, or L1, L2, L3 from GLONASS, and so on.
	*/
	uint8_t gnss_frequency_mask;


    /**
    * Elevation in degrees, the valid range is [-90, 90].
    * The value contains the 'elevation uncertainty' in it.
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_ELEVATION.
    */
    double elevation_deg;

    /**
    * 1-Sigma uncertainty of the elevation in degrees, the valid range is [0, 90].
    * The uncertainty is represented as the absolute (single sided) value.
    *
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_ELEVATION_UNCERTAINTY.
    */
    double elevation_uncertainty_deg;

    /**
    * Azimuth in degrees, in the range [0, 360).
    * The value contains the 'azimuth uncertainty' in it.
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_AZIMUTH.
    *  */
    double azimuth_deg;

    /**
    * 1-Sigma uncertainty of the azimuth in degrees, the valid range is [0, 180].
    * The uncertainty is represented as an absolute (single sided) value.
    *
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_AZIMUTH_UNCERTAINTY.
    */
    double azimuth_uncertainty_deg;

    /**
    * Whether the GNSS represented by the measurement was used for computing the most recent fix.
    * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_USED_IN_FIX.
    */
#ifdef _WIN32
    BOOLEAN used_in_fix;
#else
    bool used_in_fix;
#endif
	/**
	* 3-element array containing groups of paramters, each of which is specific to a GNSS
	* frequency point. The validity of each group is masked by bit field in
	* gnss_frequency_mask.
	*/
    GnssChannelMeas_t channel_meas[AGGNSS_MAX_FREQ_NUM];

	/**
	* reserved1[0]: CODE_XXX type, 8bit*3freq
	* reserved1[1-3]: locktime in ms, 3 freq, [1] as freq1
	* 
	*/
    int32_t reserved1[4];
	/**
	* For ST chiptype, reserved2[0]:iono correction,  reserved2[1]:trop correction
	*/
    double  reserved2[4];
} AGGnss_Measurement_t;


/**
* Represents a reading of GNSS measurements. For devices where GnssSystemInfo's
* year_of_hw is set to 2016+, it is mandatory that these be provided, on
* request, when the GNSS receiver is searching/tracking signals.
*
* - Reporting of GPS constellation measurements is mandatory.
* - Reporting of all tracked constellations are encouraged.
*/
typedef struct {
    /** set to sizeof(AGGnss_Data) */
    uint64_t size;

    /** Number of measurements. */
    uint64_t measurement_count;

    /** The array of measurements. */
    AGGnss_Measurement_t measurements[AGGNSS_MAX_MEASUREMENT];

    /** The GPS clock time reading. */
    AGGnss_Clock_t clock;
} AGGnss_Data_t;

/**
 * Status of Navigation Message
 * When a message is received properly without any parity error in its navigation words, the
 * status should be set to NAV_MESSAGE_STATUS_PARITY_PASSED. But if a message is received
 * with words that failed parity check, but GPS is able to correct those words, the status
 * should be set to NAV_MESSAGE_STATUS_PARITY_REBUILT.
 * No need to send any navigation message that contains words with parity error and cannot be
 * corrected.
 */
typedef uint16_t NavigationMessageStatus_t;
#define AGGNSS_NAV_MESSAGE_STATUS_UNKONW           (0)
#define AGGNSS_NAV_MESSAGE_STATUS_PARITY_PASSED    (1<<0)
#define AGGNSS_NAV_MESSAGE_STATUS_PARITY_REBUILT   (1<<1)

/** Represents a GPS navigation message (or a fragment of it). */
typedef struct {
    /** set to sizeof(AGGnss_NavigationMessage) */
    uint64_t size;

    /**
    * Pseudo-random number for the SV, or FCN/OSN number for Glonass. The
    * distinction is made by looking at constellation field. Values should be
    * in the range of:
    *
    * - GPS:     1-32
    * - SBAS:    120-151, 183-192
    * - GLONASS: 1-24, the orbital slot number (OSN), if known.  Or, if not:
    *            93-106, the frequency channel number (FCN) (-7 to +6) offset by + 100
    *            i.e. report an FCN of -7 as 93, FCN of 0 as 100, and FCN of +6 as 106.
    * - QZSS:    193-200
    * - Galileo: 1-36
    * - Beidou:  1-37
    */
    int16_t svid;

    /**
    * The type of message contained in the structure.
    * This is a Mandatory value.
    */
    AGGnss_NavigationMessageType_t type;

    /**
    * The status of the received navigation message.
    * No need to send any navigation message that contains words with parity error and cannot be
    * corrected.
    */
    NavigationMessageStatus_t status;

    /**
    * Message identifier.
    * It provides an index so the complete Navigation Message can be assembled. i.e. fo L1 C/A
    * subframe 4 and 5, this value corresponds to the 'frame id' of the navigation message.
    * Subframe 1, 2, 3 does not contain a 'frame id' and this value can be set to -1.
    */
    int16_t message_id;

    /**
    * Sub-message identifier.
    * If required by the message 'type', this value contains a sub-index within the current
    * message (or frame) that is being transmitted.
    * i.e. for L1 C/A the submessage id corresponds to the sub-frame id of the navigation message.
    */
    int16_t submessage_id;

    /**
    * The length of the data (in bytes) contained in the current message.
    * If this value is different from zero, 'data' must point to an array of the same size.
    * e.g. for L1 C/A the size of the sub-frame will be 40 bytes (10 words, 30 bits/word).
    *
    * This is a Mandatory value.
    */
    uint64_t data_length;

    /**
    * The data of the reported GPS message.
    * The bytes (or words) specified using big endian format (MSB first).
    *
    * For L1 C/A, each subframe contains 10 30-bit GPS words. Each GPS word (30 bits) should be
    * fitted into the last 30 bits in a 4-byte word (skip B31 and B32), with MSB first.
    */
    uint8_t data[64];

} AGGnss_NavigationMessage_t;

/** Maximum number of Navigation models in gnss_nav_model_callback(). */
#define AGGNSS_MAX_NAVIGATION_MODEL   64


/**
* Represents GPS Navigation model info
* Refer to ICD-200 for the description of each element.
*/
typedef struct {
    uint8_t     codeL2;            /* Code on L2 flag                        */
    uint8_t     URA;               /* SF1 raw accuracy factor                */
    uint8_t     SV_health;         /* SV health byte                         */
    uint8_t     L2Pdata;           /* L2-P data flag                         */
    uint8_t     fit_interval;      /* As in ICD-200                          */
    uint8_t     IODE;              /* As in ICD-200                          */
    uint16_t    weeknum;           /* GPS week number                        */
    uint16_t    IODC;              /* IODC -- 10 LSBs                        */
    float       SVacc;             /* SV accuracy; meters                    */
    float       T_GD;              /* Group delay time factor; seconds       */
    float       t_oc;              /* Time of this block's applicability;    */
    float       a_f2;              /* SV clock coef2; sec/sec^2              */
    float       a_f1;              /* SV clock coef1; sec/sec                */
    float       a_f0;              /* SV clock coef0; sec                    */
    float       C_uc;              /* Radians                                */
    float       C_us;              /* Radians                                */
    float       t_oe;              /* Seconds                                */
    float       C_ic;              /* Radians                                */
    float       C_is;              /* Radians                                */
    float       C_rc;              /* Meters                                 */
    float       C_rs;              /* Meters                                 */
    double      delta_n;           /* Radians/sec                            */
    double      M_0;               /* Radians                                */
    double      e;                 /* Dimensionless                          */
    double      sqrt_A;            /* Meters**-1/2                           */
    double      OMEGA_0;           /* Radians                                */
    double      i_0;               /* Radians                                */
    double      omega;             /* Radians                                */
    double      OMEGADOT;          /* Radians                                */
    double      IDOT;              /* Radians                                */
}GpsNaviModel_t;


/**
* Represents Glonass Navigation model info
* 
*/
typedef struct
{
    uint8_t     FT;                /*User Range Accuracy index.  P32 ICD Glonass for value of Ft.*/
    uint8_t     M;                 /*Glonass vehicle type. M=1 means type M*/
    uint8_t     Bn;                /*Bn SV health see p30 ICD Glonass. */
    uint8_t     utc_offset;        /*Current GPS-UTC leap seconds [sec]; 0 if unknown. */
    uint8_t     En;                /*Age of current information in days */
    uint8_t     P1;                /*Time interval between adjacent values of tb in minutes */
    uint8_t     P2;                /*1 if tb is odd and 0 if tb is even */
    double      deltaTau;          /*time difference between transmission in G2 and G1*/
    double      gamma;             /*SV clock frequency error ratio  */
    double      freqno;            /*Freq slot: -7 to +13 incl. */
    double      lsx;               /* x luni solar accel  */
    double      lsy;               /* y luni solar accel  */
    double      lsz;               /*z luni solar accel  */
    double      tauN;              /*SV clock bias scale factor 2^-30 [seconds]. */
    double      gloSec;            /*gloSec=[(N4-1)*1461 + (NT-1)]*86400 + tb*900 */
    double      x;                 /*x position at toe  */
    double      y;                 /*y position at toe  */
    double      z;                 /*z position at toe */
    double      vx;                /* x velocity at toe  */
    double      vy;                /* y velocity at toe  */
    double      vz;                /*z velocity at toe  */
    double      tauGPS;
} GlonassNaviModel_t;


/**
* Represents Beidou Navigation model info
* 
*/
typedef struct
{
    uint8_t     AODE;              /*Age of Data, Ephemeris*/
    uint8_t     AODC;              /*Age of Data, Clock (AODC)*/
    uint8_t     URAI;              /*URA Index, URA is used to describe the signal-in-space accuracy in meters*/
    uint32_t    weeknum;           /* BDS week number */
    float       t_oc;              /* Time of this block's applicability;    */
    float       t_oe;              /* Ephemeris reference time  */
    float       C_uc;              /* Radians                                */
    float       C_us;              /* Radians                                */
    float       C_ic;              /* Radians                                */
    float       C_is;              /* Radians                                */
    float       C_rc;              /* Meters                                 */
    float       C_rs;              /* Meters                                 */
    double      sqrt_A;            /* Meters**-1/2                           */
    double      e;                 /* Dimensionless                          */
    double      omega;             /* Radians                                */
    double      delta_n;           /* Radians/sec                            */
    double      M_0;               /* Radians                                */
    double      OMEGA_0;           /* Radians                                */
    double      OMEGADOT;          /* Radians                                */
    double      i_0;               /* Radians                                */
    double      IDOT;              /* Radians                                */
    double      a_0;               /*Clock correction polynomial coefficient(seconds) */
    double      a_1;               /*Clock correction polynomial coefficient (sec/sec) */
    double      a_2;               /*Clock correction polynomial coefficient (sec/sec2) */
    double      T_GD_1;            /*Equipment group delay differential TGD1 (seconds) */
	uint8_t     SV_health;         /*health status of this Ephemeris*/
} BdsNaviModel_t;

/**
* Represents Galileo Navigation model info
*
*/
typedef struct
{
	int8_t   E1BDVS;  /* 1bit, Data Validity Status */
	int8_t   E1BSHS;  /* 2bit, Signal Health Status */
	uint8_t  SV_health;  /* health status of this Ephemeris */
	uint16_t IODnav;  /* issue of data  */
	uint32_t weeknum; /* week number */
	/* orbit */
	float    t_oe;    /* Ephemeris reference time  */
	double   omega;   /* Radians                                */
	double   DeltaN;  /* Radians/sec                            */
	double   M_0;     /* Radians                                */
	double   OMEGADOT;/* Radians                                */
	double   e;       /* Dimensionless                          */
	double   IDOT;    /* Radians                                */
	double   sqrt_A;  /* Meters**-1/2                           */
	double   i_0;     /* Radians                                */
	double   OMEGA_0; /* Radians                                */
	float    C_rs;    /* Meters                                 */
	float    C_is;    /* Radians                                */
	float    C_us;    /* Radians                                */
	float    C_rc;    /* Meters                                 */
	float    C_ic;    /* Radians                                */
	float    C_uc;    /* Radians                                */
	/* clock */
	float    t_oc;   /* Time of this block's applicability;    */
	float    a_f2;   /*Clock correction polynomial coefficient (sec/sec2) */
	double   a_f1;   /*Clock correction polynomial coefficient (sec/sec) */
	double   a_f0;   /*Clock correction polynomial coefficient(seconds) */
	double   bgd;    /*Equipment group delay */
	uint8_t	 sisa;   /*signal in space accuracy */
	uint8_t	 stanModelID;	/* OPTIONAL, 0:INAV  1:FNAV, default 0 */
}GalNaviModel_t;

/** Represents a GNSS navigation model message. */
typedef struct {
    /** set to sizeof(GnssNavigationModelMessage_t) */
    uint64_t size;

    /**
    * Pseudo-random number for the SV, or FCN/OSN number for Glonass. The
    * distinction is made by looking at constellation field. Values should be
    * in the range of:
    *
    * - GPS:     1-32
    * - GLONASS: 1-24, the orbital slot number (OSN), if known.  Or, if not:
    *            93-106, the frequency channel number (FCN) (-7 to +6) offset by + 100
    *            i.e. report an FCN of -7 as 93, FCN of 0 as 100, and FCN of +6 as 106.
    * - Beidou:  1-37
    */
    int16_t svid;

    /**
    * Defines the constellation of the given SV. Value should be one of those
    * GNSS_CONSTELLATION_* constants
    */
    AGGnss_Constellation_t constellation;
    union{
        GpsNaviModel_t       gps_eph;
        GlonassNaviModel_t   gln_eph;
        BdsNaviModel_t       bds_eph;
        GalNaviModel_t       gal_eph;
    }choice;

} GnssNavigationModelMessage_t;

/** Represents a GNSS navigation model message. */
typedef struct {
    /** set to sizeof(GnssNavigationModelMessageV2_t) */
    uint64_t size;

    /**
    * Pseudo-random number for the SV, or FCN/OSN number for Glonass. The
    * distinction is made by looking at constellation field. Values should be
    * in the range of:
    *
    * - GPS:     1-32
    * - GLONASS: 1-24, the orbital slot number (OSN), if known.  Or, if not:
    *            93-106, the frequency channel number (FCN) (-7 to +6) offset by + 100
    *            i.e. report an FCN of -7 as 93, FCN of 0 as 100, and FCN of +6 as 106.
    * - Beidou:  1-37
    */
    int16_t svid;

    /**
    * Defines the constellation of the given SV. Value should be one of those
    * GNSS_CONSTELLATION_* constants
    */
    AGGnss_Constellation_t constellation;
    union{
        GpsNaviModel_t       gps_eph;
        GlonassNaviModel_t   gln_eph;
        BdsNaviModel_t       bds_eph;
        GalNaviModel_t       gal_eph;
    }choice;

    union {
        int64_t    array[4];
    } reserved_int;

    union {
        struct {
            double bds_eph_tgd2; /* Equipment group delay differential T_GD_2 (seconds) */
        } bds_eph_ext;
        double     array[4];
    } reserved_dbl;

} GnssNavigationModelMessageV2_t;

typedef struct
{
    /** set to sizeof(GnssNavigationModelData_t) */    
    uint64_t size; 

    /** Number of satellite navigation models. */    
    uint64_t navigation_model_count; 

    /** The array of navigation models. */    
    GnssNavigationModelMessage_t navigation_models[AGGNSS_MAX_MEASUREMENT];
}GnssNavigationModelData_t;

typedef struct
{
    /** set to sizeof(GnssNavigationModelDataV2_t) */    
    uint64_t size; 

    /** Number of satellite navigation models. */    
    uint64_t navigation_model_count; 

    /** The array of navigation models. */    
    GnssNavigationModelMessageV2_t navigation_models[AGGNSS_MAX_MEASUREMENT];
}GnssNavigationModelDataV2_t;

typedef struct {
    AGGnss_Constellation_t  constellation;
    float                  alfa0;
    float                  alfa1;
    float                  alfa2;
    float                  alfa3;
    float                  beta0;
    float                  beta1;
    float                  beta2;
    float                  beta3;
} GnssIonosphericModelMessage_t;

/*fusion type*/
typedef uint8_t GnssFusionType;
#define FUSION_TYPE_INVALID             0
#define FUSION_TYPE_GNSSONLY            1
#define FUSION_TYPE_DGNSS               2
#define FUSION_TYPE_FUSION              3   /* duplicated. */
#define FUSION_TYPE_DR                  4
#define FUSION_TYPE_CHIP                5
#define FUSION_TYPE_CHIP_FUSION         6
#define FUSION_TYPE_GO_FUSION           8	/* fusion with standalone pvt result. */
#define FUSION_TYPE_DGNSS_FUSION        9
#define FUSION_TYPE_RTK_FIX             10
#define FUSION_TYPE_RTK_FLOAT           11
#define FUSION_TYPE_RTK_PROPAGATE       12
#define FUSION_TYPE_RTK_FIX_FUSION      13
#define FUSION_TYPE_RTK_FLOAT_FUSION    14

/* the type of input GNSS solution. */
typedef struct {
    uint16_t is_use_sv_num_valid : 1;
    uint16_t is_hor_accuracy_valid : 1;
    uint16_t is_hor_velocity_valid : 1;
    uint16_t is_heading_valid : 1;
    uint16_t is_avg_cn0_valid : 1;
    uint16_t is_utc_valid : 1;
	uint16_t is_systime_valid : 1;
	uint16_t is_ecef_pos_valid : 1;
	uint16_t is_ecef_vel_valid : 1;
	uint16_t is_dop_valid : 1;
	uint16_t is_pos_err_valid : 1;
	uint16_t is_vel_err_valid : 1;
    uint16_t : 4;
} gsol_validity_t;

typedef uint8_t dop_valid_flags;
#define DOP_VALID_FLAG_PDOP    ((uint8_t)0x01)
#define DOP_VALID_FLAG_HDOP    ((uint8_t)0x02)
#define DOP_VALID_FLAG_VDOP    ((uint8_t)0x04)
#define DOP_VALID_FLAG_TDOP    ((uint8_t)0x08)

typedef struct
{
	dop_valid_flags valid_flag;
	float pdop;
	float hdop;
	float vdop;
	float tdop;
} raw_gnss_dop_t;

typedef struct
{
	gsol_validity_t    validity_mask;
	GnssFusionType     fusion_type;
	uint8_t            used_sv_num;
	float              hor_accuracy;      /* horizontal accuracy, in m. */
	float              hor_velocity;      /* horizontal velocity, in m/s. */
	float              heading;           /* in degree. */
	float              avg_cn0;           /* average CN0, in  */
	uint64_t           utc_timestamp_ms;
	uint64_t           sys_timestamp_ms;
	double             pos_ecef[3];       /* ECEF position, in meter. */
	float              vel_ecef[3];       /* ECEF velocity, in meter/s. */
	float              pos_err_lla[3];    /* latitude, longitude and altitude standard error. */
	float              vel_err_lla[3];    /* E/N/U velocity standard error. */
	raw_gnss_dop_t     dops;
} raw_gnss_sol_t;


typedef struct
{
	uint64_t       utc_timestamp_ms;
	uint8_t        used_sv_num;
	uint8_t        gps_L1_num;
	uint8_t        gps_L2_num;
	uint8_t        gps_L5_num;
	uint8_t        qzs_L1_num;
	uint8_t        qzs_L2_num;
	uint8_t        qzs_L5_num;
	uint8_t		   gps_num;
	uint8_t		   qzss_num;
	uint8_t        gln_num;
	uint8_t        gal_num;
	uint8_t        bds_num;
	uint16_t       avg_cn0;
	float          cycle_clip_ratio;
	float          hdop;
	float          pdop;
} unc_scene_t;

typedef struct
{
	/* local time */
	uint64_t            timestamp;
	/* the number of accumulated raw data */
	uint32_t            raw_data_num;
	/* x, y, z */
	float               raw_data[3];
} f3axis_t;

typedef struct {
	f3axis_t acc_params;
} fa_param_t;

typedef struct {
	f3axis_t gyro_params;
} fg_param_t;

typedef struct {
	f3axis_t mag_params;
} fm_param_t;

typedef struct
{
	/* local time */
	uint64_t            timestamp;
	float               data;
} fb_param_t;

typedef struct
{
	/* local time */
	uint64_t            timeStamp;
	/* speed in m/s */
	float               speed;
} fo_param_t;

typedef struct
{
	uint64_t           timestamp;
	uint32_t           ms_running;
	uint8_t            left_rear_valid;
	uint8_t            right_rear_valid;
	uint8_t            left_front_valid;
	uint8_t            right_front_valid;
	union left_rear_t {
		float          speed;
		uint32_t       pulse_count;
	} lr;
	union right_rear_t {
		float          speed;
		uint32_t       pulse_count;
	} rr;
	union left_front_t {
		float          speed;
		uint32_t       pulse_count;
	} lf;
	union right_front_t {
		float          speed;
		uint32_t       pulse_count;
	} rf;
} fw_t;

typedef uint16_t fi_type_t;
#define FI_TYPE_ACC     ((fi_type_t)0x0001)
#define FI_TYPE_GYRO    ((fi_type_t)0x0002)
#define FI_TYPE_MAG     ((fi_type_t)0x0004)
#define FI_TYPE_BARO    ((fi_type_t)0x0008)
#define FI_TYPE_ODO     ((fi_type_t)0x0010)
#define FI_TYPE_WT      ((fi_type_t)0x0020)

typedef uint8_t fs_format_t;
#define FS_FORMAT_SAPERATE       1
#define FS_FORMAT_PACKAGE        2
#define FS_FORMAT_PPS            3

typedef struct {
	fi_type_t         flags;
	fa_param_t        acc;
	fg_param_t        gyro;
	fm_param_t        mag;
	fb_param_t        baro;
	fo_param_t        odo;
	fw_t              wheeltick;
} fs_params_t;

#define MAX_SENSOR_UNIT_CNT    15
typedef struct {
	/* local time stamp. */
	uint64_t               timestamp;
	/* number of valid unit in pack. */
	uint32_t               param_cnt;
	/* sensor data package. */
	fs_params_t    param_pack[MAX_SENSOR_UNIT_CNT];
} fs_pack_t;

typedef struct {
	/* 1: valid; 0: invalid */
	uint32_t            valid;
	int64_t             pps_timestamp;
} fs_pps_t;

typedef enum {
	PVT_FROM_RE = 0,
	PVT_FROM_DE = 1,
	PVT_FROM_GNSS = 2,
	PVT_FROM_FE = 3,
	PVT_SRC_MAX = 4
} PvtSrc_t;

/** GpsLocation has valid latitude and longitude. */
#define AGGNSS_LOCATION_HAS_LAT_LONG   0x0001
/** GpsLocation has valid altitude. */
#define AGGNSS_LOCATION_HAS_ALTITUDE   0x0002
/** GpsLocation has valid speed. */
#define AGGNSS_LOCATION_HAS_SPEED      0x0004
/** GpsLocation has valid bearing. */
#define AGGNSS_LOCATION_HAS_BEARING    0x0008
/** GpsLocation has valid accuracy. */
#define AGGNSS_LOCATION_HAS_ACCURACY   0x0010

/** DeviceID Length */
#define AGGNSS_DEVICE_LENGTH   128

/* Milliseconds since January 1, 1970 */
typedef uint64_t GnssUtcTime_t;

typedef enum
{
    AGGNSS_STATUS_UNKNOWN = 0,
    AGGNSS_STATUS_SYSTEM_FAILURE = 1001,
    AGGNSS_STATUS_NETWORK_UNAVAILABLE = 1002,
    AGGNSS_STATUS_CONFIG_ERROR = 1003,

    AGGNSS_STATUS_RTD_INIT_SUCCESS = 2000,
    AGGNSS_STATUS_RTD_INIT_FAILED = 2001,
    AGGNSS_STATUS_RTD_START_SUCCESS = 2002,
    AGGNSS_STATUS_RTD_START_FAILED = 2003,
    AGGNSS_STATUS_RTD_STOP_SUCCESS = 2004,
    AGGNSS_STATUS_RTD_STOP_FAILED = 2005,
    AGGNSS_STATUS_RTD_RUNTIME_ERROR = 2006,

    AGGNSS_STATUS_AGNSS_REQUEST_SUCCESS = 5000,
    AGGNSS_STATUS_AGNSS_REQUEST_FAILED = 5001
} AGGnss_SdkStatus_t;

typedef uint16_t ag_gnss_ading_data_t;
#define AGGNSS_DELETE_EPHEMERIS        0x0001
#define AGGNSS_DELETE_ALMANAC          0x0002
#define AGGNSS_DELETE_POSITION         0x0004
#define AGGNSS_DELETE_TIME             0x0008
#define AGGNSS_DELETE_IONO             0x0010
#define AGGNSS_DELETE_UTC              0x0020
#define AGGNSS_DELETE_HEALTH           0x0040
#define AGGNSS_DELETE_SVDIR            0x0080
#define AGGNSS_DELETE_SVSTEER          0x0100
#define AGGNSS_DELETE_SADATA           0x0200
#define AGGNSS_DELETE_RTI              0x0400
#define AGGNSS_DELETE_CELLDB_INFO      0x8000
#define AGGNSS_DELETE_ALL              0xFFFF

typedef enum {
	LOG_TYPE_INVALID       = 0,
	LOG_TYPE_DEBUG         = 1,
	LOG_TYPE_PB_BIN        = 2,
	LOG_TYPE_PB_TXT        = 3,
	LOG_TYPE_NMEA_PE       = 4,
	LOG_TYPE_NMEA_HW       = 5,
	LOG_TYPE_NMEA_FE       = 6,
	LOG_TYPE_NMEA_DR       = 7,
	LOG_TYPE_SERVICE_DEBUG = 20,
	LOG_TYPE_DR_DEBUG      = 21
} ag_gnss_log_t;

typedef enum
{
  LOG_NONE = 0,
  LOG_DEBUG,
  LOG_INFO,
  LOG_WARNING,
  LOG_ERROR,
} Log_Level_t;

typedef struct {
	char gyro_type[128];
	char acce_type[128];
	char magn_type[128];
	char press_type[128];
	char odom_type[128];
	char temp_type[128];
	char ospd_type[128];
} qxdr_param_t;

typedef uint8_t   gnss_chip_type_t;
#define GNSS_CHIP_TYPE_UNKNOWN     ((gnss_chip_type_t)0)
/* 1~5 are deprecated. Keep them to be compatible with old cfg. */
#define GNSS_CHIP_TYPE_SPRD        ((gnss_chip_type_t)1)
#define GNSS_CHIP_TYPE_QCOM        ((gnss_chip_type_t)2)
#define GNSS_CHIP_TYPE_MTK         ((gnss_chip_type_t)3)
//#define GNSS_CHIP_TYPE_BRCM        ((gnss_chip_type_t)4)
#define GNSS_CHIP_TYPE_UBLOX       ((gnss_chip_type_t)5)
#define GNSS_CHIP_TYPE_SPRD_       ((gnss_chip_type_t)10)
#define GNSS_CHIP_TYPE_QCOM_SF     ((gnss_chip_type_t)20)
#define GNSS_CHIP_TYPE_QCOM_DF     ((gnss_chip_type_t)21)
#define GNSS_CHIP_TYPE_BRCM_47755  ((gnss_chip_type_t)22)
#define GNSS_CHIP_TYPE_MTK_        ((gnss_chip_type_t)30)
#define GNSS_CHIP_TYPE_UBLOX_M8    ((gnss_chip_type_t)50)
#define GNSS_CHIP_TYPE_UBLOX_F9    ((gnss_chip_type_t)51)
#define GNSS_CHIP_TYPE_ST_8090     ((gnss_chip_type_t)60)
#define GNSS_CHIP_TYPE_ST_8100     ((gnss_chip_type_t)61)
#define GNSS_CHIP_TYPE_MXT_900B    ((gnss_chip_type_t)70)
#define GNSS_CHIP_TYPE_MXT_900D    ((gnss_chip_type_t)71)

typedef uint8_t gnss_meas_type_t;
#define GNSS_MEAS_TYPE_UNKNOWN     ((gnss_meas_type_t)0)
#define GNSS_MEAS_TYPE_SPRD        ((gnss_meas_type_t)1)
#define GNSS_MEAS_TYPE_QCOM        ((gnss_meas_type_t)2)
#define GNSS_MEAS_TYPE_MTK         ((gnss_meas_type_t)3)
#define GNSS_MEAS_TYPE_BRCM        ((gnss_meas_type_t)4)
#define GNSS_MEAS_TYPE_UBLOX       ((gnss_meas_type_t)5)

#define NO_CHIP_CFG    GNSS_MEAS_TYPE_UNKNOWN
#define SPRD           GNSS_MEAS_TYPE_SPRD
#define QCOM           GNSS_MEAS_TYPE_QCOM
#define MTK            GNSS_MEAS_TYPE_MTK
#define BRCM           GNSS_MEAS_TYPE_BRCM
#define UBLOX          GNSS_MEAS_TYPE_UBLOX

// the specific sub-chiptype, used inside algorithm
typedef enum _sub_chiptype_t {
	SUB_CHIPTYPE_ANYTYPE       = 0,
	SUB_CHIPTYPE_UBLOX_M8      = 1,
	SUB_CHIPTYPE_UBLOX_F9      = 2,
	SUB_CHIPTYPE_ST_8090       = 3,
	SUB_CHIPTYPE_ST_8100       = 4,
	SUB_CHIPTYPE_MXT_900B      = 5,
	SUB_CHIPTYPE_MXT_900D      = 6,
	SUB_CHIPTYPE_QCOM_SF       = 7,
	SUB_CHIPTYPE_QCOM_DF       = 8,
	SUB_CHIPTYPE_BRCM_47755    = 9,
	SUB_CHIPTYPE_HD9310        = 10,
	SUB_CHIPTYPE_HISI1103      = 11,
	SUB_CHIPTYPE_QCOM_855      = 12
} sub_chiptype_e;

typedef enum _serial_data_type_t {
    SERIAL_DATA_TYPE_UBLOX = 1,
    SERIAL_DATA_TYPE_SINAN = 2,
    SERIAL_DATA_TYPE_ST = 3,
} serial_data_type_t;

typedef uint8_t   meas_type_mask_t;
#define MEAS_TYPE_MASK_PR      ((meas_type_mask_t)0x1)
#define MEAS_TYPE_MASK_DR      ((meas_type_mask_t)0x2)
#define MEAS_TYPE_MASK_CP      ((meas_type_mask_t)0x4)
#define MEAS_TYPE_MASK_SNR     ((meas_type_mask_t)0x8)

/* Application scenarios */
typedef uint8_t   algo_scenario_t;
#define APPLY_SCENE_ANYWHERE     ((algo_scenario_t)0)   /* for anywhere */
#define APPLY_SCENE_AUTOROOF     ((algo_scenario_t)1)   /* on the roof of auto */
#define APPLY_SCENE_AUTOINSIDE   ((algo_scenario_t)2)   /* inside the auto */
#define APPLY_SCENE_BIKE         ((algo_scenario_t)3)   /* share bike */
#define APPLY_SCENE_CAMERA       ((algo_scenario_t)4)   /* monitor camera */
#define APPLY_SCENE_WATCH        ((algo_scenario_t)5)   /* watch */
#define APPLY_SCENE_DRONE        ((algo_scenario_t)6)   /* drone */
#define APPLY_SCENE_AERO         ((algo_scenario_t)7)   /* aeroplane */


typedef uint8_t rtk_pos_mode_t;
#define RTK_POS_MODE_SINGLE          0
#define RTK_POS_MODE_DGPS            1
#define RTK_POS_MODE_KINEMATIC       2
#define RTK_POS_MODE_STATIC          3
#define RTK_POS_MODE_MOVINGBASE      4
#define RTK_POS_MODE_FIXED           5
#define RTK_POS_MODE_PPP_KINE        6
#define RTK_POS_MODE_PPP_STATIC      7
#define RTK_POS_MODE_PPP_FIXED       8

typedef uint8_t rtk_ar_mode_t;
#define RTK_ARMODE_OFF               0
#define RTK_ARMODE_CONTINUOUS        1
#define RTK_ARMODE_INSTANTANEOUS     2
#define RTK_ARMODE_FIX_HOLD          3

typedef uint8_t rtk_nav_sys_mask_t;
#define RTK_NAV_MASK_BIT_GPS         ((rtk_nav_sys_mask_t)0x01)
#define RTK_NAV_MASK_BIT_SBAS        ((rtk_nav_sys_mask_t)0x02)
#define RTK_NAV_MASK_BIT_GLN         ((rtk_nav_sys_mask_t)0x04)
#define RTK_NAV_MASK_BIT_GAL         ((rtk_nav_sys_mask_t)0x08)
#define RTK_NAV_MASK_BIT_QZSS        ((rtk_nav_sys_mask_t)0x10)
#define RTK_NAV_MASK_BIT_BDS         ((rtk_nav_sys_mask_t)0x20)

// rcv/board type of RINEX file
typedef uint8_t rinex_data_type_t;
#define RINEX_DATA_TYPE_NORMAL      ((rinex_data_type_t)0)
#define RINEX_DATA_TYPE_UM4B0       ((rinex_data_type_t)1)

typedef struct
{
	uint8_t                dynamics;        /* only used in one place? */
	gnss_chip_type_t       chip_type;
	uint8_t                is_automobile;	/* mobile phone(0) or mobile(1). */
	algo_scenario_t        applyScenario;   /* flag to indicate what scenario the rcv antena is in */
	uint8_t                rtd_usage_flag;  /* obsolete: bit0: gps, bit1:glonass, bit2:beidou */
	uint8_t                gnss_usage_flag; /* #bit0:gps, bit1:gln, bit2:bds, bit3:gal */
	meas_type_mask_t       meas_type_mask;
	uint16_t               cno_mask;
	uint16_t               start_mode;      /* 0:cold start, 1:hot start */
	double                 altLowLimit;
	double                 altHighLimit;
	double                 pdop_mask;
	double                 pos_res_thres;
	double                 ele_mask;
	double                 codeStdPhone;
	double                 codeStdAuto;
	double                 carrStdPhone;
	double                 carrStdAuto;
} pe_algo_cfg_t;

typedef struct
{
	/* nav_sys_* flag indicates which navigation system is used. */
	uint8_t          nav_sys_gps : 1;
	uint8_t          nav_sys_sbas : 1;
	uint8_t          nav_sys_glonass : 1;
	uint8_t          nav_sys_galileo : 1;
	uint8_t          nav_sys_qzss : 1;
	uint8_t          nav_sys_compass : 1;
	uint8_t          enable_first_static : 1;/* 0:off, 1:on */
	uint8_t          enable_rtd : 1;/* 0:off,1:on */
	uint8_t          bds_armode : 1;         /* 0:off, 1:on */
	uint8_t          out_rinex_file : 1;     /* 0:off, 1:on */
	uint8_t          debug_level;            /* 0:off,1:lev1,2:lev2,3:lev3,4:lev4,5:lev5 */
	uint8_t          freq_combine;           /* 1:l1,2:l1+l2,3:l1+l2+l5,4:l1+l5 */
	uint8_t          process_type;           /* 0:forward,1:backward,2:combined */
	uint8_t          glo_armode;             /* 0:off,1:on,2:auto cal,3:ext cal */
	rtk_pos_mode_t   pos_mode;
	rtk_ar_mode_t    ar_mode;
	int32_t          ar_lock_cnt;
	int32_t          ar_min_fix;
	int32_t          ar_out_cnt;
	double           ar_elevation_mask;     /* deg */
	double           ar_threshold;
	double           elevation_mask_hold;   /* deg */
	double           elevation_mask;        /* deg */
	double           max_age;
	double           rej_thres_innovation;
	double           err_ratio_1;
	double           err_ratio_2;
	double           err_phase_factor_a;
	double           err_phase_factor_b;
	double           err_phase_factor_c;
	double           err_phase_doppler_freq;
	double           init_state_std_bias;
	double           proc_noise_std_bias;
} rtk_algo_cfg_t;

/* FLP config items */
typedef enum
{
	FLP_DEVICE_UNKNOWN,
	FLP_DEVICE_M8L,
	FLP_DEVICE_MC120A,
	FLP_DEVICE_HW
} flp_device_t;

typedef enum
{
	FLP_MODE_VDR = 0,
	FLP_MODE_PDR = 1,
	FLP_MODE_AHRS = 2,
	FLP_MODE_PT = 3
} flp_mode_t;

typedef struct
{
	uint16_t      input_types;
	uint16_t      imu_freq;
	uint16_t      odo_freq;
	uint16_t      output_freq;
	uint32_t      need_mis_angle_est;
	flp_device_t  dev_type;
	flp_mode_t    algo_mode;
} flp_algo_cfg_t;

typedef struct
{
	pe_algo_cfg_t          pe_cfg;
	rtk_algo_cfg_t         rtk_cfg;
	flp_algo_cfg_t         flp_cfg;
} algo_cfg_t;

typedef struct _sbas_msg_t {
    int32_t  week;  
    double   tow;   
    int32_t  prn;          /* SBAS PRN */
    uint32_t word[8];      /* 250 valid bits + 6bits padding */
} sbas_msg_t;

typedef float pos_report_freq_t;
#define REPORT_FREQ_0_1_HZ   0.1f
#define REPORT_FREQ_0_2_HZ   0.2f
#define REPORT_FREQ_1_HZ     1.0f
#define REPORT_FREQ_5_HZ     5.0f
#define REPORT_FREQ_10_HZ    10.0f

typedef struct
{
	pos_report_freq_t rpt_freq;
	uint32_t          enable_ag_algo : 1;
} ag_pos_mode_t;

#define GNSS_DEVICE_LEN       128
#define APPKEY_LEN            32
#define APPSECRET_LEN         128

typedef struct {
	uint8_t id_type; // 0:UNKNOWN 1:IMSI 2:MSISDN 3:IMEI
	char    id[GNSS_DEVICE_LEN];
	char    device_type[GNSS_DEVICE_LEN];
	char    appkey[APPKEY_LEN];
	char    appsecret[APPSECRET_LEN];
} ag_sdk_params_t;

#ifdef __cplusplus
}
#endif

#endif /** end of _GNSS_API_DEF_H_ */

