/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFT_SSR2LOS_SRC_INTEGRITY_INTEGRITY_PROVIDER_H
#define SWIFT_SSR2LOS_SRC_INTEGRITY_INTEGRITY_PROVIDER_H

#include <swift/inputs.h>

#include <internal_types/atmo.h>
#include <internal_types/common.h>
#include <internal_types/flags.h>

#include <cstdint>

namespace swift {
namespace internal {

struct Chain {
  uint8_t id = 0;
  bool is_set = false;
};

struct ReceivedChains {
  Chain chain1 = {0, false};
  Chain chain2 = {1, false};
};

/**
 * @brief Flags area that holds flags that have been complete.
 *
 * @details Flags in this area will be filled with flag messages that have been
 * fully received and all sub-parts for each message have been combined.
 * However, the flags area as a whole could still be incomplete, i.e. not all
 * flag message types have been received for the latest stored epoch.
 */
struct CompleteFlagsArea {
  FlagsHighLevel flags_high_level;
  std::array<FlagsSatellites, cMaxConstellations> flags_satellites_set;
  FlagsGridPoints flags_tropo_grid_points;
  FlagsGridPoints flags_iono_grid_points;
  FlagsIonoTileLos flags_iono_tile_los;
  FlagsIonoGridLos flags_iono_grid_los;

  ReceivedChains high_level_received_chains;
  std::array<ReceivedChains,
             static_cast<std::size_t>(LowLvlFlagIdx::LOW_LVL_FLAGS_COUNT)>
      low_level_received_chains{};
};

class IntegrityProvider {
 public:
  IntegrityProvider();
  explicit IntegrityProvider(const ChainIds &supported_chain_ids,
                             const bool enable_flags_checking);
  virtual ~IntegrityProvider() = default;

  enum class ReturnCode : uint8_t { OK, NOT_OK, DISABLED };

  /**
   * @brief Enable the integrity provider.
   *
   * @details Enable the integrity provider so that it checks flags that are
   * added to it.
   */
  void enable();

  /**
   * @brief Disable the integrity provider.
   *
   * @details If disabled it will always be ready (i.e. is_ready returns true)
   * and all flags are considered nominal (i.e. is_*_integrity_nominal return
   * true). This can also be set through the constructor by setting
   * enable_flags_checking to false
   */
  void disable();

  /**
   * @brief Check if the integrity provider is enabled.
   *
   * @return true if integrity provider is enabled.
   */
  bool is_enabled() const;

  /**
   * @brief Add High level flags.
   *
   * @details Adds a MSG_SSR_FLAG_HIGH_LEVEL message.
   *
   * @param[in] incoming_flags High level flags to add.
   */
  void add_flags_high_level(const FlagsHighLevel &incoming_flags);

  /**
   * @brief Add Satellite flags.
   *
   * @details Adds a MSG_SSR_FLAG_SATELLITES message.
   *
   * @param[in] incoming_flags Satellite flags to add.
   */
  void add_flags_satellites(const FlagsSatellites &incoming_flags);

  /**
   * @brief Add Tropo grid points flags.
   *
   * @details Adds a MSG_SSR_FLAG_TROPO_GRID_POINTS message.
   *
   * @param[in] incoming_flags Grid points flags to add.
   */
  void add_flags_tropo_grid_points(const FlagsGridPoints &incoming_flags);

  /**
   * @brief Add Iono grid points flags.
   *
   * @details Adds a MSG_SSR_FLAG_IONO_GRID_POINTS message.
   *
   * @param[in] incoming_flags Grid points flags to add.
   */
  void add_flags_iono_grid_points(const FlagsGridPoints &incoming_flags);

  /**
   * @brief Add Iono tile los flags.
   *
   * @details Adds a MSG_SSR_FLAG_IONO_TILE_SAT_LOS message.
   *
   * @param[in] incoming_flags Iono tile los flags to add.
   */
  void add_flags_iono_tile_los(const FlagsIonoTileLos &incoming_flags);

  /**
   * @brief Add Iono grid point sat los flags.
   *
   * @details Adds a MSG_SSR_FLAG_IONO_GRID_POINT_SAT_LOS message.
   *
   * @param[in] incoming_flags Iono grid point sat los flags to add.
   */
  void add_flags_iono_grid_los(const FlagsIonoGridLos &incoming_flags);

  /**
   * @brief Find out if Integrity provider is ready.
   *
   * @details This method will return true under the following conditions:
   *     - Flags area with matching tile_set_id and tile_id exists.
   *     - coherent_timestamp is earlier or equal to High level flags Correction
   *       epoch.
   *     - High level flags have been received from the two supported Chain IDs.
   *     - Low level flags have been received from Chain IDs that have a high
   *       level flag warning for an associated flag.
   *     - Low level flag(s) associated with the warning(s) has the same
   *       Observation timestamp, SSR solution ID, Tile Set ID and Tile ID as
   *       the High level flags.
   *
   * @param[in] coherent_timestamp Latest SSR correction timestamp of the
   * coherent set that should be integrity checked.
   * @param[in] tile_set_id Tile set ID for which the check should apply to.
   * @param[in] tile_id Tile ID for which the check should apply to.
   *
   * @return OK when a complete set of flags exists, DISABLED if flags checking
   * is disabled, NOT_OK otherwise.
   */
  virtual ReturnCode is_ready(const Timestamp &coherent_timestamp,
                              const uint16_t tile_set_id,
                              const uint16_t tile_id) const;

  /**
   * @brief Find out if Integrity provider is ready for satellite corrections
   * purpose.
   *
   * @details This method will return OK if there exists a flags area for
   * whichever tile such that:
   *     - coherent_timestamp is earlier or equal to High level flags Correction
   *       epoch.
   *     - High level flags have been received from the two supported Chain IDs.
   *     - Low level flags have been received from Chain IDs that have a high
   *       level flag warning for an associated flag.
   *     - Input solution ID is the same as the one in the high level flags
   *
   * @param[in] coherent_timestamp Latest SSR correction timestamp of the
   * coherent set that should be integrity checked.
   * @param[in] solution_id Solution ID of the SSR corrections.
   *
   * @return OK when a complete set of flags exists, DISABLED if flags checking
   * is disabled, NOT_OK otherwise.
   */
  virtual IntegrityProvider::ReturnCode is_sat_ready(
      const Timestamp &coherent_timestamp, const uint8_t solution_id) const;

  /**
   * @brief Find out if integrity is nominal for a specific satellite.
   *
   * @details Returns true if code bias, phase bias, clock and orbit corrections
   * is nominal for the given Timestamp, Solution ID and Satellite ID.
   *
   * @param[in] coherent_timestamp Latest SSR correction timestamp of the
   * coherent set that should be integrity checked.
   * @param[in] solution_id Solution ID of the SSR corrections.
   * @param[in] sat_id Satellite constellation and number to check integrity
   * for.
   *
   * @return OK if integrity is nominal, DISABLED if flags checking is disabled,
   * NOT_OK otherwise.
   */
  virtual ReturnCode is_sat_integrity_nominal(
      const Timestamp &coherent_timestamp, const uint8_t solution_id,
      const SatelliteDescription &sat_id) const;

  /**
   * @brief Find out if integrity is nominal for a tropospheric grid locality.
   *
   * @details Returns true if tropospheric corrections' integrity is nominal for
   * the given Timestamp and Grid locality.
   *
   * @param[in] coherent_timestamp Latest SSR correction timestamp of the
   * coherent set that should be integrity checked.
   * @param[in] grid_locality Grid locality to be checked.
   *
   * @return OK if integrity is nominal, DISABLED if flags checking is disabled,
   * NOT_OK otherwise.
   */
  virtual ReturnCode is_tropo_integrity_nominal(
      const Timestamp &coherent_timestamp,
      const GridLocality &grid_locality) const;

  /**
   * @brief Find out if integrity is nominal for an ionospheric grid locality
   * and satellite.
   *
   * @details Returns true if ionospheric corrections' is nominal for the given
   * Timestamp, Grid locality and Satellite ID.
   *
   * @param[in] coherent_timestamp Latest SSR correction timestamp of the
   * coherent set that should be integrity checked.
   * @param[in] grid_locality Grid locality to be checked.
   * @param[in] sat_id Satellite constellation and number to check integrity
   * for.
   *
   * @return OK if integrity is nominal, DISABLED if flags checking is disabled,
   * NOT_OK otherwise.
   */
  virtual ReturnCode is_iono_integrity_nominal(
      const Timestamp &coherent_timestamp, const GridLocality &grid_locality,
      const SatelliteDescription &sat_id) const;

  /**
   * @brief Get integrity time for the latest valid High level flags.
   *
   * @param[in] coherent_timestamp Latest SSR correction timestamp of the
   * coherent set that should be integrity checked.
   * @param[in] solution_id Solution ID of the SSR corrections.
   *
   * @param[out] timestamp Pointer to timestamp to write integrity time to.
   *
   * @return OK if matching high level flags exist, DISABLED if flags checking
   * is disabled, NOT_OK otherwise.
   */
  virtual ReturnCode get_integrity_time(const Timestamp &coherent_timestamp,
                                        const uint8_t solution_id,
                                        Timestamp *timestamp) const;

  /**
   * @brief Get integrity time for the latest valid High level flags for a
   * specific tile.
   *
   * @param[in] coherent_timestamp Latest SSR correction timestamp of the
   * coherent set that should be integrity checked.
   * @param[in] grid_id Grid ID to fetch the integrity time from.
   * @param[out] timestamp Pointer to timestamp to write integrity time to.
   *
   * @return OK if a matching tile exist, DISABLED if flags checking is
   * disabled, NOT_OK otherwise.
   */
  virtual ReturnCode get_integrity_time(const Timestamp &coherent_timestamp,
                                        const GridId &grid_id,
                                        Timestamp *timestamp) const;

 private:
  // Need to support up to cMaxTiles, which means that we have to buffer up to
  // cMaxTiles flags areas.
  StaticBuffer<CompleteFlagsArea, cMaxTiles> flags_areas_;

  // Nominal flags from two different Chain IDs are needed in order to consider
  // a correction to be nominal. But only the two supported Chain IDs will be
  // considered. Flags from other Chain IDs will be ignored.
  ChainIds supported_chain_ids_;

  bool enable_flags_checking_;

  bool get_latest_valid_flags_area_idx(const Timestamp &coherent_timestamp,
                                       const uint8_t solution_id,
                                       std::size_t *idx) const;

  bool get_atmo_flags_area_idx(const Timestamp &coherent_timestamp,
                               const GridLocality &grid_locality,
                               std::size_t *flags_area_idx) const;
};

/**
 * @brief Checks if grid point is nominal
 * @param grid_idx Index of the grid.
 * @param flags_iono_grid_los Iono grid point Los low level flags.
 * @param sat_id Id of the satellite.
 * @return True if grid point is nominal, flase otherwise.
 */
bool is_grid_point_nominal(const std::size_t grid_idx,
                           const FlagsIonoGridLos &flags_iono_grid_los,
                           const SatelliteDescription &sat_id);

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTEGRITY_INTEGRITY_PROVIDER_H
