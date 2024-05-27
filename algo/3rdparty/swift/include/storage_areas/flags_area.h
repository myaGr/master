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

#ifndef SWIFT_SSR2LOS_SRC_STORAGE_AREAS_FLAGS_AREA_H
#define SWIFT_SSR2LOS_SRC_STORAGE_AREAS_FLAGS_AREA_H

#include <swift/config.h>
#include <swift/outputs.h>

#include <integrity/integrity_provider.h>
#include <internal_types/flags.h>
#include <internal_types/static_buffer.h>

#include <array>
#include <cstdint>

namespace swift {
namespace internal {

enum class FlagTypeIdx : uint8_t {
  HIGH_LEVEL,
  SATELLITES,
  TROPO_GRID_POINTS,
  IONO_GRID_POINTS,
  IONO_TILE_SAT_LOS,
  IONO_GRID_SAT_LOS
};

struct TileFlags {
  Timestamp obs_timestamp;
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;
  FlagsHighLevel flags_high_level;
  FlagsGridPoints flags_tropo_grid_points;
  FlagsGridPoints flags_iono_grid_points;
  FlagsIonoTileLos flags_iono_tile_sat_los;
  FlagsIonoGridLos flags_iono_grid_point_sat_los;
};

constexpr std::size_t cFlagsSatellitesStorageSize = 3;
struct FlagsArea {
  StaticBuffer<TileFlags, cMaxTiles> tile_flags;

  StaticBuffer<FlagsSatellites, cFlagsSatellitesStorageSize>
      flags_satellites_gps;
  StaticBuffer<FlagsSatellites, cFlagsSatellitesStorageSize>
      flags_satellites_gal;
  StaticBuffer<FlagsSatellites, cFlagsSatellitesStorageSize>
      flags_satellites_bds;
};

/**
 * @brief Store high level flags in flags area.
 *
 * @details Store high level flags in flags area and update the integrity
 * provider. If any matching and complete low level flag exist in the flags area
 * the integrity provider will be updated with those messages as well.
 *
 * @param flags_high_level[in] Flags to be stored.
 * @param flags_area[in/out] Pointer to flags area where flags will be stored.
 * @param integrity_provider[in/out] Pointer to integrity provider to be
 * updated.
 */
void store_flags_high_level(const FlagsHighLevel &flags_high_level,
                            FlagsArea *flags_area,
                            IntegrityProvider *integrity_provider);

/**
 * @brief Store satellite flags in flags area.
 *
 * @details Store satellite flags in flags area and combine them with any
 * stored, matching and incomplete flags messages of the same type. If satellite
 * flags messages are complete and a matching high level flags message exist,
 * the integrity provider will be updated with the combined satellites flags
 * message.
 *
 * @param flags_satellites[in] Flags to be stored.
 * @param flags_area[in/out] Pointer to flags area where flags will be stored.
 * @param integrity_provider[in/out] Pointer to integrity provider to be
 * updated.
 */
void store_flags_satellites(const FlagsSatellites &flags_satellites,
                            FlagsArea *flags_area,
                            IntegrityProvider *integrity_provider);

/**
 * @brief Store tropo grid points flags in flags area.
 *
 * @details Store tropo grid points flags in flags area and combine them with
 * any stored, matching and incomplete flags messages of the same type. If
 * tropo grid points flags messages are complete and a matching high level flags
 * message exist, the integrity provider will be updated with the combined
 * tropo grid points flags message.
 *
 * @param flags_tropo_grid[in] Flags to be stored.
 * @param flags_area[in/out] Pointer to flags area where flags will be stored.
 * @param integrity_provider[in/out] Pointer to integrity provider to be
 * updated.
 */
void store_flags_tropo_grid_points(const FlagsGridPoints &flags_tropo_grid,
                                   FlagsArea *flags_area,
                                   IntegrityProvider *integrity_provider);

/**
 * @brief Store iono grid points flags in flags area.
 *
 * @details Store iono grid points flags in flags area and combine them with
 * any stored, matching and incomplete flags messages of the same type. If
 * iono grid points flags messages are complete and a matching high level flags
 * message exist, the integrity provider will be updated with the combined
 * iono grid points flags message.
 *
 * @param flags_iono_grid[in] Flags to be stored.
 * @param flags_area[in/out] Pointer to flags area where flags will be stored.
 * @param integrity_provider[in/out] Pointer to integrity provider to be
 * updated.
 */
void store_flags_iono_grid_points(const FlagsGridPoints &flags_iono_grid,
                                  FlagsArea *flags_area,
                                  IntegrityProvider *integrity_provider);

/**
 * @brief Store iono tile sat los flags in flags area.
 *
 * @details Store iono tile sat los flags in flags area and combine them with
 * any stored, matching and incomplete flags messages of the same type. If
 * iono tile sat los flags messages are complete and a matching high level flags
 * message exist, the integrity provider will be updated with the combined
 * iono tile sat los flags message.
 *
 * @param flags_iono_tile_los[in] Flags to be stored.
 * @param flags_area[in/out] Pointer to flags area where flags will be stored.
 * @param integrity_provider[in/out] Pointer to integrity provider to be
 * updated.
 */
void store_flags_iono_tile_sat_los(const FlagsIonoTileLos &flags_iono_tile_los,
                                   FlagsArea *flags_area,
                                   IntegrityProvider *integrity_provider);

/**
 * @brief Store iono grid point sat los flags in flags area.
 *
 * @details Store iono grid point sat los flags in flags area and combine them
 * with any stored, matching and incomplete flags messages of the same type. If
 * iono grid point sat los flags messages are complete and a matching high level
 * flags message exist, the integrity provider will be updated with the combined
 * iono grid point sat los flags message.
 *
 * @param flags_iono_grid_los[in] Flags to be stored.
 * @param flags_area[in/out] Pointer to flags area where flags will be stored.
 * @param integrity_provider[in/out] Pointer to integrity provider to be
 * updated.
 */
void store_flags_iono_grid_point_sat_los(
    const FlagsIonoGridPointLos &flags_iono_grid_los, FlagsArea *flags_area,
    IntegrityProvider *integrity_provider);

/*
 * @brief Populate a high level flags status bitfield from the outgoing
 *        satellite corrections
 *
 * @details The output is a bitfield with the following layout:
 *   Bits 0-1: GPS
 *   Bits 2-3: GAL
 *   Bits 6-7: BDS
 *   Bits 8-9: Atmo
 *
 * The satellite data in the sat_corrections array are used to populate
 * the high level flags output.
 * If any satellites are non-nominal, the output bits for that constellation
 * will be the worst case of any present satellite, in the order:
 *    Alert -> Not monitored -> Warning -> Nominal
 *
 * @param[in] sat_corrections the satellite corrections array with the low level
 * flags set in the flags bitfield
 *
 * @return The bitfield of merged flags as per above.
 */
uint16_t create_high_level_flags_output(
    const std::array<SatelliteCorrection, cMaxSats> &sat_corrections);

/**
 * @brief Merge two flags depending on their value
 * @param flag_1 First flag
 * @param flag_2 Second flag
 * @return Merged flag
 */
UsageFlag merge_single_flags(const UsageFlag flag_1, const UsageFlag flag_2);

/**
 * @brief Convert usage flag to bietfield
 * @param flag Flag to convert
 * @return Bitfield corresponding to the usage flag
 */
uint16_t usage_flag_to_bitfield(const internal::UsageFlag &flag);

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_STORAGE_AREAS_FLAGS_AREA_H
