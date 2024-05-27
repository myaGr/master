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

#ifndef SWIFT_SSR2LOS_SRC_STORAGE_AREAS_HOLDING_AREA_H
#define SWIFT_SSR2LOS_SRC_STORAGE_AREAS_HOLDING_AREA_H

#include <cstdint>

#include <swift/outputs.h>

#include <internal_types/atmo.h>
#include <internal_types/code_bias.h>
#include <internal_types/common.h>
#include <internal_types/orbit_clock.h>
#include <internal_types/phase_bias.h>
#include <internal_types/satellite_apc.h>
#include <internal_types/satellite_bounds.h>
#include <internal_types/static_buffer.h>
#include <internal_types/tile_definition.h>

namespace swift {
namespace internal {

constexpr uint8_t cOrbitClockStorageSize = 6;
constexpr uint8_t cCodeBiasStorageSize = 2;
constexpr uint8_t cPhaseBiasStorageSize = 2;
constexpr uint8_t cTileDefinitionStorageSize = cMaxTiles;
constexpr uint8_t cAtmoGridStorageSize = 2;
constexpr uint8_t cStecPolynomialStorageSize = 2;
constexpr uint8_t cBoundsStorageSize = 2;
constexpr uint8_t cSatelliteApcStorageSize = 2;

struct HoldingArea {
  StaticBuffer<SsrOrbitClockCollection, cOrbitClockStorageSize> orbits_clocks;
  StaticBuffer<SsrOrbitClockBoundsCollection, cBoundsStorageSize>
      orbit_clock_bounds;
  StaticBuffer<SsrOrbitClockBoundsDegradationCollection, cBoundsStorageSize>
      orbit_clock_degradations;

  StaticBuffer<SsrCodeBiasesCollection, cCodeBiasStorageSize> code_biases;
  StaticBuffer<SsrCodeAndPhaseBiasBoundsCollection, cBoundsStorageSize>
      code_and_phase_bias_bounds;
  StaticBuffer<SsrPhaseBiasesCollection, cPhaseBiasStorageSize> phase_biases;
  StaticBuffer<SsrSatelliteApcCollection, cSatelliteApcStorageSize>
      satellite_apc;

  StaticBuffer<TileDefinitionCollection, cTileDefinitionStorageSize>
      tile_definitions;
  StaticBuffer<AtmoGrid, cAtmoGridStorageSize> atmo_grids;
  StaticBuffer<StecPolynomialSatCollection, cStecPolynomialStorageSize> stecs;
};

/**
 * @brief Stores an incoming orbit clock collection into the holding area.
 * @details If an existing collection has a matching timestamp and collection Id
 * when comparing to the incoming collection and has its complete flag set to
 * false, that collection is instead updated. Otherwise the incoming collection
 * gets inserted into the orbit clock storage.
 *
 * @param collection The incoming collection to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the orbit clock storage that was
 *                      updated.
 * @return Success if the incoming collection was stored and had the complete
 *                 flag set to true.
 *         Corrections unavailable if the incoming collection was stored and had
 *                                the complete flag set to false.
 *         Broken sequence if the storing of the incoming collection failed.
 */
ReturnCode store_orbit_clock(const SsrOrbitClockCollection &collection,
                             HoldingArea *holding_area,
                             std::size_t *updated_index);

/**
 * @brief Stores an incoming orbit clock bounds collection into the holding
 *        area.
 * @details If an existing collection has a matching timestamp and collection Id
 * when comparing to the incoming collection and has its complete flag set to
 * false, that collection is instead updated. Otherwise the incoming collection
 * gets inserted into the orbit clock bounds storage.
 *
 * @param collection The incoming collection to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the orbit clock bounds storage
 *                      that was updated.
 * @return Success if the incoming collection was stored and had the complete
 *                 flag set to true.
 *         Corrections unavailable if the incoming collection was stored and had
 *                                 the complete flag set to false.
 *         Broken sequence if the storing of the incoming collection failed.
 */
ReturnCode store_orbit_clock_bounds(
    const SsrOrbitClockBoundsCollection &collection, HoldingArea *holding_area,
    std::size_t *updated_index);

/**
 * @brief Stores an incoming orbit clock bounds degradation collection into the
 *        holding area.
 * @details If an existing collection has a matching timestamp and collection Id
 * when comparing to the incoming collection and has its complete flag set to
 * false, that collection is instead updated. Otherwise the incoming collection
 * gets inserted into the orbit clock bounds degradation storage.
 *
 * @param collection The incoming collection to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the orbit clock bounds
 *                      degradation storage that was updated.
 * @return Success if the incoming collection was stored and had the complete
 *                 flag set to true.
 *         Corrections unavailable if the incoming collection was stored and had
 *                                 the complete flag set to false.
 *         Broken sequence if the storing of the incoming collection failed.
 */
ReturnCode store_orbit_clock_degradation(
    const SsrOrbitClockBoundsDegradationCollection &collection,
    HoldingArea *holding_area, std::size_t *updated_index);

/**
 * @brief Stores an incoming code bias collection into the holding area.
 * @details If an existing collection has a matching timestamp and collection id
 * when comparing to the incoming collection and has its complete flag set to
 * false, that collection is instead updated. Otherwise the incoming collection
 * gets inserted into the code biases storage.
 *
 * @param collection The incoming collection to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the code biases storage that was
 *                      updated.
 * @return Success if the incoming collection was stored and had the complete
 *                 flag set to true.
 *         Corrections unavailable if the incoming collection was stored and had
 *                                the complete flag set to false.
 *         Broken sequence if the storing of the incoming collection failed.
 */
ReturnCode store_code_biases(const SsrCodeBiasesCollection &collection,
                             HoldingArea *holding_area,
                             std::size_t *updated_index);

/**
 * @brief Stores an incoming code and phase bias bounds collection into the
 *        holding area.
 * @details If an existing collection has a matching timestamp and collection Id
 * when comparing to the incoming collection and has its complete flag set to
 * false, that collection is instead updated. Otherwise the incoming collection
 * gets inserted into the code/phase bias bounds storage.
 *
 * @param collection The incoming collection to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the code/phase bias bounds
 *                      storage that was updated.
 * @return Success if the incoming collection was stored and had the complete
 *                 flag set to true.
 *         Corrections unavailable if the incoming collection was stored and had
 *                                 the complete flag set to false.
 *         Broken sequence if the storing of the incoming collection failed.
 */
ReturnCode store_code_and_phase_bias_bounds(
    const SsrCodeAndPhaseBiasBoundsCollection &collection,
    HoldingArea *holding_area, std::size_t *updated_index);

/**
 * @brief Stores an incoming phase bias collection into the holding area.
 * @details If an existing collection has a matching timestamp and collection id
 * when comparing to the incoming collection and has its complete flag set to
 * false, that collection is instead updated. Otherwise the incoming collection
 * gets inserted into the phase biases storage.
 *
 * @param collection The incoming collection to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the phase biases storage that
 *                      was updated.
 * @return Success if the incoming collection was stored and had the complete
 *                 flag set to true.
 *         Corrections unavailable if the incoming collection was stored and had
 *                                the complete flag set to false.
 *         Broken sequence if the storing of the incoming collection failed.
 */
ReturnCode store_phase_biases(const SsrPhaseBiasesCollection &collection,
                              HoldingArea *holding_area,
                              std::size_t *updated_index);

/**
 * @brief Stores an incoming satellite apc collection into the holding area.
 * @details If an existing collection has a matching timestamp, solution id and
 * iod_ssr when comparing to the incoming collection, that collection is instead
 * updated. Otherwise the incoming collection gets inserted into the
 * satellite apc storage.
 *
 * @param collection The incoming collection to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the satellite apc storage that
 *                      was updated.
 * @return Success if the incoming collection was stored.
 *         Corrections unavailable if the incoming collection was stored and had
 *                                the complete flag set to false.
 *         Broken sequence if the storing of the incoming collection failed.
 */
ReturnCode store_satellite_apcs(const SsrSatelliteApcCollection &collection,
                                HoldingArea *holding_area,
                                std::size_t *updated_index);
/**
 * @brief Stores an incoming tile definition into the holding area.
 * @details The incoming tile definition is always inserted into the tile
 * definition storage.
 *
 * @param collection The incoming collection to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the tile definition storage that
 *                      was updated.
 * @return Success if the incoming tile definition was stored and had the
 *                 complete flag set to true.
 *         Corrections unavailable if the incoming tile definition was stored
 *                                 and had the complete flag set to false.
 */
ReturnCode store_tile_definition(const TileDefinitionCollection &collection,
                                 HoldingArea *holding_area,
                                 std::size_t *updated_index);

/**
 * @brief Stores an incoming atmo grid into the holding area.
 * @details If an existing collection has a matching timestamp, collection id,
 * tile id and tile set id when comparing to the incoming grid and has its
 * complete flag set to false, that grid is instead updated. Otherwise the
 * incoming grid gets inserted into the atmo grid storage.
 *
 * @param grid The incoming grid to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the atmo grid storage that was
 *                      updated.
 * @return Success if the incoming grid was stored and had the complete
 *                 flag set to true.
 *         Corrections unavailable if the incoming grid was stored and had
 *                                the complete flag set to false.
 *         Broken sequence if the storing of the incoming grid failed.
 */
ReturnCode store_atmo_grid(const AtmoGrid &grid, HoldingArea *holding_area,
                           std::size_t *updated_index);

/**
 * @brief Stores an incoming stec polynomial sat collection into the holding
 * area.
 * @details If an existing collection has a matching timestamp, collection id,
 * tile id and tile set id when comparing to the incoming collection and has its
 * complete flag set to false, that collection is instead updated. Otherwise the
 * incoming collection gets inserted into the stec polynomial sat storage.
 *
 * @param collection The incoming collection to store in the holding area.
 * @param holding_area Pointer to the holding area.
 * @param updated_index Pointer to the index in the stec polynomial storage that
 *                      was updated.
 * @return Success if the incoming collection was stored and had the complete
 *                 flag set to true.
 *         Corrections unavailable if the incoming collection was stored and had
 *                                the complete flag set to false.
 *         Broken sequence if the storing of the incoming collection failed.
 */
ReturnCode store_stec_polynomials(const StecPolynomialSatCollection &collection,
                                  HoldingArea *holding_area,
                                  std::size_t *updated_index);

/**
 * @brief Checks if the storage contains the Apc signal
 * @param incoming Incoming Apc signal
 * @param storage Storage area
 * @return True if signal is in storage, false otherwise.
 */
bool contains_signal(const SsrSatelliteApc &incoming,
                     const SsrSatelliteApc &storage);

/**
 * @brief Update stored Ssr Orbit Clock satellite
 * @param incoming New satellite to be updated
 * @param stored Stored satellite
 * @return
 */
void update_satellite(const SsrOrbitClockSat &incoming,
                      SsrOrbitClockSat *const stored);

/**
 * @brief Update stored Ssr Orbit Clock Bounds satellite
 * @param incoming New satellite to be updated
 * @param stored Stored satellite
 * @return
 */
void update_satellite(const SsrOrbitClockBoundsSat &incoming,
                      SsrOrbitClockBoundsSat *const stored);

/**
 * @brief Check if sequence is broken
 * @param incoming Incoming collection with Ssr Satellite Apc objects
 * @param stored Stored collection with Ssr Satellite Apc objects
 * @return True if sequence is broken, false otherwise
 */
bool is_sequence_broken(const SsrSatelliteApcCollection &incoming,
                        const SsrSatelliteApcCollection &stored);

/**
 * @brief Check if collection contains a specific constellation
 * @tparam Collection Collection type
 * @param constellation Constellation to check inside collection
 * @param collection Collection where to check constellation
 * @param found_idx Index of the first satellite with the desired constellation
 * @return True if constellation was found, false otherwise
 */
template <typename Collection>
bool contains_constellation(const GnssId constellation,
                            const Collection &collection,
                            std::size_t *const found_idx = nullptr) {
  const std::size_t collection_satellites_size = collection.satellites.size();
  for (std::size_t sat_idx = 0; (sat_idx < collection.satellites_count) &&
                                (sat_idx < collection_satellites_size);
       ++sat_idx) {
    if (collection.satellites[sat_idx].sat_id.constellation == constellation) {
      if (found_idx != nullptr) {
        *found_idx = sat_idx;
      }
      return true;
    }
  }
  return false;
}

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_STORAGE_AREAS_HOLDING_AREA_H
