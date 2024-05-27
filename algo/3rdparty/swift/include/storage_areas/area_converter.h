/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFT_SSR2LOS_SRC_STORAGE_AREAS_AREA_CONVERTER_H
#define SWIFT_SSR2LOS_SRC_STORAGE_AREAS_AREA_CONVERTER_H

#include <array>

#include <swift/config.h>

#include <integrity/integrity_provider.h>
#include <storage_areas/active_area.h>
#include <storage_areas/holding_area.h>

namespace swift {
namespace internal {

struct CriteriaVariables {
  CollectionIdentifier id;
  uint16_t update_interval_s = 0;
  Timestamp timestamp;
};

struct CompatibleAtmoCollection {
  std::size_t stec_storage_idx = 0;
  std::size_t atmo_storage_idx = 0;
  TileId tile;
};
using CompatibleAtmoCollections =
    StaticBuffer<CompatibleAtmoCollection, cMaxTiles>;

static constexpr std::size_t cCompatibleSatComponentCount = 7;
using CompatibleSatCollections =
    StaticBuffer<CollectionIndexMap, cCompatibleSatComponentCount>;

/**
 * @brief Contains indices in Holding Area for both satellite specific
 * components as well as atmosphere specific components, that together forms
 * a compatible collection, for one satellite.
 */
struct CompatibleCollection {
  bool is_iod_ssr_set = false;
  uint8_t iod_ssr = 0;
  uint8_t iod_atmo = 0;

  CompatibleSatCollections sat_components;
  CompatibleAtmoCollections atmo_components;
};

class AreaConverter {
 public:
  AreaConverter() : AreaConverter(InitConfig{}){};
  explicit AreaConverter(const InitConfig &config)
      : enable_bounds_calculation_(
            config.integrity.enable_bounds_calculation){};

  /**
   * @brief Convert the content of holding area to active area.
   * @details Using a CollectionIndexMap which contains a CollectionType and an
   * index pointing to an item in a collection, which would typically be set
   * when storing a received callback from the RTCM decoder in the holding area.
   * This struct is what could be seen as the "trigger" to start the conversion
   * process.
   *
   * @param[in] holding_area The holding area.
   * @param[in] collection_trigger The collection that "triggered" the
   * conversion process.
   * @param[in] integrity_provider The integrity provider that holds integrity
   * information regarding the correction to be moved into active.
   * @param[out] active_area Pointer to the active area.
   * @return Success if compatible collections are found and the active area has
   * been updated. Corrections unavailable if the compatibility criteria failed.
   */
  ReturnCode convert_collections(const HoldingArea &holding_area,
                                 const CollectionIndexMap &collection_trigger,
                                 const IntegrityProvider &integrity_provider,
                                 ActiveArea *active_area) const;

  /**
   * @brief Convert the content of holding area to active area using the latest
   * received triggers.
   * @details This function will create a trigger from the latest stored
   * collection in holding area for each collection type. Then it try to convert
   * collections using those triggers. If any trigger convert collections
   * successfully this function will return SUCCESS.
   *
   * @param[in] holding_area The holding area.
   * @param[out] active_area Pointer to the active area.
   * @return Success if compatible collections are found and the active area has
   * been updated either with satellite corrections or atmospheric corrections
   * or both. Corrections unavailable if the compatibility criteria fails for
   * every collection type.
   */
  ReturnCode convert_collections_with_latest_triggers(
      const HoldingArea &holding_area,
      const IntegrityProvider &integrity_provider,
      ActiveArea *active_area) const;

 private:
  bool enable_bounds_calculation_;

  /**
   * @brief Convert the content of holding area to active area using the latest
   * received satellite triggers.
   * @details This function will create a trigger from the latest stored
   * collection in holding area for each satellite collection type. Then it try
   * to convert collections using those triggers. If any trigger convert
   * collections successfully this function will return SUCCESS.
   *
   * @param[in] holding_area The holding area.
   * @param[out] active_area Pointer to the active area.
   * @return Success if compatible collections are found and the active area has
   * been updated with satellite corrections. Corrections unavailable if the
   * compatibility criteria fails for every collection type.
   */
  ReturnCode convert_sat_collections_with_latest_triggers(
      const HoldingArea &holding_area,
      const IntegrityProvider &integrity_provider,
      ActiveArea *active_area) const;

  /**
   * @brief Convert the content of holding area to active area using the latest
   * received atmospheric triggers.
   * @details This function will create a trigger from the latest stored
   * collection in holding area for each atmospheric collection type. Then it
   * try to convert collections using those triggers. If any trigger convert
   * collections successfully this function will return SUCCESS.
   *
   * @param[in] holding_area The holding area.
   * @param[out] active_area Pointer to the active area.
   * @return Success if compatible collections are found and the active area has
   * been updated with atmospheric corrections. Corrections unavailable if the
   * compatibility criteria fails for every collection type.
   */
  ReturnCode convert_atmo_collections_with_latest_triggers(
      const HoldingArea &holding_area,
      const IntegrityProvider &integrity_provider,
      ActiveArea *active_area) const;

  /**
   * @brief Convert the satellite collections content from holding area to
   * coherent sets later used to fill the active area.
   *
   * @param[in] holding_area The holding area.
   * @param[in] integrity_provider The integrity provider that holds integrity
   * information regarding the correction to be moved into active.
   * @param[in] criteria Criteria given by the collection that "triggered" the
   * conversion process.
   * @param[in] satellites_count Number of satellites in satellites array.
   * @param[in] satellites Array of satellite descriptions for which we
   * try to convert the satellite collections from the holding area to the
   * active area.
   * @param[out] coherent_sets_count Pointer to the number of coherent sets
   * created.
   * @param[out] coherent_sets Pointer to the coherent sets that will update the
   * active area.
   */
  void convert_sat_collections(
      const HoldingArea &holding_area,
      const IntegrityProvider &integrity_provider,
      const CriteriaVariables &criteria, const std::size_t satellites_count,
      const std::array<SatelliteDescription, cMaxSats> &satellites,
      std::size_t *coherent_sets_count,
      std::array<SsrCoherentSet, cMaxSats> *coherent_sets) const;

  /**
   * @brief Convert the atmospheric collections content from holding area to
   * coherent sets later used to fill the active area.
   *
   * @param[in] holding_area The holding area.
   * @param[in] integrity_provider The integrity provider that holds integrity
   * information regarding the correction to be moved into active.
   * @param[in] criteria Criteria given by the collection that "triggered" the
   * conversion process.
   * @param[in] satellites_count Number of satellites in satellites array.
   * @param[in] satellites Array of satellite descriptions for which we
   * try to convert the atmospheric collections from the holding area to the
   * active area.
   * @param[out] coherent_sets_count Pointer to the number of coherent sets
   * created.
   * @param[out] coherent_sets Pointer to the coherent sets that will update the
   * active area.
   */
  void convert_atmo_collections(
      const HoldingArea &holding_area,
      const IntegrityProvider &integrity_provider,
      const CriteriaVariables &criteria, const std::size_t satellites_count,
      const std::array<SatelliteDescription, cMaxSats> &satellites,
      std::size_t *coherent_sets_count,
      std::array<SsrCoherentSet, cMaxSats> *coherent_sets) const;

  /**
   * @brief From the collection that triggered the conversion process, extract a
   * list of all satellites IDs. Pick the satellites in this list from all
   * other compatible collections.
   *
   * @param[in] holding_area The holding area.
   * @param[in] collection_trigger The collection that "triggered" the
   * conversion process.
   * @param[out] satellites_count Pointer to the number of satellites.
   * @param[out] satellites Pointer to the array containing satellite
   * IDs.
   */
  static void filter_satellites(
      const HoldingArea &holding_area,
      const CollectionIndexMap &collection_trigger,
      std::size_t *satellites_count,
      std::array<SatelliteDescription, cMaxSats> *satellites);

  /**
   * @brief From the holding area, extract indices for satellite specific
   * corrections (orbit&clock, code bias, phase bias) which match with the
   * criteria parameters.
   * @details For each CollectionIndexMap in the sat_component output,
   * the following criteria are met:
   * - The complete flag is set to true.
   * - The collection identifier is equal to the identifier in the criteria.
   * - The time criteria is met.
   *
   * @param[in] sat_id The satellite ID to move into active area.
   * @param[in] criteria Criteria for the tested collections to meet.
   * @param[in] holding_area The holding area.
   * @param[in] integrity_provider The integrity provider.
   * @param[out] compatible_collection Pointer to the container with indices
   * into the holding area storage for which their components match the criteria
   * as explained above.
   * @return true if anything is present in the output.
   */
  bool get_compatible_sat_collections(
      const SatelliteDescription &sat_id, const CriteriaVariables &criteria,
      const HoldingArea &holding_area,
      const IntegrityProvider &integrity_provider,
      CompatibleCollection *compatible_collection) const;

  /**
   * @brief From the holding area, extract indices for Stec and Atmo components
   * which match both the criteria parameters as well as match each others
   * tile_id and tile_set_id.
   * @details For each atmo_component in the output, the following
   * criteria are met:
   * - The complete flag is set to true.
   * - The collection identifier is equal to the identifier in the criteria.
   * - The time criteria is met.
   * - The atmo and stec components have matching tile_id and tile_set_id.
   *
   * @param[in] sat_id The satellite ID to move into active area.
   * @param[in] criteria Criteria for the tested collections to meet.
   * @param[in] holding_area The holding area.
   * @param[in] integrity_provider The integrity provider.
   * @param[out] compatible_collection Pointer to the container with indices
   * into the holding area storage for which their components match the criteria
   * as explained above.
   * @return true if anything is present in the output.
   */
  static bool get_compatible_atmo_collections(
      const SatelliteDescription &sat_id, const CriteriaVariables &criteria,
      const HoldingArea &holding_area,
      const IntegrityProvider &integrity_provider,
      CompatibleCollection *compatible_collection);
  /**
   * @brief Convert orbits and clocks from holding area to active area.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] collection The orbits clocks collection to convert.
   * @param[out] orbit Pointer to the updated orbit set in active area.
   * @param[out] clock Pointer to the updated clock set in active area.
   * @param[out] coherent_set_iode Pointer to the set IODE value for the
   * satellite in the coherent set.
   */
  void convert_orbits_clocks(const SatelliteDescription &sat_id,
                             const SsrOrbitClockCollection &collection,
                             SsrCoherentSet::Orbit *orbit,
                             SsrCoherentSet::Clock *clock,
                             uint16_t *coherent_set_iode) const;

  /**
   * @brief Convert orbit and clock bounds from holding area to active area.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] collection The orbit and clock bounds collection to convert.
   * @param[out] orbit Pointer to the updated orbit set in active area.
   * @param[out] clock Pointer to the updated clock set in active area.
   */
  void convert_orbit_clock_bounds(
      const SatelliteDescription &sat_id,
      const SsrOrbitClockBoundsCollection &collection,
      SsrCoherentSet::Orbit *orbit, SsrCoherentSet::Clock *clock) const;

  /**
   * @brief Convert orbit and clock bounds degradation from holding area to
   * active area.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] collection The orbit and clock bounds degradation collection to
   * convert.
   * @param[out] orbit Pointer to the updated orbit set in active area.
   * @param[out] clock Pointer to the updated clock set in active area.
   */
  void convert_orbit_clock_degradations(
      const SatelliteDescription &sat_id,
      const SsrOrbitClockBoundsDegradationCollection &collection,
      SsrCoherentSet::Orbit *orbit, SsrCoherentSet::Clock *clock) const;

  /**
   * @brief Convert code biases from holding area to active area.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] collection The code bias collection to convert.
   * @param[out] code_bias Pointer to the updated code bias set in active area.
   */
  void convert_code_biases(const SatelliteDescription &sat_id,
                           const SsrCodeBiasesCollection &collection,
                           SsrCoherentSet::CodeBias *code_bias) const;

  /**
   * @brief Convert code and phase bias bounds from holding area to active area.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] collection The code and phase bias bound collection to convert.
   * @param[out] code_bias Pointer to the updated code bias set in active area.
   * @param[out] phase_bias Pointer to the updated phase bias set in active
   * area.
   */
  void convert_code_and_phase_bounds(
      const SatelliteDescription &sat_id,
      const SsrCodeAndPhaseBiasBoundsCollection &collection,
      SsrCoherentSet::CodeBias *code_bias,
      SsrCoherentSet::PhaseBias *phase_bias) const;

  /**
   * @brief Convert phase biases from holding area to active area.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] collection The phase biases collection to convert.
   * @param[out] phase_bias Pointer to the updated phase bias in active area.
   */
  void convert_phase_biases(const SatelliteDescription &sat_id,
                            const SsrPhaseBiasesCollection &collection,
                            SsrCoherentSet::PhaseBias *phase_bias) const;

  /**
   * @brief Convert satellite antenna phase center parameters from holding area
   * to active area.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] collection The satellite apc collection to convert.
   * @param[out] satellite_apc Pointer to the updated satellite apc set in
   * active area.
   */
  void convert_satellite_apc(const SatelliteDescription &sat_id,
                             const SsrSatelliteApcCollection &collection,
                             SsrCoherentSet::SatelliteApc *satellite_apc) const;

  /**
   * @brief Convert tile definition parameters from holding area
   * to active area.
   *
   * @param[in] collection The tile definition collection to convert.
   * @param[out] tile_definitions Pointer to the updated tile definitions in
   * active area.
   */
  void convert_tile_definitions(
      const TileDefinitionCollection &collection,
      StaticBuffer<TileDefinition, cMaxTiles> *tile_definitions) const;

  /**
   * @brief Helper function to fill an Iono set with stec polynomials when
   * converting atmospheric components.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] stec The StecPolynomialSatCollection to use from holding area.
   * @param[out] iono_set The Iono set to fill.
   */
  void fill_stec(const SatelliteDescription &sat_id,
                 const StecPolynomialSatCollection &stec,
                 SsrCoherentSet::Iono *iono_set) const;

  /**
   * @brief Helper function to fill a Tropo set and Iono set with atmo grid
   * points when converting atmospheric components.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] atmo_grid The AtmoGrid to use from holding area.
   * @param[out] tropo_set The tropo set to fill.
   * @param[out] iono_grid_points The grid point to fill for an Iono set.
   */
  void fill_atmo(
      const SatelliteDescription &sat_id, const AtmoGrid &atmo_grid,
      SsrCoherentSet::Tropo *tropo_set,
      std::array<StecResiduals, cMaxGridPoints> *iono_grid_points) const;

  /**
   * @brief For each satellite components, fill the complete set with the
   * information in the holding area pointed by the index container. The
   * complete set will then update the active area.
   *
   * @param[in] sat_id The satellite ID to move into active area.
   * @param[in] holding_area The holding area.
   * @param[in] compatible_collection Container with indices into the holding
   * area storage for which their components match the criteria as explained
   * above.
   * @param[out] complete_set Pointer to the correction set to fill with
   * compatible collection values
   */
  void convert_satellite_components(
      const SatelliteDescription &sat_id, const HoldingArea &holding_area,
      const CompatibleSatCollections &compatible_sat_collection,
      SsrCoherentSet *complete_set) const;

  /**
   * @brief Convert stec polynomials collection and Atmo grid from holding area
   * to active area.
   *
   * @param[in] sat_id The satellite ID to use.
   * @param[in] holding_area The holding area.
   * @param[in] compatible_atmo_collections Indices for each collection to use
   * from the stec polynomial storage and atmo grid storage respectively. Stored
   * in a StaticBuffer of cMaxTiles to enable smooth transitions when changing
   * tiles.
   * @param[out] tropo_storage Pointer to the updated tropo storage in active
   * area.
   * @param[out] iono_storage Pointer to the updated iono storage in active
   * area.
   */
  void convert_atmospheric_components(
      const SatelliteDescription &sat_id, const HoldingArea &holding_area,
      const CompatibleAtmoCollections &compatible_atmo_collections,
      StaticBuffer<SsrCoherentSet::Tropo, cMaxTiles> *tropo_storage,
      StaticBuffer<SsrCoherentSet::Iono, cMaxTiles> *iono_storage) const;
};

/**
 * @brief Make the criteria that needs to be fulfilled for the messages to be
 * added to the active area
 * @param collection_trigger Incoming collection that triggered a conversion
 * @param holding_area Holding area
 * @param criteria Criteria to be set
 */
void make_criteria(const CollectionIndexMap &collection_trigger,
                   const HoldingArea &holding_area,
                   CriteriaVariables *const criteria);

/**
 * @brief Checks if the storage contains a specific satellite
 * @param sat_id Satellite identifier
 * @param storage Storage area
 * @return True if satellite is in storage, false otherwise.
 */
bool contains_satellite(
    const SatelliteDescription &sat_id,
    const StaticBuffer<SsrSatelliteApc, cApcSlots> &storage);

/**
 * @brief Checks if the storage contains a specific satellite
 * @tparam SatCollection Satellite collection type
 * @param sat_id Satellite identifier
 * @param storage Storage area
 * @param storage_count Amount of satellites in storage
 * @return True if satellite is in storage, false otherwise.
 */
template <typename SatCollection>
bool contains_satellite(const SatelliteDescription &sat_id,
                        const std::array<SatCollection, cMaxSats> &storage,
                        const std::size_t storage_count) {
  const std::size_t storage_size = storage.size();
  for (std::size_t sat_idx = 0;
       (sat_idx < storage_count) && (sat_idx < storage_size); ++sat_idx) {
    if (storage[sat_idx].sat_id == sat_id) {
      return true;
    }
  }
  return false;
}

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_STORAGE_AREAS_AREA_CONVERTER_H
