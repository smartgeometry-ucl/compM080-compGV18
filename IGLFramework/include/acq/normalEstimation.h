//
// Created by bontius on 19/01/17.
//

#ifndef ACQ_NORMALESTIMATION_H
#define ACQ_NORMALESTIMATION_H

#include "acq/typedefs.h"
#include <vector>

namespace acq {

/** \brief Estimates the normal of a single point given its ID and the ID of its neighbours.
 *
 * \param[in] cloud             N x 3 matrix containing points in rows.
 * \param[in] pointIndex        Row-index of point.
 * \param[in] neighbourIndices  List of row-indices of neighbours.
 *
 * \return A 3D vector that is the normal of point with ID \p pointIndex.
 */
Eigen::Matrix <typename CloudT::Scalar, 3, 1>
calculatePointNormal(
    CloudT               const& cloud,
    int                  const  pointIndex,
    std::vector <size_t> const& neighbourIndices);


/** \brief Estimates the neighbours of all points in cloud returning \p k neighbours max each.
 *
 * \param[in] k         How many neighbours too look for in point.
 * \param[in] maxLeafs  FLANN parameter, maximum kdTree depth.
 *
 * \return An associative container with the varying length lists of neighbours.
 */
NeighboursT
calculateCloudNeighbours(
    CloudT               const& cloud,
    int                  const  k,
    int                  const  maxLeafs = 10);

/** \brief Estimates the normals of all points in cloud using \p k neighbours max each.
 *
 * \param[in] cloud      Input pointcloud, N x 3, N 3D points in rows.
 * \param[in] neighbours Precomputed lists of neighbour Ids.
 *
 * \return N x 3 3D normals, the normals of the points in \p cloud.
 */
NormalsT
calculateCloudNormals(
    CloudT               const& cloud,
    NeighboursT          const& neighbours);

void
orientCloudNormals(
    NeighboursT const& neighbours,
    NormalsT         & normals);

} //...ns acq

#endif //ACQ_NORMALESTIMATION_H
