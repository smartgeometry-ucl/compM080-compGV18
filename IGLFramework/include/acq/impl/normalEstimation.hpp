//
// Created by bontius on 19/01/17.
//

#ifndef ACQ_NORMALESTIMATION_HPP
#define ACQ_NORMALESTIMATION_HPP

#include "acq/normalEstimation.h"

#include "nanoflann/nanoflann.hpp"

#include "Eigen/Eigenvalues"        // SelfAdjointEigenSolver

#include <queue>
#include <set>
#include <iostream>

namespace acq {

Eigen::Matrix <typename CloudT::Scalar, 3, 1>
calculatePointNormal(
    CloudT                  const& cloud,
    int                     const  pointIndex,
    std::vector <size_t>    const& neighbourIndices
) {
    //! Floating point type
    typedef typename CloudT::Scalar Scalar;
    //! 3x1 vector type
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    //! 3x3 matrix type
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    // Covariance matrix initialized to 0-s:
    Matrix3 cov(Matrix3::Zero());

    // For each neighbouring point
    for (size_t const neighbourIndex : neighbourIndices) {

        // Warn, if first neighbour is same point
        if (pointIndex == neighbourIndex) {
            std::cerr << "same index..."
                      << pointIndex << " vs "
                      << neighbourIndex << "\n";
            continue;
        } //...if same index

        // Calculate relative vector to neighbour
        Vector3 const vToNeighbour =
            cloud.row(neighbourIndex) - cloud.row(pointIndex);

        // Sum up covariance
        cov += vToNeighbour * vToNeighbour.transpose();

    } //...For neighbours

    // Solve for neighbourhood smallest eigen value
    Eigen::SelfAdjointEigenSolver <Matrix3> es(cov);

    // Find index of smallest eigen value
    int const smallestEigenValueId =
        static_cast<int>(
            std::distance(
                es.eigenvalues().data(),
                std::min_element(
                    es.eigenvalues().data(),
                    es.eigenvalues().data() + 3
                )
            )
        );

    // Return smallest eigen vector
    return es.eigenvectors()
             .col(smallestEigenValueId)
             .normalized();

} // calculatePointNormal()

NeighboursT
calculateCloudNeighbours(
    CloudT  const& cloud,
    int     const  k,
    int     const  maxLeafs
) {
    // Floating point type
    typedef typename CloudT::Scalar Scalar;
    // Point dimensions
    enum { Dim = 3 };
    // Copy-free Eigen->FLANN wrapper
    typedef nanoflann::KDTreeEigenMatrixAdaptor <
        /*    Eigen matrix type: */ CloudT,
        /* Space dimensionality: */ Dim,
        /*      Distance metric: */ nanoflann::metric_L2
    > KdTreeWrapperT;

    // Safety check dimensionality
    if (cloud.cols() != Dim) {
        std::cerr << "Point dimension mismatch: " << cloud.cols()
                  << " vs. " << Dim
                  << "\n";
        throw new std::runtime_error("Point dimension mismatch");
    } //...check dimensionality

    // Build KdTree
    KdTreeWrapperT cloudIndex(Dim, cloud, maxLeafs);
    cloudIndex.index->buildIndex();

    // Neighbour indices
    std::vector<size_t> neighbourIndices(k);
    std::vector<Scalar> out_dists_sqr(k);

    // Placeholder structure for nanoFLANN
    nanoflann::KNNResultSet <Scalar> resultSet(k);

    // Associative list of neighbours: { pointId => [neighbourId_0, nId_1, ... nId_k-1] }
    NeighboursT neighbours;
    // For each point, store normal
    for (int pointId = 0; pointId != cloud.rows(); ++pointId) {
        // Initialize nearest neighhbour estimation
        resultSet.init(&neighbourIndices[0], &out_dists_sqr[0]);

        // Make sure it's ok to expose raw data pointer of point
        static_assert(
            std::is_same<Scalar, double>::value,
            "Double input assumed next. Otherwise, explicit copy is needed!"
        );

        // Find neighbours of point in "pointId"-th row
        cloudIndex.index->findNeighbors(
            /*                Output wrapper: */ resultSet,
            /* Query point double[3] pointer: */ cloud.row(pointId).data(),
            /*    How many neighbours to use: */ nanoflann::SearchParams(k)
        );

        // Store list of neighbours
        std::pair<NeighboursT::iterator, bool> const success =
            neighbours.insert(std::make_pair(pointId, neighbourIndices));

        if (!success.second)
            std::cerr << "Could not store neighbours of point " << pointId
                      << ", already inserted?\n";

    } //...for all points

    // return estimated normals
    return neighbours;
} //...calculateCloudNormals()

NormalsT
calculateCloudNormals(
    CloudT      const& cloud,
    NeighboursT const& neighbours
) {
    // Output normals: N x 3
    CloudT normals(cloud.rows(), 3);

    // For each point, store normal
    for (int pointId = 0; pointId != cloud.rows(); ++pointId) {
        // Estimate vertex normal from neighbourhood indices and cloud
        normals.row(pointId) =
            calculatePointNormal(
                /*        PointCloud: */ cloud,
                /*      ID of vertex: */ pointId,
                /* Ids of neighbours: */ neighbours.at(pointId)
            );
    } //...for all points

    // Return estimated normals
    return normals;
} //...calculateCloudNormals()

void
orientCloudNormals(
    NeighboursT const& neighbours,
    NormalsT         & normals
) {
    // List of points to visit
    std::queue<int> queue;
    // Quick-lookup unique container of already visited points
    std::set<int> visited;

    // Initialize list with one point
    queue.push(rand() % normals.size()); // TODO: pick point with low curvature
    // Set visited
    visited.insert(queue.front());

    // While points to visit exist
    while (queue.empty()) {
        // Read next point from queue
        int const pointId = queue.front();
        // Remove point from queue
        queue.pop();

        // Fetch neighbours
        NeighboursT::const_iterator const iter = neighbours.find(pointId);
        // Check, if any neighbours
        if (iter == neighbours.end()) {
            std::cerr << "Could not find neighbours of point " << pointId << "\n";
            continue;
        }

        for (int const neighbourId : iter->second) {
            // If unvisited
            if (visited.find(neighbourId) == visited.end()) {
                // Enqueue for next level
                queue.push(neighbourId);
                // Mark visited
                visited.insert(neighbourId);

                // Flip neighbour normal, if not same direction as precursor point
                if (normals.row(pointId).dot(normals.row(neighbourId)) < 0.f)
                    normals.row(neighbourId) *= -1.f;
            } //...if neighbour unvisited
        } //...for each neighbour of point
    } //...while points in queue
} //...orientCloudNormals()

} //...ns acq

#endif //ACQ_NORMALESTIMATION_HPP
