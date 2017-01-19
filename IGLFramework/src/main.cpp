#include "nanogui/formhelper.h"
#include "nanogui/screen.h"

#include "igl/readOFF.h"
//#ifdef _WIN32
//	#include "nanogui/opengl.h"
//#endif
#include "igl/viewer/Viewer.h"

#include <iostream>
#include "nanoflann.hpp"


// TODO: move to typedefs.h
typedef Eigen::MatrixXd CloudT;

// This function is called every time a keyboard button is pressed
bool key_down(igl::viewer::Viewer& viewer, unsigned char key, int modifier) {
    #if 0
    switch(key)
    {
        case '1':
            viewer.data.set_normals(N_faces);
            return true;
        case '2':
            viewer.data.set_normals(N_vertices);
            return true;
        case '3':
            viewer.data.set_normals(N_corners);
            return true;
        default: break;
    }
    #endif

    return false;
}

Eigen::Matrix<typename CloudT::Scalar, 3, 1>
calculatePointNormal(CloudT const& cloud, int const pointIndex, std::vector<size_t> const& neighbourIndices) {
    typedef typename CloudT::Scalar Scalar;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

    Matrix3 cov(Matrix3::Zero());
    for (size_t const neighbourIndex : neighbourIndices) {
        if (pointIndex == neighbourIndex) {
            std::cerr << "same index..." << pointIndex << " vs " << neighbourIndex << "\n";
            continue;
        }
        Vector3 const pos =
            cloud.row(neighbourIndex) - cloud.row(pointIndex);
        cov += pos * pos.transpose();
    }

    // solve for neighbourhood biggest eigen value
    Eigen::SelfAdjointEigenSolver<Matrix3> es(cov);
    int const smallestEigenValueId =
        static_cast<int>(
            std::distance(
                es.eigenvalues().data(),
                std::min_element(
                    es.eigenvalues().data(),
                    es.eigenvalues().data()+3
                )
            )
        );

    std::cout << "MinEigValId: " << smallestEigenValueId << "\n";
    return es.eigenvectors().col(smallestEigenValueId).normalized();
} // calculatePointNormal()

CloudT
calculateCloudNormals(CloudT const& cloud, int const k, int const maxLeafs = 10) {
    typedef typename CloudT::Scalar Scalar;
    enum { Dim = 3 };
    typedef nanoflann::KDTreeEigenMatrixAdaptor<
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>,
        Dim,
        nanoflann::metric_L2
    >  KdTreeWrapperT;

    if (cloud.cols() != Dim) {
        std::cerr << "Point dimension mismatch: " << cloud.cols() << " vs. " << Dim << "\n";
        throw new std::runtime_error("Point dimension mismatch");
    }

    std::cout << "cols: " << cloud.cols() << " vs dim: " << Dim << "\n";
    KdTreeWrapperT cloudIndex( Dim, cloud, maxLeafs);
    cloudIndex.index->buildIndex();

    // do a knn search
    std::vector<size_t> neighbourIndices(k);
    std::vector<Scalar> out_dists_sqr(k);

    nanoflann::KNNResultSet<Scalar> resultSet(k);

    CloudT normals(cloud.rows(), 3);
    for (int pointId = 0; pointId != cloud.rows(); ++pointId) {
        resultSet.init(&neighbourIndices[0], &out_dists_sqr[0]);
        static_assert(
            std::is_same<Scalar, double>::value,
            "Double input assumed next. Otherwise, explicit copy is needed!"
        );
        cloudIndex.index->findNeighbors(
            resultSet,
            cloud.row(pointId).data(),
            nanoflann::SearchParams(k));

//        std::cout << "knnSearch(nn=" << k << "): \n";
//        for (size_t i = 0; i < neighbourIndices.size(); ++i)
//            std::cout << "ret_index[" << i << "]=" << neighbourIndices[i] << " out_dist_sqr=" << out_dists_sqr[i]
//                      << std::endl;
        normals.row(pointId) = calculatePointNormal(cloud, pointId, neighbourIndices);
    }
    return normals;
} //...calculateCloudNormals()


void
recalcNeighbours(int kNeighbours, CloudT const& V, igl::viewer::Viewer &viewer) {
    CloudT const normals =
        calculateCloudNormals(
            /*        cloud: */ V,
            /* k-neighbours: */ kNeighbours
        );
    viewer.data.set_normals(normals);
    viewer.data.lines = Eigen::MatrixXd(0, 9);
    viewer.data.add_edges(V, V + normals * 0.01, Eigen::Vector3d::Zero());
}

int main(int argc, char *argv[]) {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    bool boolVariable = true;
    float floatVariable = 0.1f;
    int kNeighbours = 5;
    enum Orientation { Up=0, Down, Left, Right } dir = Up;

    // Load a mesh in OFF format
    std::string meshPath = "../3rdparty/libigl/tutorial/shared/bunny.off";
    if (argc > 1)
        meshPath = std::string(argv[1]);
    igl::readOFF(meshPath, V, F);
    if (!(V.rows() > 0)) {
        std::cerr << "Could not read mesh at " << meshPath << "...exiting...\n";
        return EXIT_FAILURE;
    }

    // Plot the mesh
    igl::viewer::Viewer viewer;

    // Extend viewer menu
    viewer.callback_init = [&](igl::viewer::Viewer& viewer)
    {
        // Add new group
        viewer.ngui->addGroup("Acquisition16");

        viewer.ngui->addVariable<int>(
            "kNeighbours",
            [&](int val) {
                kNeighbours = val; // set
                recalcNeighbours(kNeighbours, V, viewer);
            },
            [&]() {
                return kNeighbours; // get
            }
        );

        // Expose variable directly ...
        viewer.ngui->addVariable("float",floatVariable);

        // ... or using a custom callback
        viewer.ngui->addVariable<bool>(
            "bool",
            [&](bool val) {
                boolVariable = val; // set
            },
            [&]() {
                return boolVariable; // get
            }
        );

        // Expose an enumaration type
        viewer.ngui->addVariable<Orientation>("Direction",dir)->setItems({"Up","Down","Left","Right"});

        // Add a button
        viewer.ngui->addButton("Print Hello",[](){ std::cout << "Hello\n"; });

        // Add an additional menu window
        //viewer.ngui->addWindow(Eigen::Vector2i(220,10),"New Window");

        // Expose the same variable directly ...
        //viewer.ngui->addVariable("float",floatVariable);

        // Generate menu
        viewer.screen->performLayout();

        return false;
    };

    viewer.callback_key_down = &key_down;
    viewer.core.show_lines = false;
    viewer.data.set_mesh(V, F);
    recalcNeighbours(kNeighbours, V, viewer);
    viewer.launch();

    return 0;
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
//template class igl::viewer::Viewer;
//template void igl::per_face_normals<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);
#endif
