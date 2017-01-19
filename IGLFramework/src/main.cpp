#include "acq/normalEstimation.h"

#include "nanogui/formhelper.h"
#include "nanogui/screen.h"

#include "igl/readOFF.h"
#include "igl/viewer/Viewer.h"

#include <iostream>

namespace acq {

/** \brief                 Re-estimate normals of cloud \p V fitting planes
 *                         to the \p kNeighbours nearest neighbours of each point.
 * \param[in ] kNeighbours How many neighbours to use (Typiclaly: 5..15)
 * \param[in ] V           Input pointcloud. Nx3, where N is the number of points.
 * \param[out] viewer      The viewer to show the normals at.
 * \return                 The estimated normals, Nx3.
 */
CloudT
recalcNormals(
    int const kNeighbours,
    CloudT const& V,
    igl::viewer::Viewer& viewer) {

    NeighboursT const neighbours =
        calculateCloudNeighbours(
            /* [in]        cloud: */ V,
            /* [in] k-neighbours: */ kNeighbours
        );

    // Estimate normals for points in cloud V
    CloudT normals =
        calculateCloudNormals(
            /* [in]               Cloud: */ V,
            /* [in] Lists of neighbours: */ neighbours
        );

    // Flip normals consistently
    orientCloudNormals(
        /* [in     ]        Lists of neighbours: */ neighbours,
        /* [in, out] Normals to change in place: */ normals
    );

    // [Optional] Set viewer face normals for shading
    //viewer.data.set_normals(normals);

    // Clear visualized lines (see Viewer.clear())
    viewer.data.lines = Eigen::MatrixXd(0, 9);

    // Add normals to viewer
    viewer.data.add_edges(
        /* [in] Edge starting points: */ V,
        /* [in]       Edge endpoints: */ V + normals * 0.01,
        /* [in]               Colors: */ Eigen::Vector3d::Zero()
    );

    return normals;
} //...recalcNormals()

} //...ns acq

int main(int argc, char *argv[]) {
    // Pointcloud vertices, N rows x 3 columns.
    Eigen::MatrixXd V;
    // Face indices, M x 3 integers referring to V.
    Eigen::MatrixXi F;

    // How many neighbours to use for normal estimation, shown on GUI.
    int kNeighbours = 5;

    // Dummy enum to demo GUI
    enum Orientation { Up=0, Down, Left, Right } dir = Up;
    // Dummy variable to demo GUI
    bool boolVariable = true;
    // Dummy variable to demo GUI
    float floatVariable = 0.1f;

    // Load a mesh in OFF format
    std::string meshPath = "../3rdparty/libigl/tutorial/shared/bunny.off";
    if (argc > 1) {
        meshPath = std::string(argv[1]);
    } else {
        std::cout << "Usage: iglFrameWork <path-to-off-mesh.>" << "\n";
    }

    // Read mesh
    igl::readOFF(meshPath, V, F);
    // Check, if any vertices read
    if (V.rows() <= 0) {
        std::cerr << "Could not read mesh at " << meshPath
                  << "...exiting...\n";
        return EXIT_FAILURE;
    } //...if vertices read

    // Visualize the mesh in a viewer
    igl::viewer::Viewer viewer;

    // Extend viewer menu using a lambda function
    // "[&]" captures all scope variables by reference
    viewer.callback_init = [&](igl::viewer::Viewer& viewer)
    {
        // Add new group
        viewer.ngui->addGroup("Acquisition16");

        // Add k-neighbours variable to GUI
        viewer.ngui->addVariable<int>(
            /* Displayed name: */ "kNeighbours",
            /*  Setter lambda: */ [&](int val) {
                // Store new value
                kNeighbours = val;
                // Recalculate normals for cloud and update viewer
                acq::recalcNormals(kNeighbours, V, viewer);
            },
            /*  Getter lambda: */ [&]() {
                return kNeighbours; // get
            }
        );

        // Expose variable directly ...
        viewer.ngui->addVariable("float", floatVariable);

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
        viewer.ngui->addVariable<Orientation>("Direction",dir)->setItems(
            {"Up","Down","Left","Right"}
        );

        // Add a button
        viewer.ngui->addButton("Print Hello",[](){
            std::cout << "Hello\n";
        });

        // Add an additional menu window
        //viewer.ngui->addWindow(Eigen::Vector2i(220,10),"New Window");

        // Expose the same variable directly ...
        //viewer.ngui->addVariable("float",floatVariable);

        // Generate menu
        viewer.screen->performLayout();

        return false;
    }; //...viewerr menu

    // Don't show face edges
    viewer.core.show_lines = false;
    // Show mesh
    viewer.data.set_mesh(V, F);
    // Calculate normals on launch
    acq::recalcNormals(kNeighbours, V, viewer);

    // Start viewer
    viewer.launch();

    return 0;
} //...main()
