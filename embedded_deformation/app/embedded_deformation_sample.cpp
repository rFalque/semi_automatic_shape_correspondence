/**
 * Author: R. Falque
 * 
 * main for testing the embedded deformation implementation
 * by R. Falque
 **/

// dependencies
#include "utils/IO_libIGL/readOBJ.h"
#include "utils/IO/readPLY.h"
#include "utils/IO/writePLY.h"
#include "utils/IO/readCSV.h"

#include "utils/IO/readGraphOBJ.h"
#include "utils/visualization/polyscopeWrapper.h"

#include "utils/cloud/embeddedDeformation.h"
#include "utils/visualization/plotMesh.h"
#include "utils/visualization/plotCloud.h"
#include "utils/cloud/embeddedDeformationOPtions.h"
#include "utils/mesh/computeNormals.h"

#include "utils/eigenTools/nanoflannWrapper.h"

#include <yaml-cpp/yaml.h>
#include <tuple>

void plot_mesh_and_cloud(Mesh mesh, Mesh cloud) {
    visualization::init();
    visualization::add_mesh(mesh.V, mesh.F, "deformed mesh");
    visualization::set_mesh_color(Eigen::Vector3d{1.0, 0.0, 0.0}, "deformed mesh");
    visualization::add_cloud(cloud.V, "target cloud");
    visualization::show();
    visualization::clear_structures();
}

double get_scale(Eigen::MatrixXd handles_source_eigen, Eigen::MatrixXd handles_target_eigen) {
    double scale_source = 0;
    int counter_source = 0;
    for (int i=0; i<handles_source_eigen.rows(); i++)
        for (int j=0; j<handles_source_eigen.rows(); j++)
            if (i!=j) {
            counter_source ++;
            scale_source += (handles_source_eigen.row(i) - handles_source_eigen.row(j)).norm();
            }
    scale_source /= counter_source;

    double scale_target = 0;
    int counter_target = 0;
    for (int i=0; i<handles_target_eigen.rows(); i++)
        for (int j=0; j<handles_target_eigen.rows(); j++)
            if (i!=j) {
            counter_target ++;
            scale_target += (handles_target_eigen.row(i) - handles_target_eigen.row(j)).norm();
            }
    scale_target /= counter_target;

    return scale_target / scale_source;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> find_fine_correspondences(Mesh mesh_template, Mesh cloud_target, double correspondences_distance_threshold) {

    // fine deformation
    std::cout << "start searching for correspondences\n";
    int skip = 10;
    nanoflann_wrapper kdtree_source_mesh(mesh_template.V);
    nanoflann_wrapper kdtree_target_cloud(cloud_target.V);
    std::vector<int> deformation_handle_ids;
    std::vector<Eigen::Vector3d> deformation_handle_target;
    for (int vertex_id=0; vertex_id<mesh_template.V.cols(); vertex_id=vertex_id+skip) {
        // search for closest point from the source mesh on the target cloud
        std::vector<int> point_index_target;
        point_index_target = kdtree_target_cloud.return_k_closest_points(mesh_template.V.col(vertex_id), 1);

        // search for closest point from the target cloud on the source mesh
        std::vector<int> point_index_source;
        point_index_source = kdtree_source_mesh.return_k_closest_points(cloud_target.V.col(point_index_target[0]), 1);

        // if the distance is under a threshold stack it to the the deformation handles
        if ((mesh_template.V.col(vertex_id) - mesh_template.V.col(point_index_source[0])).norm() < correspondences_distance_threshold) {
            // if the dot product of the normals is negative, the point is not a good correspondence
            if (mesh_template.N.col(vertex_id).dot(cloud_target.N.col(point_index_target[0])) > 0) {
                deformation_handle_ids.push_back(vertex_id);
                deformation_handle_target.push_back( cloud_target.V.col(point_index_target[0]) );
            }
        }
    }

    Eigen::MatrixXd handles_source_eigen_dense = Eigen::MatrixXd::Zero(3, deformation_handle_ids.size());
    Eigen::MatrixXd handles_target_eigen_dense = Eigen::MatrixXd::Zero(3, deformation_handle_ids.size());
    for (int i=0; i<deformation_handle_ids.size(); i++) {
        handles_source_eigen_dense.col(i) = mesh_template.V.col(deformation_handle_ids.at(i));
        handles_target_eigen_dense.col(i) = deformation_handle_target.at(i);
    }

    return  {handles_source_eigen_dense, handles_target_eigen_dense};
}

int main(int argc, char* argv[])
{
    // load settings
    bool visualization = true;
    embeddedDeformationOptions opts;
    opts.loadYAML("../config.yaml");
    
    // load the template and the cloud target
    Mesh mesh_template = readPLY(opts.path_source_file);
    Mesh cloud_target = readPLY(opts.path_target_file);
    std::cout << "Mesh loaded\n";

    // compute normals
    mesh_template.N = compute_vertices_normals(mesh_template.V, mesh_template.F);
    cloud_target.N = compute_vertices_normals(cloud_target.V, cloud_target.F);
    std::cout << "Normals computed\n";

    // load deformation handles
    Eigen::MatrixXd handles_source_eigen = read_csv<Eigen::MatrixXd>(opts.path_source_annotations);
    Eigen::MatrixXd handles_target_eigen = read_csv<Eigen::MatrixXd>(opts.path_target_annotations);
    std::cout << "Deformation handles loaded\n";


    // plot the correspondences
    if (visualization) {
        visualization::init();
        visualization::add_cloud(cloud_target.V, "target cloud");
        visualization::add_mesh(mesh_template.V, mesh_template.F, "template mesh");
        visualization::set_mesh_color(Eigen::Vector3d{1.0, 0.0, 0.0}, "template mesh");
        visualization::add_vectors(handles_source_eigen.transpose(), handles_target_eigen.transpose(), "correspondences");
        visualization::show();
        visualization::clear_structures();
    }

    // get the scale
    double ratio = get_scale(handles_source_eigen, handles_target_eigen);

    if (visualization)
        plot_mesh_and_cloud(mesh_template, cloud_target);

    // test with scale update
    mesh_template.V *= ratio;
    handles_source_eigen *= ratio;


    // coarse deformation
    EmbeddedDeformation* non_rigid_deformation_ED_coarse;
    non_rigid_deformation_ED_coarse = new EmbeddedDeformation(mesh_template.V, opts);
    non_rigid_deformation_ED_coarse->deform(handles_source_eigen, handles_target_eigen, mesh_template.V);

    if (visualization) {
        non_rigid_deformation_ED_coarse->show_deformation_graph();
        plot_mesh_and_cloud(mesh_template, cloud_target);
    }

    auto [handles_source_eigen_dense, handles_target_eigen_dense] = find_fine_correspondences(mesh_template, cloud_target, opts.correspondences_threshold);
    //auto [handles_target_eigen_dense, handles_source_eigen_dense] = find_fine_correspondences(cloud_target, mesh_template, opts.correspondences_threshold);

    // plot the correspondences
    if (visualization) {
        visualization::init();
        visualization::add_cloud(cloud_target.V, "target cloud");
        visualization::add_mesh(mesh_template.V, mesh_template.F, "template mesh");
        visualization::set_mesh_color(Eigen::Vector3d{1.0, 0.0, 0.0}, "template mesh");
        visualization::add_vectors(handles_source_eigen_dense, handles_target_eigen_dense, "correspondences");
        visualization::show();
        visualization::clear_structures();
    }

    // fine deformation
    EmbeddedDeformation* non_rigid_deformation_ED_dense;
    non_rigid_deformation_ED_dense = new EmbeddedDeformation(mesh_template.V, opts);
    non_rigid_deformation_ED_dense->deform(handles_source_eigen_dense, handles_target_eigen_dense, mesh_template.V);

    if (visualization)
        plot_mesh_and_cloud(mesh_template, cloud_target);


    auto [handles_source_eigen_dense2, handles_target_eigen_dense2] = find_fine_correspondences(mesh_template, cloud_target, opts.correspondences_threshold);
    // fine deformation
    opts.w_con = 50;
    EmbeddedDeformation* non_rigid_deformation_ED_dense2;
    non_rigid_deformation_ED_dense2 = new EmbeddedDeformation(mesh_template.V, opts);
    non_rigid_deformation_ED_dense2->deform(handles_source_eigen_dense2, handles_target_eigen_dense2, mesh_template.V);


    if (visualization)
        plot_mesh_and_cloud(mesh_template, cloud_target);

    // save deformed mesh
    writePLY(opts.path_output_file, mesh_template, true);

    return 0;
}
