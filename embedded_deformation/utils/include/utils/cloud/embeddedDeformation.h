/*
*   embedded deformation implementation
*   by R. Falque
*   14/11/2018
*/

#ifndef EMBEDDED_DEFORMATION_H
#define EMBEDDED_DEFORMATION_H

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>

#include "embeddedDeformationOPtions.h"
#include "embeddedDeformationCostFunction.h"

//#include "libGraphCpp/graph.hpp"
#include "utils/graph/graph.h"
//#include "libGraphCpp/plotGraph.hpp"
#include "utils/visualization/plotGraph.h"

#include "utils/cloud/downsampling.h"

class EmbeddedDeformation
{
public:

	// provide the mesh and the graph
	EmbeddedDeformation(Eigen::MatrixXd V_in, 
						 Eigen::MatrixXi F_in,
						 Eigen::MatrixXd N_in, 
						 Eigen::MatrixXi E_in,
						 embeddedDeformationOptions opts);

	// provide the mesh
	EmbeddedDeformation(Eigen::MatrixXd V_in, 
						 Eigen::MatrixXi F_in,
						 double grid_resolution,
						 int k);

	EmbeddedDeformation(Eigen::MatrixXd V_in, 
						 Eigen::MatrixXi F_in,
						 embeddedDeformationOptions opts);

	// provide only a point cloud
	EmbeddedDeformation(Eigen::MatrixXd V_in, 
						 double grid_resolution,
						 int k);

	EmbeddedDeformation(Eigen::MatrixXd V_in, 
						 embeddedDeformationOptions opts);

	// destructor
	~EmbeddedDeformation(){
		if(deformation_graph_ptr_ != nullptr) delete deformation_graph_ptr_;
	}

	void deform(Eigen::MatrixXd sources_in, Eigen::MatrixXd targets_in, Eigen::MatrixXd & V_deformed);
	void deform_other_points(Eigen::MatrixXd & V);
	void update_normals(Eigen::MatrixXd & normals);
	void show_deformation_graph();

private:
	// by order of appearance:
	double w_rot_ = 1;
	double w_reg_ = 10;
	double w_rig_ = 10;
	double w_con_ = 100;

	// options
	bool use_knn_;
	bool use_dijkstra_;
	bool use_farthest_sampling_ = false;
	bool verbose_;
	int nodes_connectivity_;
	bool transpose_input_and_output_;

	Eigen::MatrixXd V_;
	Eigen::MatrixXi F_;
	
	libgraphcpp::Graph* deformation_graph_ptr_ = nullptr;;

	// graph properties definition
	std::vector<Eigen::Matrix3d> rotation_matrices_;
	std::vector<Eigen::Vector3d> translations_;

	// specific for searching on the geodesic distance
	std::vector<int> indexes_of_deformation_graph_in_V_;
};

#endif
