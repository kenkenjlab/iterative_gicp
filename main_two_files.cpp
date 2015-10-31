#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include "iterative_gicp.hpp"

bool binary_(true), inv_transform_(false);
boost::filesystem::path src_filepath_, tgt_filepath_, output_filepath_;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;
unsigned int k_(100), itr_align_(50), itr_icp_(3);

int parseArguments(int argc, char** argv);
int loadPointCloud(const boost::filesystem::path &path, CloudT &cloud);
int savePointCloud(const boost::filesystem::path &path, const CloudT &cloud);
void showHelp() {
	std::printf("Help\n");
	std::printf("==============\n");
	std::printf("COMMAND_NAME [-option] src tgt\n");
}


int main(int argc, char** argv) {
	if(parseArguments(argc, argv)) { return 1; }
	std::printf("Correspondence Randomness: %u\n", k_);
	std::printf("Max Iterations in One ICP: %u\n", itr_align_);
	std::printf("ICP Iterations: %u\n", itr_icp_);
	if(inv_transform_) { std::printf("Inverse Transformation Mode\n"); }

	// Load point clouds
	CloudT::Ptr src_cloud_(new CloudT), tgt_cloud_(new CloudT), output_cloud_(new CloudT);
	std::cout << "Source: ";
	if(loadPointCloud(src_filepath_, *src_cloud_)) { return 1; }
	std::cout << "Target: ";
	if(loadPointCloud(tgt_filepath_, *tgt_cloud_)) { return 2; }

	// Registration
	IterativeGICP<PointT> igicp;
	igicp.setCorrespondenceRandomness(k_);
	igicp.setMaximumIterations(itr_align_);
	igicp.setGICPIteration(itr_icp_);
	igicp.setInputSource(src_cloud_);
	igicp.setInputTarget(tgt_cloud_);
	igicp.setInverseTransformationMode(inv_transform_);
	std::cout << "Alignment started.\n";
	igicp.align(*output_cloud_);

	// Save
	savePointCloud(output_filepath_, *output_cloud_);

	return 0;
}

int parseArguments(int argc, char** argv) {
	if(argc < 3) { showHelp(); return 1; }

	// Filepaths
	src_filepath_ = argv[argc - 2];
	tgt_filepath_ = argv[argc - 1];
	output_filepath_ = tgt_filepath_;
	output_filepath_ = tgt_filepath_.parent_path() / (src_filepath_.stem().string() + "_" + tgt_filepath_.stem().string() + tgt_filepath_.extension().string());

	// Parameters
	pcl::console::parse_argument(argc, argv, "-k", k_);
	pcl::console::parse_argument(argc, argv, "-itr_align", itr_align_);
	pcl::console::parse_argument(argc, argv, "-itr_icp", itr_icp_);
	inv_transform_ = pcl::console::find_argument(argc, argv, "-inv_transform") != -1;
	return 0;
}

int loadPointCloud(const boost::filesystem::path &path, CloudT &cloud) {
	std::cout << "Loading \"" << path.string() << "\"...";
	int ret = pcl::io::loadPCDFile(path.string(), cloud);
	if(ret) { return 1; }
	std::cout << "done\n";
	return 0;
}

int savePointCloud(const boost::filesystem::path &path, const CloudT &cloud) {
	std::cout << "Saving as \"" << path.string() << "\"...";
	int ret = pcl::io::savePCDFile(path.string(), cloud, binary_);
	if(ret) { return 1; }
	std::cout << "done\n";
	return 0;
}