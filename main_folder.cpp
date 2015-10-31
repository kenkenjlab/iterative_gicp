#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <boost/filesystem.hpp>
#include "iterative_gicp.hpp"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;

bool binary_(true), inv_transform_(true);
std::vector<boost::filesystem::path> pcd_filepaths_;
std::string output_filename_("");
boost::filesystem::path input_dirpath_, output_dirpath_;
unsigned int k_(100), itr_align_(50), itr_icp_(3);
CloudT::Ptr src_cloud_(new CloudT), tgt_cloud_(new CloudT), output_cloud_(new CloudT);
pcl::console::TicToc tt_;

int parseArguments(int argc, char** argv);
int listupPcdFiles(const boost::filesystem::path &dirpath, std::vector<boost::filesystem::path> &filepaths);
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
	std::cout << std::endl;

	// Iterate
	bool first_iteration(true);
	for(int i = 0; i < pcd_filepaths_.size(); i++) {
		tt_.tic();
		const boost::filesystem::path &current_pcd_filepath(pcd_filepaths_[i]);
		const std::string current_label = current_pcd_filepath.stem().string();

		// Load
		std::printf("[%d/%d] %s: ", i, pcd_filepaths_.size(), current_pcd_filepath.string().c_str());
		if(loadPointCloud(current_pcd_filepath, *src_cloud_)) { continue; }
		if(first_iteration) {
			*tgt_cloud_ = *src_cloud_;
			first_iteration = false;
			tt_.toc_print();
			continue;
		}
		
		// Align
		IterativeGICP<PointT> igicp;
		igicp.setCorrespondenceRandomness(k_);
		igicp.setMaximumIterations(itr_align_);
		igicp.setGICPIteration(itr_icp_);
		igicp.setInputSource(src_cloud_);
		igicp.setInputTarget(tgt_cloud_);
		igicp.setInverseTransformationMode(inv_transform_);
		igicp.align(*output_cloud_);
		*tgt_cloud_ = *output_cloud_;

		// Save
		output_filename_ = current_label + "_merged.pcd";
		boost::filesystem::path output_filepath = output_dirpath_ / output_filename_;
		savePointCloud(output_filepath, *output_cloud_);

		tt_.toc_print();
	}
	std::cout << "\nAll done.\n";

	return 0;
}

int parseArguments(int argc, char** argv) {
	if(argc < 2) { showHelp(); return 1; }

	// Parse arguments
	input_dirpath_ = argv[argc - 1];
	output_dirpath_ = input_dirpath_ / "merged";
	boost::filesystem::create_directory(output_dirpath_);
	listupPcdFiles(input_dirpath_, pcd_filepaths_);

	// Parameters
	pcl::console::parse_argument(argc, argv, "-k", k_);
	pcl::console::parse_argument(argc, argv, "-itr_align", itr_align_);
	pcl::console::parse_argument(argc, argv, "-itr_icp", itr_icp_);
	inv_transform_ = pcl::console::find_argument(argc, argv, "-disable_inv_transform") == -1;
	return 0;
}

int listupPcdFiles(const boost::filesystem::path &dirpath, std::vector<boost::filesystem::path> &filepaths) {
	boost::system::error_code error;
    const bool result = boost::filesystem::exists(dirpath, error);
    if (!result || error) {
		// Not exist
		return 1;
	}
    BOOST_FOREACH(const boost::filesystem::path& p, std::make_pair(boost::filesystem::directory_iterator(dirpath), boost::filesystem::directory_iterator())) {
		if(p.extension().string() == ".pcd") {
			filepaths.push_back(p.string());
		}
    }
	return 0;
}

int loadPointCloud(const boost::filesystem::path &path, CloudT &cloud) {
	int ret = pcl::io::loadPCDFile(path.string(), cloud);
	if(ret) { return 1; }
	return 0;
}

int savePointCloud(const boost::filesystem::path &path, const CloudT &cloud) {
	int ret = pcl::io::savePCDFile(path.string(), cloud, binary_);
	if(ret) { return 1; }
	return 0;
}