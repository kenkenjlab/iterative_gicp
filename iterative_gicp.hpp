#ifndef ITERATIVE_GICP_VERSION
#define ITERATIVE_GICP_VERSION 2015011601

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>

template <class PointT>
class IterativeGICP : public pcl::GeneralizedIterativeClosestPoint<PointT, PointT> {
private:
	typedef pcl::PointCloud<PointT> CloudT;

	//// Private Fields ////
	unsigned int itr_gicp_;
	bool inv_transformation_;
	Eigen::Matrix4f transformation_matrix_;

public:
	IterativeGICP()
		: itr_gicp_(3)
		, inv_transformation_(false)
	{}

	// Setter
	inline void setGICPIteration(unsigned int iteration) { itr_gicp_ = iteration; }
	inline void setInverseTransformationMode(bool mode = true) { inv_transformation_ = mode; }
	
	// Compute
	void align(CloudT &output);
	
	// 
};


template <class PointT>
void IterativeGICP<PointT>::align(CloudT &output) {
	transformation_matrix_ = Eigen::Matrix4f::Identity();
	output = *getInputSource();
	for(unsigned int i = 0; i < itr_gicp_; i++) {
		setInputSource(output.makeShared());
		GeneralizedIterativeClosestPoint<PointT, PointT>::align(output);
		transformation_matrix_ = getFinalTransformation() * transformation_matrix_;
		pcl::transformPointCloud(output, output, getFinalTransformation());		// This line is required because GICP in PCL 1.7.2 has a bug which is mentioned at https://github.com/PointCloudLibrary/pcl/issues/818.
	}
	output += *getInputTarget();
	if(inv_transformation_) {
		Eigen::Matrix4f inv_transformation_matrix = transformation_matrix_.inverse();
		pcl::transformPointCloud(output, output, inv_transformation_matrix);
	}
}


#endif