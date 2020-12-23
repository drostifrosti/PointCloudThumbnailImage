#include <chrono>
#include <iostream>
#include <stdlib.h>

#include <ApproxMVBB/ComputeApproxMVBB.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>

int main(int argc, char** argv)
{
	auto t1 = std::chrono::high_resolution_clock::now();

	pcl::PointCloud<pcl::PointXYZ> cloud;
	int res = pcl::io::loadPCDFile("_t_kreuz_oben.asc", cloud);

	if (res != 0)
	{
		std::cout << "ERROR: Could not read point cloud file." << std::endl;
		return EXIT_FAILURE;
	}
	
	std::cout << "SUCCESS: Point cloud file read." << std::endl;
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloudMat = cloud.getMatrixXfMap().topRows(3);
	std::cout << "Rows: " << cloudMat.rows() << ", columns: " << cloudMat.cols() << std::endl;

	ApproxMVBB::Matrix3Dyn points(cloudMat.cast<double>());
	// TODO: raises error in debug mode
	ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points, 0.001);
	std::cout << "Object oriented bounding box computed" << std::endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud(&cloud);
	viewer->addPointCloud<pcl::PointXYZ>(ptrCloud);
	viewer->addCube((oobb.m_q_KI * oobb.center()).cast<float>(), oobb.m_q_KI.cast<float>(), oobb.extent().x(), oobb.extent().y(), oobb.extent().z());
	viewer->setShowFPS(false);
	viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
	Eigen::Vector3d centerBB = oobb.m_q_KI * oobb.center();
	Eigen::Vector3d yBB = oobb.m_q_KI * Eigen::Vector3d::UnitY();
	Eigen::Vector3d xBB = oobb.m_q_KI * Eigen::Vector3d::UnitX();
	double dist = 1000.;
	viewer->setCameraPosition(centerBB.x() + dist * yBB.x(), centerBB.y() + dist * yBB.y(), centerBB.z() + dist * yBB.z(),
		centerBB.x(), centerBB.y(), centerBB.z(), xBB.x(), xBB.y(), xBB.z());
	viewer->spin();
	viewer->saveScreenshot("screenshot.png");

	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	std::cout << "Duration: " << duration << "ms" << std::endl;

	return EXIT_SUCCESS;
}
