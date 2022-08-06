#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <direct.h>

using namespace std;

int main(int argc, char** argv)
{
	DWORD t, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;

	pcl::PCLPointCloud2::Ptr cloud1(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//读取需要处理的点云
	string src, src1, src2;
	//src1 = "D:/DailyStudy/extractplane/data/src/";
	src1 = "D:/test/src/";
	cout << "Input cloud:";
	cin >> src2;
	src = src1 + src2;

	cout << "Loading...\n";
	t1 = GetTickCount();
	pcl::PCDReader reader;
	if (reader.read(src, *cloud1) == -1) {
		cout << "Cloud Loading Failed.\n" << std::endl;
		return (-1);
	}
	else {
		t2 = GetTickCount();
		std::cerr << "Before Voxel Filtered: " << cloud1->width * cloud1->height
			<< " Points (" << pcl::getFieldsList(*cloud1) << ").\n";
		cout << "Loading Successful,Use Time:" << (t2 - t1) << "(ms)." << "\n";
	}

	cout << "Input the Voxel Size：";
	float v;
	cin >> v;
	cout << "Voxel Filtering...\n";
	t3 = GetTickCount();
	//体素滤波
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud1);
	sor.setLeafSize(v, v, v);
	sor.filter(*cloud2);
	t4 = GetTickCount();
	std::cerr << "After Voxel Filtered: " << cloud2->width * cloud2->height
		<< " Points (" << pcl::getFieldsList(*cloud2) << ")," << "Use Time:" << (t4 - t3) << "(ms).\n";

	//将点云类型从pointcloud2转换为pointxyz
	pcl::fromPCLPointCloud2(*cloud2, *cloud);

	//保存滤波后的点云
	cout << "Saving the Filtered Cloud.\n";
	string srcf, srcf1, srcf2;
	srcf1 = "D:/test/src/";
	cout << "Naming the Filtered Cloud:";
	cin >> srcf2;
	srcf = srcf1 + srcf2;
	t5 = GetTickCount();
	/*pcl::PCDWriter writer1;
	writer1.write<pcl::PointXYZ>(srcf, *cloud, false);*/
	pcl::io::savePCDFileBinary(srcf, *cloud);
	t6 = GetTickCount();
	cout << "Saved," << "Use Time:" << (t6 - t5) << "(ms).\n";

	cout << "PCA Sloving normal.\n";
	//设置搜索的方式，PCA求法线
	cout << "Input the Value of KSearch.:";
	int k1;
	cin >> k1;
	t7 = GetTickCount();
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(k1);
	//normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);
	t8 = GetTickCount();
	cout << "Sloved," << "Use Time:" << (t8 - t7) << "(ms).\n";
	cout << "Region Growing.\n";

	//聚类对象<点，法线>
	int k2, k3;
	cout << "Input the MinClusterSize:";
	cin >> k2;
	cout << "Input the NumberOfNeighbours:";
	cin >> k3;

	t9 = GetTickCount();
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(k2);  //最小的聚类的点数
    //reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);  //最大的
	reg.setSearchMethod(tree);    //搜索方式
	reg.setNumberOfNeighbours(k3);    //设置搜索的邻域点的个数
	//reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);         //输入点
	reg.setInputNormals(normals);     //输入的法线
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);     //设置曲率的阈值

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	t10 = GetTickCount();
	cout << "Region Growed," << "Use Time:" << (t10 - t9) << "(ms).\n";

	cout << "Extracted:" << clusters.size() << "planes" << std::endl;

	int m = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		//创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中 

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cluster->points.push_back(cloud->points[*pit]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		//保存聚类结果
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Saving Results..." << "There are " << cluster->points.size() << " ponints " << "in " << m + 1 << " plane." << std::endl;
		//string FolderPath = "D:/DailyStudy/extractplane/data/" + src2 + "’results";
		string FolderPath = "D:/test/res/" + src2 + "’results"; 
		if (0 != _access(FolderPath.c_str(), 0))
		{
			_mkdir(FolderPath.c_str());
		}
		std::stringstream ss;
		string resc = FolderPath + "/";
		ss << resc << m + 1 << ".pcd";
		pcl::io::savePCDFileBinary(ss.str(), *cluster);
		m++;
	}

	cout << "Visualization...\n" << "\n";
	//可视化聚类的结果
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("RG_Results");
	viewer.showCloud(colored_cloud);
	cout << "Close the Windows and Save the Results.\n" << "\n";
	while (!viewer.wasStopped())
	{
	}
	string res, res1, res2;
	//res1 = "D:/DailyStudy/extractplane/data/res/";
	res1 = "D:/test/res/";
	cout << "Naming the Processed Cloud:";
	cin >> res2;
	res = res1 + res2;
	cout << "Saving...\n";
	t11 = GetTickCount();
	/*pcl::PCDWriter writer2;//将点云写入磁盘
	writer2.writeBinary(res, *colored_cloud, false);//改成想要输出的点云名称*/
	pcl::io::savePCDFileBinary(res, *colored_cloud);
	t12 = GetTickCount();
	cout << "Saved,Use Time:" << (t12 - t11) << "(ms).\n";

	t = (t2 - t1) + (t4 - t3) + (t6 - t5) + (t8 - t7) + (t10 - t9) + (t12 - t11);
	cout << "Total Time：" << t << "(ms).\n";
	getchar();
	return (0);
}