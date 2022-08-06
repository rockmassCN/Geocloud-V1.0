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

	//��ȡ��Ҫ����ĵ���
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

	cout << "Input the Voxel Size��";
	float v;
	cin >> v;
	cout << "Voxel Filtering...\n";
	t3 = GetTickCount();
	//�����˲�
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud1);
	sor.setLeafSize(v, v, v);
	sor.filter(*cloud2);
	t4 = GetTickCount();
	std::cerr << "After Voxel Filtered: " << cloud2->width * cloud2->height
		<< " Points (" << pcl::getFieldsList(*cloud2) << ")," << "Use Time:" << (t4 - t3) << "(ms).\n";

	//���������ʹ�pointcloud2ת��Ϊpointxyz
	pcl::fromPCLPointCloud2(*cloud2, *cloud);

	//�����˲���ĵ���
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
	//���������ķ�ʽ��PCA����
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

	//�������<�㣬����>
	int k2, k3;
	cout << "Input the MinClusterSize:";
	cin >> k2;
	cout << "Input the NumberOfNeighbours:";
	cin >> k3;

	t9 = GetTickCount();
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(k2);  //��С�ľ���ĵ���
    //reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);  //����
	reg.setSearchMethod(tree);    //������ʽ
	reg.setNumberOfNeighbours(k3);    //���������������ĸ���
	//reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);         //�����
	reg.setInputNormals(normals);     //����ķ���
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);     //�������ʵ���ֵ

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	t10 = GetTickCount();
	cout << "Region Growed," << "Use Time:" << (t10 - t9) << "(ms).\n";

	cout << "Extracted:" << clusters.size() << "planes" << std::endl;

	int m = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		//�����µĵ������ݼ�cloud_cluster�������е�ǰ����д�뵽�������ݼ��� 

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cluster->points.push_back(cloud->points[*pit]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		//���������
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Saving Results..." << "There are " << cluster->points.size() << " ponints " << "in " << m + 1 << " plane." << std::endl;
		//string FolderPath = "D:/DailyStudy/extractplane/data/" + src2 + "��results";
		string FolderPath = "D:/test/res/" + src2 + "��results"; 
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
	//���ӻ�����Ľ��
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
	/*pcl::PCDWriter writer2;//������д�����
	writer2.writeBinary(res, *colored_cloud, false);//�ĳ���Ҫ����ĵ�������*/
	pcl::io::savePCDFileBinary(res, *colored_cloud);
	t12 = GetTickCount();
	cout << "Saved,Use Time:" << (t12 - t11) << "(ms).\n";

	t = (t2 - t1) + (t4 - t3) + (t6 - t5) + (t8 - t7) + (t10 - t9) + (t12 - t11);
	cout << "Total Time��" << t << "(ms).\n";
	getchar();
	return (0);
}