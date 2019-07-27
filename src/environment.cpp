/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void lidarObstacleDetection(pcl::visualization::PCLVisualizer::Ptr &viewer,
                            ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloudI)
{
    //1. Downsample the cloud data using FilterCloud
    //renderPointCloud(viewer, inputCloudI, "inputCloudI", Color(1, 1, 0));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloudI = pointProcessorI->FilterCloud(inputCloudI, 0.3, Eigen::Vector4f(-10.0, -6.0, -2, 1), Eigen::Vector4f(25, 7, 10, 1));
    std::cout << "Done Downsampling/Filtering" << std::endl;
    //renderPointCloud(viewer, filterCloudI, "filteredCloudI", Color(0, 1, 0));

    //2. Segmement the road plane from the rest of the objects
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloudI = pointProcessorI->SegmentPlane(filterCloudI, 100, 0.2);
    std::cout << "Done Segmentation" << std::endl;

    //3. Render the Street Plane
    renderPointCloud(viewer, segmentCloudI.second, "planeCloud", Color(0, 1, 1));
    renderPointCloud(viewer, segmentCloudI.first, "ObsCloud", Color(1, 1, 0));

    //4. Cluster the points into objects
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusterI = pointProcessorI->Clustering(segmentCloudI.first, 0.4, 8, 300);
    std::cout << "Done Clustering" << std::endl;

    //5. Render the Clusters
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusterI)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + clusterId, colors[clusterId]);

        //6. Render Bounding Boxes to encompass each cluseter object
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    pcl::PointXYZI minPoint, maxPoint;
    pcl::PointCloud<pcl::PointXYZI> ::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloud,"inputCloud");
    pcl::getMinMax3D(*inputCloud, minPoint, maxPoint);
    std::cout <<"minPoint:" << minPoint << std::endl;
    std::cout <<"maxPoint:" << maxPoint << std::endl;
    pcl::PointCloud<pcl::PointXYZI> ::Ptr  filterCloud = pointProcessorI.FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-20.0,-10.0,-5,1), Eigen::Vector4f(20,10,5,1));
    renderPointCloud(viewer,filterCloud,"filterCloud");

}

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar =new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderPointCloud(viewer,inputCloud,"blue");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloudI = pointProcessor.SegmentPlane(inputCloud,1000,0.4);
    renderPointCloud(viewer, segmentCloudI.first, "ObstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloudI.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>cloudClusterI = pointProcessor.Clustering(segmentCloudI.first,1, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusterI)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+clusterId,colors[clusterId]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box, clusterId);
        ++clusterId;
    }

  
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)


    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        //Clear out the viewer with past iteration renders
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        //Load a frame of pcd
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        //Run Lidar Obstacle Detection Pipeline
        lidarObstacleDetection( viewer, pointProcessorI, inputCloudI );

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    } 
}