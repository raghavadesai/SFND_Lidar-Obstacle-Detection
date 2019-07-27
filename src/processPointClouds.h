// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
//#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
    PointT point;
    int id;
    Node<PointT> *left;
    Node<PointT> *right;

    Node(PointT arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL)
    {
    }
};

template <typename PointT>
struct KdTree
{
    Node<PointT> *root;

    KdTree()
        : root(NULL)
    {
    }

    void insertHelper(Node<PointT> **node, uint depth, PointT point, int id)
    {
        if (*node == NULL)
        {
            *node = new Node<PointT>(point, id);
        }
        else
        {
            /* current deimesnion at this depth level */
            uint cd = depth % 3;

            switch(cd)
            {
                case 0:
                    if (point.x < ((*node)->point.x))
                        insertHelper(&((*node)->left), depth + 1, point, id);
                    else
                        insertHelper(&((*node)->right), depth + 1, point, id);
                    break;
                case 1:
                    if (point.y < ((*node)->point.y))
                        insertHelper(&((*node)->left), depth + 1, point, id);
                    else
                        insertHelper(&((*node)->right), depth + 1, point, id);
                    break;
                case 2:
                    if (point.z < ((*node)->point.z))
                        insertHelper(&((*node)->left), depth + 1, point, id);
                    else
                        insertHelper(&((*node)->right), depth + 1, point, id);
                    break;
                default:
                    break;
            }
        }
    }

    void insert(PointT point, int id)
    {
        insertHelper(&root, 0, point, id);
    }
    // return a list of point ids in the tree that are within distance of target

    void searchHelper(PointT target, Node<PointT> *node, int depth, float distanceTol, std::vector<int> &ids)
    {
        if (node != NULL)
        {
            if ((node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol)) &&
                (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol)) &&
                (node->point.z >= (target.z - distanceTol)) && (node->point.z <= (target.z + distanceTol)))
            {
                float distance = sqrt( (node->point.x - target.x) * (node->point.x - target.x) + 
                                       (node->point.y - target.y) * (node->point.y - target.y) +
                                       (node->point.z - target.z) * (node->point.z - target.z) );

                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }
            //Check across boundary
            /* current deimesnion at this depth level */
            uint cd = depth % 3;
            switch (cd)
            {
            case 0:
                if ((target.x - distanceTol) < node->point.x)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.x + distanceTol) > node->point.x)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
                break;
            case 1:
                if ((target.y - distanceTol) < node->point.y)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.y + distanceTol) > node->point.y)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
                break;
            case 2:
                if ((target.z - distanceTol) < node->point.z)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.z + distanceTol) > node->point.z)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
                break;
            default:
                break;
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target

    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};



template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::unordered_set<int> &cluster, std::vector<bool> &processed, KdTree<PointT> *tree, float distanceTol);
};
#endif /* PROCESSPOINTCLOUDS_H_ */