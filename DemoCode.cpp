#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <cstdlib>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.6f %6.6f %6.6f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.6f %6.6f %6.6f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.6f %6.6f %6.6f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.6f, %6.6f, %6.6f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointT,PointT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;
bool next_iteration = false;

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}


int
main (int argc, char** argv)
{
  //pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_blob (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGBNormal>), cloud_f (new       pcl::PointCloud<pcl::PointXYZRGBNormal>), cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>), cloud1 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  PointCloudT::Ptr scene_original (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);
  PointCloudT::Ptr original_cad (new PointCloudT);
  PointCloudT::Ptr transformed_cad (new PointCloudT);
  PointCloudT::Ptr scene_transformed (new PointCloudT);
  PointCloudT::Ptr transformed_cloud (new PointCloudT);

  // Fill in the cloud data
  
  pcl::io::loadPCDFile<PointT> (argv[2], *object);
  int iterations = atoi (argv[3]);
  *original_cad=*object; 
  
  pcl::io::loadPCDFile (argv[1], *cloud);
  Eigen::Matrix4d transformation_matrix_kinect_to_base = Eigen::Matrix4d::Identity ();
  transformation_matrix_kinect_to_base(0,0)=-0.013248;
	transformation_matrix_kinect_to_base(0,1)=-0.996764;
	transformation_matrix_kinect_to_base(0,2)=-0.079287;
	transformation_matrix_kinect_to_base(0,3)=1.415635;
	transformation_matrix_kinect_to_base(1,0)=-0.994990;
	transformation_matrix_kinect_to_base(1,1)=0.021016;
	transformation_matrix_kinect_to_base(1,2)=-0.096709;
	transformation_matrix_kinect_to_base(1,3)=0.162094;
	transformation_matrix_kinect_to_base(2,0)=0.098131;
	transformation_matrix_kinect_to_base(2,1)=0.077697;
	transformation_matrix_kinect_to_base(2,2)=-0.992036;
	transformation_matrix_kinect_to_base(2,3)=1.478825;
	transformation_matrix_kinect_to_base(3,0)=0.000000;
	transformation_matrix_kinect_to_base(3,1)=0.000000;
	transformation_matrix_kinect_to_base(3,2)=0.000000;
	transformation_matrix_kinect_to_base(3,3)=1.000000;
  pcl::transformPointCloud (*cloud, *cloud1, transformation_matrix_kinect_to_base); 
  

  pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
  pass.setInputCloud (cloud1);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.2, 1.5);
  
  pass.filter (*cloud_filtered);
  
  

  *scene = *cloud_filtered;
  pcl::io::savePCDFileASCII ("Dome_wrt_Kinect.pcd", *scene);
  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointT> grid;
  const float leaf = 0.05f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  
  grid.filter (*object); //original is *object

  grid.setInputCloud (scene);
  
  grid.filter (*scene); //original is *scene
  
  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointT,PointT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene); //original is scene
  nest.compute (*scene); //origiinal is *scene
  
  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object); //original is object
  fest.setInputNormals (object); //original is object 
  fest.compute (*object_features); 
  fest.setInputCloud (scene); //original is scene
  fest.setInputNormals (scene);//original is scene
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointT,PointT,FeatureT> align;
  align.setInputSource (object); //original is object
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene); //original is scene
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (25); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (32); // Number of nearest features to use
  align.setSimilarityThreshold (0.09f); // Polygonal edge length similarity threshold
  //align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4d transformation = align.getFinalTransformation ().cast<double>();
    std::cout << transformation << std::endl;
    
    
    //pcl::console::print_info ("    | %6.6f %6.6f %6.6f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    //pcl::console::print_info ("R = | %6.6f %6.6f %6.6f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    //pcl::console::print_info ("    | %6.6f %6.6f %6.6f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    //pcl::console::print_info ("\n");
    //pcl::console::print_info ("t = < %0.6f, %0.6f, %0.6f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    //pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
    pcl::io::savePCDFileASCII ("Transformed_Cad_File.pcd", *object_aligned);
    pcl::transformPointCloud (*original_cad, *transformed_cad, transformation);

  }

  Eigen::Matrix4d transformation1 = align.getFinalTransformation ().cast<double>();
  

// The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
  PointCloudT::Ptr cloud_icp_1 (new PointCloudT);
  PointCloudT::Ptr source_cloud (new PointCloudT);
  //PointCloudT::Ptr transformed_cloud (new PointCloudT);
  
  // Checking program arguments
  



  pcl::console::TicToc time;
  time.tic ();
  *cloud_in=*cloud_filtered; //target
  *cloud_icp=*transformed_cad; //source
   
  
   

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  *cloud_tr = *cloud_icp;

 

   time.tic ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (cloud_in); //original is cloud_icp
  icp.setInputTarget (cloud_icp); //original is cloud_in
  icp.align (*cloud_in); //original is *cloud_icp
  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
  
  //transformation_matrix =  
  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    //print4x4Matrix (transformation_matrix);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }
   
  

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  
    if (next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic ();
      icp.align (*cloud_in); //original is *cloud_icp
      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

      if (icp.hasConverged ())
      {
        printf ("\033[11A");  // Go up 11 lines in terminal output.
        printf ("\nICP has converged, score is %6.4lf\n", icp.getFitnessScore ());
        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
        //print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

        ss.str ("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str ();
        }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
      }
    }
    next_iteration = false;
  



  *source_cloud=*cloud_tr; 
  std::cout << transformation_matrix << std::endl;


  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transformation_matrix);
   
   
   std::cout << transformation1 << std::endl;
   //Eigen::Matrix4d transformation ;
   Eigen::Matrix4d Final_Transformation = transformation_matrix*transformation1;
   print4x4Matrix (Final_Transformation);

  

  // Visualization
  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> source_cloud_color_handler (original_cad, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (original_cad, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> transformed_cad_color_handler (transformed_cloud, 255, 0, 0); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cad_color_handler, "transformed_cad");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> scene_transformed_color_handler (cloud_filtered, 0, 0, 255); // Blue
  viewer.addPointCloud (cloud_filtered, scene_transformed_color_handler, "scene_transformed");
  pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return (0);

}
