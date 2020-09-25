#include <iostream>
#include <string.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

double compute_ICP(std::string file1, std::string file2);
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void show_result(){

  //input
  PointCloudT::Ptr cloud_org (new PointCloudT);  
  PointCloudT::Ptr cloud_map (new PointCloudT);  
  PointCloudT::Ptr cloud_icp (new PointCloudT);  

  pcl::io::loadPLYFile("/home/wei/桌面/pcltest/duck.ply",*cloud_org);
  pcl::io::loadPLYFile("/home/wei/桌面/pcltest/test.ply",*cloud_map);
  *cloud_icp = *cloud_map;
  
  //icp procedu:
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(20);
  icp.setInputSource (cloud_icp);
  icp.setInputTarget(cloud_org);
  icp.align(*cloud_icp);

  double icpScore = -1;

  if (icp.hasConverged ())
  {
    std::cout << "Compute the ICP for the two mesh and ICP finished" <<std::endl;
    icpScore = icp.getFitnessScore();
  }
  // compute a distance:
  // float test1 = pcl::geometry::squaredDistance(cloud_org,cloud_org);
  // float test2 = pcl::geometry::squaredDistance(cloud_org,cloud_map);
  // std::cout <<"test"<<test1 << "test2"<<test2 <<std::endl;


  // dispaly the result in the pcl viewer
  pcl::visualization::PCLVisualizer viewer ("ICP for duck");
  int v1 (0);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  // Deftin the color:
  float bckgr_gray_level = 0.0;
  float txt_gray_lvl = 1 - bckgr_gray_level;
  // Add org cloud:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_org, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                          (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_org, cloud_in_color_h, "cloud_org_v1", v1);
  std::string info;
  info = "White: Ground truth point cloud\nGreen: Mapping result point cloud\nICP scores: "+ std::to_string(icpScore) ;

  viewer.addText (info, 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);


  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  // Add mapping resuilt:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_icp, 20, 180, 20);
  viewer.addPointCloud (cloud_icp, cloud_tr_color_h, "cloud_map_v1", v1);


  while (!viewer.wasStopped ())
  {
   viewer.spinOnce ();
  }

}

void show_octomap(){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile("/home/wei/桌面/pcltest/modles/test2.pcd", cloud);

  octomap::ColorOcTree tree( 0.04 );
  for(auto p:cloud.points){
    tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
  }

  tree.updateInnerOccupancy();
  tree.write("/home/wei/桌面/pcltest/modles/test2.ot");

  std::cout<< "Done"<<std::endl;
}

void bounding_box()
{

  //input
  PointCloudT::Ptr cloud_org (new PointCloudT);  
  PointCloudT::Ptr cloud_map (new PointCloudT);  

  pcl::io::loadPLYFile("/home/wei/桌面/pcltest/duck.ply",*cloud_org);
  pcl::io::loadPLYFile("/home/wei/桌面/pcltest/test.ply",*cloud_map);

  // Compute the max/min axis:
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud_map, minPt, maxPt);

  std::cout << "xmin:" << minPt.x << "; xmax:" << maxPt.x
            << "\nymin:" << minPt.y << "; ymax:" << maxPt.y
            << "\nzmin:" << minPt.z << "; zmax:" << maxPt.z
            << std::endl; 

  // reconstruct a cube point cloud for the mesh:
      //1. compute how many points 
  float distance = 10./512.;
  float xpoints, ypoints, zpoints;
  xpoints = int ( (maxPt.x-minPt.x) / distance) + 1;
  ypoints = int ( (maxPt.y-minPt.y) / distance) + 1;
  zpoints = int ( (maxPt.z-minPt.z) / distance) + 1;

  int xy_side = xpoints * ypoints;
  int xz_side = xpoints * zpoints;
  int yz_side = ypoints * zpoints;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_side(new pcl::PointCloud<pcl::PointXYZ>(2*xz_side+2*xy_side+2*yz_side,1));
  std::vector<pcl::PointXYZ> side_point;
  pcl::PointXYZ tmp;
  
  // build the point cloud for xy side:
  for(int i=0; i<ypoints; i++){
    for(int j=0; j<xpoints; j++){
      // (xmin,ymin,zmin)
      tmp.x = minPt.x + j*distance; 
      tmp.y = minPt.y + i*distance;
      tmp.z = minPt.z;
      side_point.push_back(tmp);
      // (xmin,ymin,zax)
      tmp.z = maxPt.z;
      side_point.push_back(tmp);
    }
  }

  // build the point cloud for xz side:
  for(int i=0; i<zpoints; i++){
    for(int j=0; j<xpoints; j++){
      // (xmin,ymin,zmin)
      tmp.x = minPt.x + j*distance; 
      tmp.y = minPt.y;
      tmp.z = minPt.z + i*distance;
      side_point.push_back(tmp);
      // (xmin,ymin,zax)
      tmp.y = maxPt.y;
      side_point.push_back(tmp);
    }
  }

  // build the point cloud for yz side:
  for(int i=0; i<ypoints; i++){
    for(int j=0; j<zpoints; j++){
      // (xmin,ymin,zmin)
      tmp.x = minPt.x; 
      tmp.y = minPt.y + i*distance;
      tmp.z = minPt.z + j*distance;
      side_point.push_back(tmp);
      // (xmin,ymin,zax)
      tmp.x = maxPt.x;
      side_point.push_back(tmp);
    }
  }

  int pos = 0;

  for(auto&point: *cloud_side){
    tmp = side_point.at(pos);
    point.x = tmp.x;
    point.y = tmp.y;
    point.z = tmp.z;
    pos += 1;
  }

  // try addCube function

  pcl::visualization::PCLVisualizer viewer ("bounding box for super8 mesh");

  // Deftin the color:
  float bckgr_gray_level = 0.0;
  float txt_gray_lvl = 1 - bckgr_gray_level;
  // Add org cloud:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_map, 20,100,20);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h2 (cloud_org, 255,10,10);

  viewer.addPointCloud (cloud_map, cloud_in_color_h, "cloud_org_v1");
  viewer.addPointCloud (cloud_org,  cloud_in_color_h2, "cloud_org");
  viewer.addPointCloud (cloud_side, "test" );
  //viewer.addCube (minPt.x,maxPt.x,minPt.y,maxPt.y,minPt.z,maxPt.z);

  //
  while (!viewer.wasStopped ())
  {
   viewer.spinOnce ();
  }


  // TO DO: marching cube for the cube:
  
}


void show_info()
{
  std::cout << "Usage: [Option]" <<std::endl;
  std::cout << "Options:" <<std::endl;
  std::cout << "-diffg=1, show different mesh compared with orginal mesh" <<std::endl;
  std::cout << "-i1: file1 mesh" <<std::endl;
  std::cout << "-i2: file2 mesh" <<std::endl;
  std::cout << "-i3: file3 mesh" <<std::endl;
  std::cout << "-diffg=2, show apprximate box and ICP evluation" <<std::endl;
  std::cout << "-i1: file1 orginal mesh" <<std::endl;
  std::cout << "-i2: file2 reconstructed mesh" <<std::endl;
  std::cout << "-i3: file3 any message" <<std::endl;
}

void compare(std::string file1, std::string file2, std::string file3)
{
  // Info: teset the different duck size:
  PointCloudT::Ptr duck_big (new PointCloudT);  
  PointCloudT::Ptr duck_small (new PointCloudT); 
  pcl::io::loadPLYFile("/home/wei/桌面/model/duckbig/duckbig.ply",*duck_big);
  pcl::io::loadPLYFile("/home/wei/桌面/pcltest/duck.ply",*duck_small);

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*duck_big, minPt, maxPt);

  std::cout << "Big duck info:" << std::endl;
  std::cout << "xmin:" << minPt.x << "; xmax:" << maxPt.x
            << "\nymin:" << minPt.y << "; ymax:" << maxPt.y
            << "\nzmin:" << minPt.z << "; zmax:" << maxPt.z
            << std::endl; 

  pcl::getMinMax3D(*duck_small, minPt, maxPt);

  std::cout << "small duck info:" << std::endl;
  std::cout << "xmin:" << minPt.x << "; xmax:" << maxPt.x
            << "\nymin:" << minPt.y << "; ymax:" << maxPt.y
            << "\nzmin:" << minPt.z << "; zmax:" << maxPt.z
            << std::endl; 

  //Set input
  PointCloudT::Ptr duck_org (new PointCloudT);  
  PointCloudT::Ptr duck_gdiv1 (new PointCloudT);
  PointCloudT::Ptr duck_gdiv3 (new PointCloudT); 
  PointCloudT::Ptr duck_gdiv10 (new PointCloudT);   

  pcl::io::loadPLYFile("/home/wei/桌面/model/duckbig/duckbig.ply",*duck_org);
  pcl::io::loadPLYFile(file1,*duck_gdiv1);
  pcl::io::loadPLYFile(file2,*duck_gdiv3);
  pcl::io::loadPLYFile(file3,*duck_gdiv10);

  // Test Show some result;

  pcl::visualization::PCLVisualizer viewer ("compare different gravity result");
  int v1 (0);
  int v2 (1);
  int v3 (2);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 0.5, v2);
  viewer.createViewPort (0.5, 0.5, 1, 1,   v3);
  // Deftin the color:
  float bckgr_gray_level = 0.0;
  float txt_gray_lvl = 1 - bckgr_gray_level;
  // Add org cloud:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color1 (duck_org, 0,255,0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color2 (duck_gdiv1, 255,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color3 (duck_gdiv3, 0,0,255);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color4 (duck_gdiv10, 255,255,0);

  viewer.addPointCloud (duck_org,    color1, "org1",v1);
  viewer.addPointCloud (duck_org,    color1, "org2",v2);
  viewer.addPointCloud (duck_org,    color1, "org3",v3);
  viewer.addPointCloud (duck_gdiv1,  color2, "1g",v1);
  viewer.addPointCloud (duck_gdiv3,  color3, "0.3g",v2);
  viewer.addPointCloud (duck_gdiv10, color4, "0.1g",v3);

  viewer.addText ("Green: Original duck\n Red: result in g", 
    10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info1", v1);

  viewer.addText ("Green: Original duck\n Blue: result in 0.3g", 
    10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info2", v2);


  viewer.addText ("Green: Original duck\n Yellow: result in 0.1g", 
    10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info3", v3);



  while (!viewer.wasStopped ())
  {
   viewer.spinOnce ();
  }
}

void apprximate_box(std::string file1, std::string file2){
  //

  double approximate_x,real_x_big,real_x_small;
  // Info: teset the different duck size:
  PointCloudT::Ptr duck_big (new PointCloudT);  
  PointCloudT::Ptr duck_small (new PointCloudT); 
  pcl::io::loadPLYFile("/home/wei/桌面/model/duckbig/duckbig.ply",*duck_big);
  pcl::io::loadPLYFile("/home/wei/桌面/pcltest/duck.ply",*duck_small);

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*duck_big, minPt, maxPt);

  // std::cout << "Big duck info:" << std::endl;
  // std::cout << "xmin:" << minPt.x << "; xmax:" << maxPt.x
  //           << "\nymin:" << minPt.y << "; ymax:" << maxPt.y
  //           << "\nzmin:" << minPt.z << "; zmax:" << maxPt.z
  //           << std::endl; 
  real_x_big = maxPt.x - minPt.x;
  pcl::getMinMax3D(*duck_small, minPt, maxPt);
  real_x_small = maxPt.x - minPt.x;

  // std::cout << "small duck info:" << std::endl;
  // std::cout << "xmin:" << minPt.x << "; xmax:" << maxPt.x
  //           << "\nymin:" << minPt.y << "; ymax:" << maxPt.y
  //           << "\nzmin:" << minPt.z << "; zmax:" << maxPt.z
  //           << std::endl; 

  //Set input
  PointCloudT::Ptr duck_org (new PointCloudT);  
  PointCloudT::Ptr duck_reconstructed (new PointCloudT); 

  pcl::io::loadPLYFile(file1,*duck_org);
  pcl::io::loadPLYFile(file2,*duck_reconstructed);

  // Find the max edge:

  float v_org, v_reconstructed;
  pcl::getMinMax3D(*duck_org, minPt, maxPt);
  v_reconstructed = (maxPt.x-minPt.x) * (maxPt.y-minPt.z) * (maxPt.z-minPt.z);

  pcl::getMinMax3D(*duck_reconstructed, minPt, maxPt);
  double x_edge,y_edge,z_edge,max_edge;
  // draw the cube 
      //1. compute how many points 
  float distance = 10./512.;
  float xpoints, ypoints, zpoints;

  double dist;
    dist = (maxPt.x-minPt.x);
  maxPt.x = maxPt.x + 0.1*dist;
  minPt.x = minPt.x - 0.1*dist;
    dist = (maxPt.y-minPt.y);
  maxPt.y = maxPt.y + 0.1*dist;
  minPt.y = minPt.y - 0.1*dist;
    dist = (maxPt.z-minPt.z);
  maxPt.z = maxPt.z + 0.1*dist;
  minPt.z = minPt.z - 0.1*dist;


  approximate_x = maxPt.x - minPt.x;
  v_org = (maxPt.x-minPt.x) * (maxPt.y-minPt.z) * (maxPt.z-minPt.z);
  

  xpoints = int ( (maxPt.x-minPt.x) / distance) + 1;
  ypoints = int ( (maxPt.y-minPt.y) / distance) + 1;
  zpoints = int ( (maxPt.z-minPt.z) / distance) + 1;



  int xy_side = xpoints * ypoints;
  int xz_side = xpoints * zpoints;
  int yz_side = ypoints * zpoints;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_side(new pcl::PointCloud<pcl::PointXYZ>(2*xz_side+2*xy_side+2*yz_side,1));
  std::vector<pcl::PointXYZ> side_point;
  pcl::PointXYZ tmp;
  // build the point cloud for xy side:
  for(int i=0; i<ypoints; i++){
    for(int j=0; j<xpoints; j++){
      // (xmin,ymin,zmin)
      tmp.x = minPt.x + j*distance; 
      tmp.y = minPt.y + i*distance;
      tmp.z = minPt.z;
      side_point.push_back(tmp);
      // (xmin,ymin,zax)
      tmp.z = maxPt.z;
      side_point.push_back(tmp);
    }
  }

  // build the point cloud for xz side:
  for(int i=0; i<zpoints; i++){
    for(int j=0; j<xpoints; j++){
      // (xmin,ymin,zmin)
      tmp.x = minPt.x + j*distance; 
      tmp.y = minPt.y;
      tmp.z = minPt.z + i*distance;
      side_point.push_back(tmp);
      // (xmin,ymin,zax)
      tmp.y = maxPt.y;
      side_point.push_back(tmp);
    }
  }

  // build the point cloud for yz side:
  for(int i=0; i<ypoints; i++){
    for(int j=0; j<zpoints; j++){
      // (xmin,ymin,zmin)
      tmp.x = minPt.x; 
      tmp.y = minPt.y + i*distance;
      tmp.z = minPt.z + j*distance;
      side_point.push_back(tmp);
      // (xmin,ymin,zax)
      tmp.x = maxPt.x;
      side_point.push_back(tmp);
    }
  }

  int pos = 0;

  for(auto&point: *cloud_side){
    tmp = side_point.at(pos);
    point.x = tmp.x;
    point.y = tmp.y;
    point.z = tmp.z;
    pos += 1;
  }

  //ICP socre:
  double ICP_score;
  ICP_score = compute_ICP(file1,file2);
  std::cout<< "ICP score is" << ICP_score << std::endl;
  std::cout<< "approximate propgation is;" << approximate_x/real_x_small << std::endl;
  std::cout<< "approximate propgation is;" << approximate_x/real_x_big << std::endl;
  std::cout<< "approximate volumn is:" << v_reconstructed / v_org << std::endl;

  // show the result.
  pcl::visualization::PCLVisualizer viewer ("apprximate box for reconstructed mesh");
  // Deftin the color:
  float bckgr_gray_level = 0.0;
  float txt_gray_lvl = 1 - bckgr_gray_level;
  // Add org cloud:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (duck_org, 20,100,20);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h2 (duck_reconstructed, 255,10,10);

  viewer.addPointCloud (duck_org, cloud_in_color_h, "org");
  viewer.addPointCloud (duck_reconstructed,  cloud_in_color_h2, "reconstructed");
  viewer.addPointCloud (cloud_side, "apprximate_shape" );

  viewer.addText ("Green: Original duck\n Red: reconstructed mesh\n apprximate cube: While\n", 
    10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info1");

  //
  while (!viewer.wasStopped ())
  {
   viewer.spinOnce ();
  }

}


void check_size(){

  // cube = "/home/wei/桌面/rock_model_used/cube.ply";
  // rock1 = "/home/wei/桌面/rock_model_used/rock1.ply";
  // rock2 = "/home/wei/桌面/rock_model_used/rock2.ply";

  PointCloudT::Ptr cube (new PointCloudT);  
  PointCloudT::Ptr rock1 (new PointCloudT); 
  PointCloudT::Ptr rock2 (new PointCloudT); 

  pcl::io::loadPLYFile("/home/wei/桌面/rock_model_used/cube.ply",*cube);
  pcl::io::loadPLYFile("/home/wei/桌面/rock_model_used/rock1.ply",*rock1);
  pcl::io::loadPLYFile("/home/wei/桌面/rock_model_used/rock2.ply",*rock2);

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cube, minPt, maxPt);

  std::cout << "cube model info:" << std::endl;
  std::cout << "xmin:" << minPt.x << "; xmax:" << maxPt.x
            << "\nymin:" << minPt.y << "; ymax:" << maxPt.y
            << "\nzmin:" << minPt.z << "; zmax:" << maxPt.z
            << std::endl; 

  pcl::getMinMax3D(*rock1, minPt, maxPt);

  std::cout << "rock1 model info:" << std::endl;
  std::cout << "xmin:" << minPt.x << "; xmax:" << maxPt.x
            << "\nymin:" << minPt.y << "; ymax:" << maxPt.y
            << "\nzmin:" << minPt.z << "; zmax:" << maxPt.z
            << std::endl; 

  pcl::getMinMax3D(*rock2, minPt, maxPt);

  std::cout << "rock2 model info:" << std::endl;
  std::cout << "xmin:" << minPt.x << "; xmax:" << maxPt.x
            << "\nymin:" << minPt.y << "; ymax:" << maxPt.y
            << "\nzmin:" << minPt.z << "; zmax:" << maxPt.z
            << std::endl; 


}

double compute_ICP(std::string file1, std::string file2){
  // input:
  PointCloudT::Ptr duck_org (new PointCloudT);  
  PointCloudT::Ptr duck_reconstructed (new PointCloudT); 

  pcl::io::loadPLYFile(file1,*duck_org);
  pcl::io::loadPLYFile(file2,*duck_reconstructed);

  //icp:
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  int iterations = 1;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (duck_reconstructed);
  icp.setInputTarget (duck_org);
  icp.setMaximumIterations (1);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  return icp.getFitnessScore();
}

int main (int argc, char** argv)
{ 
  
  if( argc < 2)
  {
    std::cout << "Plase Enter the arguments" <<std::endl;
    show_info();
  }

  int nOptionIndex = 1;
  int task=0;
  std::string input1,input2,input3;

  while ( nOptionIndex<argc )
  {
    if(std::strncmp(argv[nOptionIndex],"-diffg=1",8)==0)
    {
      task = 1;
    }
    else if(std::strncmp(argv[nOptionIndex],"-diffg=2",8)==0)
    {
      task = 2;
    }
    else if(std::strncmp(argv[nOptionIndex],"-diffg=3",8)==0)
    {
      task = 3;
    }
    else if(std::strncmp(argv[nOptionIndex],"-i1=",4)==0)
    {
      input1 = &argv[nOptionIndex][4];
    }

    else if(std::strncmp(argv[nOptionIndex],"-i2=",4)==0)
    {
      input2 = &argv[nOptionIndex][4];
    }

    else if(std::strncmp(argv[nOptionIndex],"-i3=",4)==0)
    {
      input3 = &argv[nOptionIndex][4];
    }
    else{
      std::cout << "Usage wrong" <<std::endl;
      show_info();
    }
    nOptionIndex ++ ;
  }

  std::cout<< "task:" << task
           << "\ninput1:" << input1
           << "\ninput2:" << input2
           << "\ninput3:" << input3
  <<std::endl;


  if(task==1){
    compare(input1, input2, input3);
  }
  else if(task==2){
    apprximate_box(input1,input2);
  }
  else if (task==3){
    check_size();
  }
  // from_ply();
  // show_result();
  // show_octomap();
  // bounding_box();
  
  return 0;
}



//./icp -diffg=1 -i1=/home/wei/桌面/mesh/falling_one_axis/512_div1.ply -i2=/home/wei/桌面/mesh/falling_one_axis/512_div3.ply -i3=/home/wei/桌面/mesh/falling_one_axis/512_div10.ply

// ./icp -diffg=1 -i1=/home/wei/桌面/mesh/big_one_axis/div1.ply -i2=/home/wei/桌面/mesh/big_one_axis/div3.ply -i3=/home/wei/桌面/mesh/big_one_axis/div10.ply

//./icp -diffg=2 -i1=/home/wei/桌面/model/duckbig/duckbig.ply -i2=/home/wei/桌面/project_bag/result/big/tsdf/e3.ply -i3=0

// ./icp -diffg=2 -i1=/home/wei/桌面/pcltest/duck.ply -i2=/home/wei/桌面/project_bag/result/small/ofusion/e1.ply -i3=0









  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  // std::cout<< "HELLO PCL AND TEST THE ICP:" <<std::endl;


  // for(auto& point:*cloud_in)
  // {
  //  point.x = 1024 * rand() / (RAND_MAX + 1.0f);
  //   point.y = 1024 * rand() / (RAND_MAX + 1.0f);
  //   point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  // }

  // std::cout << "Saved " << cloud_in->points.size () << " data points to input:" << std::endl;

  // for(auto& point:*cloud_in)
  //  std::cout<<point<<std::endl;

  // *cloud_out = *cloud_in;
  // std::cout << "size:" << cloud_out->points.size() << std::endl;

  // for(auto& point:*cloud_out)
  //  std::cout<<point<<std::endl;

  // //ICP procedu:
  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // icp.setInputSource(cloud_in);
  // icp.setInputTarget(cloud_out);

  // pcl::PointCloud<pcl::PointXYZ> Final;
  // icp.align(Final);


  // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  // icp.getFitnessScore() << std::endl;
  // std::cout << icp.getFinalTransformation() << std::endl;