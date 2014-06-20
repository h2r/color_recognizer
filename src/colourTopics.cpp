#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>

// specific message types of this package
#include <colour_recognizer/ObjectProperty.h>
#include <colour_recognizer/ObjectArray.h>

//other basic message types
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <typeinfo>


#include <math.h>
#include <string>

ros::Publisher pub;
ros::Publisher pubCloud;
ros::Publisher pubObjectLocation;

typedef pcl::PointCloud<pcl::PointXYZRGB> myPointCloud;
typedef boost::shared_ptr <std::vector<int> > IndicesPtr;

// pcl::PCLPointCloud2 globalCloud;
myPointCloud::Ptr globalCloud (new myPointCloud);

const int colorNumbersUsed=52;
const std::string colourNameArray[colorNumbersUsed] = {"purple","green","blue","pink","brown","red","light blue","teal","orange","light green","magenta","yellow","sky blue","grey","lime green","light purple","violet","dark green","turquoise","lavender","dark blue","tan","cyan","aqua","forest green","mauve","dark purple","bright green","maroon","olive","salmon","beige","royal blue","navy blue","lilac","black","hot pink","light brown","pale green","peach","olive green","dark pink","periwinkle","sea green","lime","indigo","mustard","light pink","rose","bright blue","neon green","white"};
const int redArray[colorNumbersUsed] = {126, 21, 3, 255, 101, 229, 149, 2, 249, 150, 194, 255, 117, 146, 137, 191, 154, 3, 6, 199, 0, 209, 0, 19, 6, 174, 53, 1, 101, 110, 255, 230, 5, 0, 206, 0, 255, 173, 199, 255, 103, 203, 142, 83, 170, 56, 206, 255, 207, 1, 12, 255};
const int greenArray[colorNumbersUsed] = {30, 176, 67, 129, 55, 0, 208, 147, 115, 249, 0, 255, 187, 149, 254, 119, 14, 53, 194, 159, 3, 178, 255, 234, 71, 113, 6, 255, 0, 117, 121, 218, 4, 17, 162, 0, 2, 129, 253, 176, 122, 65, 130, 252, 255, 2, 179, 209, 98, 101, 255, 255};
const int blueArray[colorNumbersUsed] = {156, 26, 223, 192, 0, 0, 252, 134, 6, 123, 120, 20, 253, 145, 5, 246, 234, 0, 172, 239, 91, 111, 255, 201, 12, 129, 62, 7, 33, 14, 108, 166, 170, 70, 253, 0, 141, 80, 181, 124, 4, 107, 254, 161, 50, 130, 1, 223, 117, 252, 12, 255};




// ros::NodeHandle nh;

void 
marker_cb (const visualization_msgs::MarkerArray::ConstPtr& inputMarkerArray)
{
  colour_recognizer::ObjectArray objectArrayTemp;
  visualization_msgs::MarkerArray tempMarkerArray;

  myPointCloud::Ptr cloud_filtered_sum (new myPointCloud);
  // myPointCloud::Ptr cloud_filtered_sum ;
  int flag=0;


  // find out mean of all points within a marker array
  for(int i=0;i<inputMarkerArray->markers.size();i++){
    colour_recognizer::ObjectProperty objectTemp;
    float xSum=0.0;
    float ySum=0.0;
    float zSum=0.0;
    for(int j=0;j<inputMarkerArray->markers[i].points.size();j++){
      xSum+=inputMarkerArray->markers[i].points[j].x;
      ySum+=inputMarkerArray->markers[i].points[j].y;
      zSum+=inputMarkerArray->markers[i].points[j].z;
    }
  
  float xPos=xSum/inputMarkerArray->markers[i].points.size();
  float yPos=ySum/inputMarkerArray->markers[i].points.size();
  float zPos=zSum/inputMarkerArray->markers[i].points.size();
  // standard deviation of all points wihtin a marker array
  float xVarSum=0.0;
  float yVarSum=0.0;
  float zVarSum=0.0;
  for(int j=0;j<inputMarkerArray->markers[i].points.size();j++){
      xVarSum+=pow(inputMarkerArray->markers[i].points[j].x - xPos,2);
      yVarSum+=pow(inputMarkerArray->markers[i].points[j].y - yPos,2);
      zVarSum+=pow(inputMarkerArray->markers[i].points[j].z - zPos,2);
    }
  
  float xStd=sqrt(xVarSum/inputMarkerArray->markers[i].points.size());
  float yStd=sqrt(yVarSum/inputMarkerArray->markers[i].points.size());
  float zStd=sqrt(zVarSum/inputMarkerArray->markers[i].points.size());
  // making a marker cube with dimensions of 2 SD along the sides, 4 SD would ensure 98% of the points, but 
  // the cloud does not align very well with a large cube. 2 SD seems satisfactory
  
  visualization_msgs::Marker tempMarker;
  tempMarker.header.frame_id = inputMarkerArray->markers[i].header.frame_id;
  tempMarker.header.stamp = ros::Time();
  tempMarker.ns = "my_namespace";
  tempMarker.id = inputMarkerArray->markers[i].id;
  tempMarker.type = visualization_msgs::Marker::CUBE;
  tempMarker.action = visualization_msgs::Marker::ADD;
  tempMarker.pose.position.x = xPos;
  tempMarker.pose.position.y = yPos;
  tempMarker.pose.position.z = zPos;
  tempMarker.pose.orientation.x = 0.0;
  tempMarker.pose.orientation.y = 0.0;
  tempMarker.pose.orientation.z = 0.0;
  tempMarker.pose.orientation.w = 1.0;
  tempMarker.scale.x = 2*xStd;
  tempMarker.scale.y = 2*yStd;
  tempMarker.scale.z = 2*zStd;
  tempMarker.color.a = 1.0;
  tempMarker.color.r = 0.0;
  tempMarker.color.g = 1.0;
  tempMarker.color.b = 0.0;
  tempMarkerArray.markers.push_back(tempMarker);

  myPointCloud::Ptr cloud_filtered_z (new myPointCloud);
  myPointCloud::Ptr cloud_filtered_y (new myPointCloud);
  myPointCloud::Ptr cloud_filtered_x (new myPointCloud);
  // myPointCloud::Ptr cloud_filtered_z;
  // myPointCloud::Ptr cloud_filtered_y;
  // myPointCloud::Ptr cloud_filtered_x;

  pcl::PassThrough<pcl::PointXYZRGB> pass_z(true);
  pcl::PassThrough<pcl::PointXYZRGB> pass_y;
  pcl::PassThrough<pcl::PointXYZRGB> pass_x;


  IndicesPtr zPointer (new std::vector<int>);
  IndicesPtr yPointer (new std::vector<int>);
  IndicesPtr xPointer (new std::vector<int>);


  pass_z.setInputCloud (globalCloud);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (zPos-zStd, zPos+zStd);
  pass_z.filter (*zPointer);


  // pass_z.setFilterFieldName ("y");
  // pass_z.setFilterLimits (yPos-yStd, yPos+yStd);

  pass_z.setIndices (zPointer);
  pass_z.setFilterFieldName ("x");
  pass_z.setFilterLimits (xPos-xStd, xPos+xStd);
  pass_z.filter (*xPointer);

  pass_z.setIndices(xPointer);
  pass_z.setFilterFieldName ("y");
  pass_z.setFilterLimits (yPos-yStd, yPos+yStd);
  // pass_z.filter (*zPointer);

  //pass.setFilterLimitsNegative (true);
  pass_z.filter (*cloud_filtered_y);

  // pass_y.setInputCloud (cloud_filtered_z);
  // pass_y.setFilterFieldName ("y");
  // pass_y.setFilterLimits (yPos-yStd, yPos+yStd);
  // //pass.setFilterLimitsNegative (true);
  // pass_z.filter (*cloud_filtered_y);

  // pass_x.setInputCloud (cloud_filtered_y);
  // pass_x.setFilterFieldName ("x");
  // pass_x.setFilterLimits (xPos-xStd, xPos+xStd);
  // //pass.setFilterLimitsNegative (true);
  // pass_x.filter (*cloud_filtered_x);
  int sizeCloud = cloud_filtered_y->size();
  if(sizeCloud==0){
    sizeCloud+=1;
  }
  if(flag==0){
    flag=1;
    *cloud_filtered_sum=*cloud_filtered_y;
    // ROS_INFO("size: %zu",(*cloud_filtered_y).size());
    int redColor=0;
    int blueColor=0;
    int greenColor=0;
    // float xLoc=0.0;
    // float yLoc=0.0;
    // float zLoc=0.0;
    for(int pointCount=0;pointCount<cloud_filtered_y->size();pointCount++){
      redColor += (*cloud_filtered_y)[pointCount].r;
      greenColor += (*cloud_filtered_y)[pointCount].g;
      blueColor += (*cloud_filtered_y)[pointCount].b;
      // xLoc+= (*cloud_filtered_y)[pointCount].x;
      // yLoc+= (*cloud_filtered_y)[pointCount].y;
      // zLoc+= (*cloud_filtered_y)[pointCount].z;
      // ROS_INFO("G value: %d",(*cloud_filtered_y)[pointCount].g);
      // ROS_INFO("R value: %d",(*cloud_filtered_y)[pointCount].r);
      // ROS_INFO("B value: %d",(*cloud_filtered_y)[pointCount].b);
      // ROS_INFO("type : %s", typeid((*cloud_filtered_y)[pointCount].g).name());
    }

    // ROS_INFO("end");

    int redColorAvg = ((int)(redColor/sizeCloud));
    int greenColorAvg = ((int)(greenColor/sizeCloud));
    int blueColorAvg = ((int)(blueColor/sizeCloud));

    int minDist = 999999;
    int minIndex = 0;


    for (int colourCount = 0;colourCount<colorNumbersUsed;colourCount++){
      int colourDist = (redColorAvg - redArray[colourCount])*(redColorAvg - redArray[colourCount]) + (greenColorAvg - greenArray[colourCount])*(greenColorAvg - greenArray[colourCount]) + (blueColorAvg - blueArray[colourCount])*(blueColorAvg - blueArray[colourCount]) ;
      if(colourDist<minDist){
        minDist = colourDist;
        minIndex = colourCount;
      }
          }


    colour_recognizer::ObjectProperty tempProperty;
    tempProperty.id=inputMarkerArray->markers[i].id;
    tempProperty.colour = colourNameArray[minIndex];
    tempProperty.locationX=xPos;
    tempProperty.locationY=yPos;
    tempProperty.locationZ=zPos;
    tempProperty.objectType = "dunno";
    
    tempProperty.colourHist[0]=redColorAvg;
    tempProperty.colourHist[1]=greenColorAvg;
    tempProperty.colourHist[2]=blueColorAvg;

    // arrayTemp[0]=int(redColor/cloud_filtered_y->size());
    // arrayTemp[1]=int(greenColor/cloud_filtered_y->size();
    // arrayTemp[2]=int(blueColor/cloud_filtered_y->size());
    // tempProperty.colourHist=arrayTemp;
    objectArrayTemp.objects.push_back(tempProperty);


  }
  else{
    *cloud_filtered_sum+=*cloud_filtered_y; 
    int redColor=0;
    int blueColor=0;
    int greenColor=0;
    // float xLoc=0.0;
    // float yLoc=0.0;
    // float zLoc=0.0;
    for(int pointCount=0;pointCount<cloud_filtered_y->size();pointCount++){
      redColor += (*cloud_filtered_y)[pointCount].r;
      greenColor += (*cloud_filtered_y)[pointCount].g;
      blueColor += (*cloud_filtered_y)[pointCount].b;
      // xLoc+= (*cloud_filtered_y)[pointCount].x;
      // yLoc+= (*cloud_filtered_y)[pointCount].y;
      // zLoc+= (*cloud_filtered_y)[pointCount].z;
      // ROS_INFO("G value: %d",(*cloud_filtered_y)[pointCount].g);
      // ROS_INFO("R value: %d",(*cloud_filtered_y)[pointCount].r);
      // ROS_INFO("B value: %d",(*cloud_filtered_y)[pointCount].b);
      // ROS_INFO("type : %s", typeid((*cloud_filtered_y)[pointCount].g).name());
    }

    // ROS_INFO("end");

    int redColorAvg = ((int)(redColor/sizeCloud));
    int greenColorAvg = ((int)(greenColor/sizeCloud));
    int blueColorAvg = ((int)(blueColor/sizeCloud));

    int minDist = 999999;
    int minIndex = 0;


    for (int colourCount = 0;colourCount<colorNumbersUsed;colourCount++){
      int colourDist = (redColorAvg - redArray[colourCount])*(redColorAvg - redArray[colourCount]) + (greenColorAvg - greenArray[colourCount])*(greenColorAvg - greenArray[colourCount]) + (blueColorAvg - blueArray[colourCount])*(blueColorAvg - blueArray[colourCount]) ;
      if(colourDist<minDist){
        minDist = colourDist;
        minIndex = colourCount;
      }
          }


    colour_recognizer::ObjectProperty tempProperty;
    tempProperty.id=inputMarkerArray->markers[i].id;
    tempProperty.colour = colourNameArray[minIndex];
    tempProperty.locationX=xPos;
    tempProperty.locationY=yPos;
    tempProperty.locationZ=zPos;
    tempProperty.objectType = "dunno";
    
    tempProperty.colourHist[0]=redColorAvg;
    tempProperty.colourHist[1]=greenColorAvg;
    tempProperty.colourHist[2]=blueColorAvg;

    // arrayTemp[0]=int(redColor/cloud_filtered_y->size());
    // arrayTemp[1]=int(greenColor/cloud_filtered_y->size();
    // arrayTemp[2]=int(blueColor/cloud_filtered_y->size());
    // tempProperty.colourHist=arrayTemp;
    objectArrayTemp.objects.push_back(tempProperty);
    
  }
  

  // cloud_filtered_x->~myPointCloud();
  // cloud_filtered_y->~myPointCloud();
  // cloud_filtered_z->~myPointCloud();







  //pubObjectLocation.publish()
  }
  //publishing a delayed point cloud and the marker array
  // pubObjectLocation.publish(tempMarkerArray);
  pubObjectLocation.publish(objectArrayTemp);
  pubCloud.publish(*cloud_filtered_sum);

  
}






void 
cloud_cb (const myPointCloud::ConstPtr& input)
{
  // pcl::PCLPointCloud2 cloud_filtered;
  // ros::NodeHandle nh1;
  // globalCloud->~myPointCloud();
  // globalCloud.reset();
  // myPointCloud::Ptr globalCloud (new myPointCloud);
  pcl::copyPointCloud(*input,*globalCloud);
  // ros::Subscriber subMarkerArray = nh1.subscribe ("/tabletop/clusters", 1, marker_cb);

  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//  const pcl::PCLPointCloud2::ConstPtr& "/camera/depth_registered/points"
//  sor.setInputCloud (input);

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber subMarkerArray = nh.subscribe ("/tabletop/clusters", 1, markers_cb);


  // sor.setInputCloud (input);
  // sor.setLeafSize (0.01, 0.01, 0.01);
  // sor.filter (cloud_filtered);    

  // Publish the dataSize 
  // pub.publish (input);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "colour_recognizer");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<myPointCloud>("/camera/depth_registered/points", 1, cloud_cb);
  ros::Subscriber subMarkerArray = nh.subscribe ("/tabletop/clusters", 1, marker_cb);

  // Create a ROS publisher for the output point cloud
  pubCloud = nh.advertise<myPointCloud> ("depth_registered_point_cloud_delayed", 1);
  // pubObjectLocation = nh.advertise<visualization_msgs::MarkerArray> ("object_location", 1);
  pubObjectLocation = nh.advertise<colour_recognizer::ObjectArray> ("object_location", 1);
  

  // Spin
  ros::spin ();
}
