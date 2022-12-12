#include "disp_to_pcl.h"

#include "Colormap.h"
#include <omega_msgs/OmegaMetadata.h>

#include <getopt.h>

SubscribeAndPublish::SubscribeAndPublish(bool _useColorMap, int _filterLevel)
{
  ROS_INFO("Initializing metadata...");
  // read the sensor metadata from the topic 'omega_metadata
  boost::shared_ptr<omega_msgs::OmegaMetadata const> metadata;
  metadata = ros::topic::waitForMessage<omega_msgs::OmegaMetadata>("/omega_metadata",ros::Duration(0.5));
  if(metadata == NULL)
  {
    ROS_ERROR("No metadata received. Shutting down... ");
    initSuccessfully_ = false ;
  }
  else
  {
    useColorMap_ = _useColorMap;
    filterLevel_ = _filterLevel;
    skipRectified_ = _useColorMap && (filterLevel_==255);
    focallength_= metadata->focallength;
    baseline_ = metadata->baseline*0.001; // baseline in mm.
    rightopticalcenterx_ = metadata->rightopticalcenterx;
    rightopticalcentery_ = metadata->rightopticalcentery;

    ROS_INFO("Advertising sensor_msgs/PointCloud2 on /omega_pcl.");
    //Topic you want to publish
    omg_pcl_ = n_.advertise<sensor_msgs::PointCloud2>("/omega_pcl",10);

    //Topic you want to subscribe
    ROS_INFO("Subscribing to /omega_disp_image.");
    subDisp_ = n_.subscribe("/omega_disp_image", 100, &SubscribeAndPublish::dispCallback, this);
    if(!skipRectified_)
    {
      ROS_INFO("Subscribing to /omega_rec_image.");
      subRec_ = n_.subscribe("/omega_rec_image", 100, &SubscribeAndPublish::recCallback, this);
    }

    ROS_INFO("Initializing point cloud computation timer.");
    computationTimer_ = n_.createTimer(ros::Duration(0.1), &SubscribeAndPublish::computationCallback, this);

    initSuccessfully_ = true ;
  }
}


void SubscribeAndPublish::dispCallback(const sensor_msgs::Image& input)
{
  dispBuffer_.push(input);
}

void SubscribeAndPublish::recCallback(const sensor_msgs::Image& input)
{
  recBuffer_.push(input);
}


void SubscribeAndPublish::computationCallback(const ros::TimerEvent& event)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr omg_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  sensor_msgs::PointCloud2 ros_omg_pcl;

  // We wait until there is a disparity image in the buffer.
  if(dispBuffer_.empty())
  {
    return;
  }
  sensor_msgs::Image disp = dispBuffer_.front();

  // If we need it, wait until there is a rectified image in the buffer.
  if(!skipRectified_ && recBuffer_.empty())
  {
    return;
  }
  sensor_msgs::Image rec ;
  if(!skipRectified_ && !recBuffer_.empty())
  {
    rec = recBuffer_.front();
  }

  // Assigning all points to single row to keep PointCloud unordered
  omg_pcl->width = disp.width * disp.height;
  omg_pcl->height = 1;

  omg_pcl->points.resize (omg_pcl->width * omg_pcl->height);

  int disp_buffer_size_coef = 1;
  int disp_subpixel_coef = 4;
  if (disp.encoding.find("mono16") != std::string::npos) {
      disp_buffer_size_coef = 2;
      disp_subpixel_coef = 16;
  }

  float Z(0);
  const uint16_t *data_16_bits = (const uint16_t *)disp.data.data();

  for (size_t i = 0; i < omg_pcl->points.size (); ++i)
  {
    int column = (int) fmod(i,disp.width) + 1;
    int row = floor(i/disp.width) + 1;        

    omg_pcl->points[i].x = 0;
    omg_pcl->points[i].y = 0;
    omg_pcl->points[i].z = 0;
    float disp_value = (disp.encoding == "mono16") ? (float) data_16_bits[i] : (float) disp.data[i];
    if (disp_value > 0)
    {
        Z = (baseline_ * focallength_) / (disp_value / disp_subpixel_coef);
        omg_pcl->points[i].x = (Z*(column - rightopticalcenterx_))/focallength_;
        omg_pcl->points[i].y = (Z*(row - rightopticalcentery_))/focallength_;
        omg_pcl->points[i].z = Z;
    }
    
    // We colorize each voxel in function of the mode selected.
    uint8_t r, g, b;
    bool filteringFlag = false;
    if(!skipRectified_)
    {
      r = rec.data[((i*3))];
      g = rec.data[((i*3)+1)];
      b = rec.data[((i*3)+2)];
      
      if(r > filterLevel_ && g > filterLevel_ && b > filterLevel_)
      {
        filteringFlag = true;
        r = 0; 
        g = 0;
        b = 0;
        omg_pcl->points[i].x = 0;
        omg_pcl->points[i].y = 0;
        omg_pcl->points[i].z = 0;
      }
    }
    if(useColorMap_ && !filteringFlag)
    {
      int disp_level = (disp.encoding == "mono16") ? (int) data_16_bits[i]/4 : (int) disp.data[i];
      r = _colorMapDisp[disp_level][0];
      g = _colorMapDisp[disp_level][1];
      b = _colorMapDisp[disp_level][2];
    }

    // pack r/g/b into rgb
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    omg_pcl->points[i].rgb = *reinterpret_cast<float*>(&rgb);

  }

  pcl::toROSMsg(*omg_pcl,ros_omg_pcl);
  ros_omg_pcl.header = disp.header;
  ros_omg_pcl.header.stamp = ros::Time::now();
  ros_omg_pcl.header.frame_id = "omega_cam";
  
  omg_pcl_.publish(ros_omg_pcl);

  dispBuffer_.pop();
  if(!skipRectified_)
  {
    recBuffer_.pop();
  }

}


bool SubscribeAndPublish::hasInitSuccessfully() const 
{
  return initSuccessfully_ ;
}


static void print_help()
{
  std::cout << "Arguments : [-c] [-f FILTER_LEVEL] [-h]" << std::endl << std::endl;
  std::cout << "\tThe node disp_to_pcl allows three optionnal arguments." << std::endl << std::endl;
  std::cout << "\t-c --color_map\tUse the color map instend of the rectified image's color." << std::endl << std::endl;
  std::cout << "\t-f FILTER_LEVEL --filter_level=FILTER_LEVEL\tSpecify the white filter level." << std::endl << std::endl;
  std::cout << "\tFILTER_LEVEL should be between 0 and 256. The default value is 256. It corresponds not to filter." << std::endl << std::endl;
  std::cout << "\t-h --help\tDisplay this help" << std::endl << std::endl;
  exit(1);
}


void parse_cmdline(int argc, char** argv, bool &useColorMap, int &filterLevel)
{
  struct option long_options[] = {
    {"color_map", no_argument, 0, 'c'},
    {"filter_level", required_argument, 0, 'f'},
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
  };

  while (1) {
    /* Use getopt to parse commandline */
    int option_index = 0, c;
    c = getopt_long(argc, argv, "chf:",long_options, &option_index);
    if (c == -1) {
      break;
    }

    switch (c)
    {
    case 'c':
      useColorMap = true;
      break;
    case 'f':
      filterLevel = atoi(optarg);
      if (filterLevel < 0 || filterLevel > 255)
      {
        print_help();
      }
      break;
    case 'h':
    case '?':
      print_help();
      break;
    default:
      break;
    }
  }
}


int main(int argc, char** argv)
{
  ROS_INFO("Starting node disp_to_pcl...");
  //Initiate ROS
  ros::init(argc, argv, "disp_to_pcl");

  bool _useColorMap=false;
  int _filterLevel=255;

  parse_cmdline(argc, argv, _useColorMap, _filterLevel);

  SubscribeAndPublish SAPObject3(_useColorMap, _filterLevel);
  if(!SAPObject3.hasInitSuccessfully())
  {
    return -1;
  }

  ros::spin();
}
