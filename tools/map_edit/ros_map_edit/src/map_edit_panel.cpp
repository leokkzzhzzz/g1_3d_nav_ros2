#include "ros_map_edit/map_edit_panel.h"
#include "ros_map_edit/map_file_manager.h"
#include "ros_map_edit/tool_manager.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>
#include <visualization_msgs/MarkerArray.h>

namespace ros_map_edit
{

MapEditPanel::MapEditPanel(QWidget* parent)
  : rviz::Panel(parent)
  , main_layout_(nullptr)
  , file_manager_(nullptr)
{
  setupUI();
  file_manager_ = new MapFileManager();
}

MapEditPanel::~MapEditPanel()
{
  delete file_manager_;
}

void MapEditPanel::onInitialize()
{
  status_label_->setText("Ready — please open a map file");
}

void MapEditPanel::setupUI()
{
  main_layout_ = new QVBoxLayout;
  
  // 一键保存组
  save_group_ = new QGroupBox("File Management");
  QVBoxLayout* save_layout = new QVBoxLayout;
  
  // 当前地图显示
  current_map_label_ = new QLabel("Current map: not loaded");
  current_map_label_->setStyleSheet("QLabel { color: #333; font-weight: bold; padding: 5px; }");
  save_layout->addWidget(current_map_label_);
  
  // 一键保存按钮
  save_all_btn_ = new QPushButton("Save All Files");
  save_all_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-size: 14px; padding: 8px; }");
  save_layout->addWidget(save_all_btn_);
  
  // Open Map按钮
  open_map_btn_ = new QPushButton("Open Map");
  save_layout->addWidget(open_map_btn_);
  
  save_group_->setLayout(save_layout);
  
  // 状态显示
  status_label_ = new QLabel("Ready — please open a map file");
  status_label_->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 8px; border: 1px solid #ccc; border-radius: 4px; }");
  
  // 文件说明
  info_label_ = new QLabel(
    "保存文件说明:\n"
    "• map.yaml - 地图配置文件\n"
    "• map.pgm - 地图图像文件\n" 
    "• map.json - 虚拟墙数据\n"
    "• map_region.json - 区域数据\n\n"
    "提示: 文件将保存到当前地图的同一目录");
  info_label_->setStyleSheet("QLabel { color: #666; font-size: 11px; padding: 8px; }");
  
  // 组装主布局
  main_layout_->addWidget(save_group_);
  main_layout_->addWidget(new QLabel("Status:"));
  main_layout_->addWidget(status_label_);
  main_layout_->addWidget(info_label_);
  main_layout_->addStretch();
  
  setLayout(main_layout_);
  
  // 连接信号
  connect(save_all_btn_, SIGNAL(clicked()), this, SLOT(saveAllFiles()));
  connect(open_map_btn_, SIGNAL(clicked()), this, SLOT(openMap()));
}

void MapEditPanel::saveAllFiles()
{
  // 获取当前地图文件路径
  std::string current_map_file = getCurrentMapFile();
  if (current_map_file.empty())
  {
    QMessageBox::warning(this, "Warning", "Please load a map file first");
    return;
  }
  
  // 从当前地图文件路径提取目录和基础名称
  QFileInfo map_info(QString::fromStdString(current_map_file));
  QString base_dir = map_info.absolutePath();
  QString base_name = map_info.baseName(); // 不包含扩展名
  
  // 构建保存文件路径
  QString yaml_file = base_dir + "/" + base_name + ".yaml";
  QString pgm_file = base_dir + "/" + base_name + ".pgm";
  QString vw_file = base_dir + "/" + base_name + ".json";
  QString region_file = base_dir + "/" + base_name + "_region.json";
  
  bool success = true;
  QStringList saved_files;
  QStringList failed_files;
  
  try
  {
    // 获取工具管理器实例
    ToolManager& toolManager = ToolManager::getInstance();
    
    // 1. 保存地图 (yaml + pgm)
    MapEraserTool* eraserTool = toolManager.getMapEraserTool();
    if (eraserTool && eraserTool->getCurrentMap().data.size() > 0)
    {
      // 使用橡皮擦工具的当前地图数据
      if (file_manager_->saveMapFiles(yaml_file.toStdString(), eraserTool->getCurrentMap()))
      {
        saved_files << "Map files (yaml+pgm)";
      }
      else
      {
        failed_files << "Map: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    else
    {
      // 使用默认方法（从话题获取）
      if (file_manager_->saveMapFiles(yaml_file.toStdString()))
      {
        saved_files << "Map files (yaml+pgm)";
      }
      else
      {
        failed_files << "Map: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    
    // 2. 保存虚拟墙
    VirtualWallTool* wallTool = toolManager.getVirtualWallTool();
    if (wallTool)
    {
      std::vector<VirtualWall> walls = wallTool->getVirtualWalls();
      if (file_manager_->saveVirtualWallsFile(vw_file.toStdString(), walls))
      {
        saved_files << "Virtual walls (" + QString::number(walls.size()) + "  walls)";
      }
      else
      {
        failed_files << "Virtual walls: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    else
    {
      // 使用默认方法
      if (file_manager_->saveVirtualWallsFile(vw_file.toStdString()))
      {
        saved_files << "Virtual walls";
      }
      else
      {
        failed_files << "Virtual walls: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    
    // 3. 保存区域
    RegionTool* regionTool = toolManager.getRegionTool();
    if (regionTool)
    {
      std::vector<Region> regions = regionTool->getRegions();
      if (file_manager_->saveRegionsFile(region_file.toStdString(), regions))
      {
        saved_files << "Regions (" + QString::number(regions.size()) + "  regions)";
      }
      else
      {
        failed_files << "Regions: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    else
    {
      // 使用默认方法
      if (file_manager_->saveRegionsFile(region_file.toStdString()))
      {
        saved_files << "Regions";
      }
      else
      {
        failed_files << "Regions: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    
    // 显示结果
    QString message = "Saved to: " + base_dir + "\n\n";
    if (!saved_files.isEmpty())
    {
      message += "Saved: " + saved_files.join(", ") + "\n";
    }
    if (!failed_files.isEmpty())
    {
      message += "Failed: " + failed_files.join(", ") + "\n";
    }
    
    if (success)
    {
      status_label_->setText("All files saved: " + base_name);
      QMessageBox::information(this, "Save Successful", message);
    }
    else
    {
      status_label_->setText("Some files failed to save");
      QMessageBox::warning(this, "Partial Save Failure", message);
    }
  }
  catch (const std::exception& e)
  {
    QString error_msg = "Error during save: " + QString::fromStdString(e.what());
    status_label_->setText("Save Failed");
    QMessageBox::critical(this, "Save Error", error_msg);
  }
}

std::string MapEditPanel::getCurrentMapFile()
{
  // 首先检查用户是否手动加载了地图文件
  if (!current_map_file_.isEmpty())
  {
    return current_map_file_.toStdString();
  }
  
  // 从参数服务器获取当前地图文件路径
  ros::NodeHandle nh;
  std::string map_file;
  if (nh.getParam("/map_server/map_file", map_file) || nh.getParam("/rviz/map_file", map_file))
  {
    return map_file;
  }
  
  return "";
}

void MapEditPanel::openMap()
{
  // 设置默认路径为ros_map_edit/maps目录
  QString default_path = "src/ros_map_edit/maps";
  
  // 如果目录不存在，尝试其他可能的路径
  QDir maps_dir(default_path);
  if (!maps_dir.exists()) {
    // 尝试相对于工作目录的路径
    QStringList possible_paths = {
      "ros_map_edit/maps",
      "../src/ros_map_edit/maps", 
      "../../src/ros_map_edit/maps",
      QDir::homePath() + "/ros_ws/cursor_ws/src/ros_map_edit/maps"
    };
    
    for (const QString& path : possible_paths) {
      if (QDir(path).exists()) {
        default_path = path;
        break;
      }
    }
  }
  
  QString filename = QFileDialog::getOpenFileName(this,
                                                  "Open Map File",
                                                  default_path,
                                                  "YAML files (*.yaml);;PGM files (*.pgm);;All files (*.*)");
  
  if (!filename.isEmpty())
  {
    current_map_file_ = filename;
    
    // 先清空所有消息，再加载并发布新地图
    clearAllMessages();
    loadAndPublishMap(filename.toStdString());
    
    // 更新当前地图显示
    QString display_name = QFileInfo(filename).fileName();
    current_map_label_->setText("Current map: " + display_name);
    current_map_label_->setStyleSheet("QLabel { color: #007700; font-weight: bold; padding: 5px; }");
  }
}

void MapEditPanel::loadAndPublishMap(const std::string& filename)
{
  try
  {
    nav_msgs::OccupancyGrid map;
    
    ROS_INFO("Loading map: %s", filename.c_str());
    status_label_->setText("Loading map: " + QString::fromStdString(filename));
    
    // 根据文件扩展名决定加载方式
    std::string ext = filename.substr(filename.find_last_of(".") + 1);
    ROS_INFO("File extension: %s", ext.c_str());
    
    if (ext == "yaml" || ext == "yml")
    {
      // 加载YAML配置的地图
      ROS_INFO("Trying YAML map...");
      if (file_manager_->loadMap(filename, map))
      {
        ROS_INFO("Map loaded: %dx%d px, resolution %.3f m/px", 
                 map.info.width, map.info.height, map.info.resolution);
        publishMap(map);
        
        // 地图加载成功后，清空并重新加载对应的虚拟墙和区域
        loadCorrespondingFiles(filename);
        
        status_label_->setText("Map loaded: " + QFileInfo(QString::fromStdString(filename)).fileName());
      }
      else
      {
        QString error = QString::fromStdString(file_manager_->getLastError());
        ROS_ERROR("Map load failed: %s", error.toStdString().c_str());
        status_label_->setText("Load failed: " + error);
      }
    }
    else if (ext == "pgm")
    {
      // 直接加载PGM文件
      ROS_INFO("Trying PGM map...");
      if (file_manager_->loadMap(filename, map))
      {
        ROS_INFO("Map loaded: %dx%d px, resolution %.3f m/px", 
                 map.info.width, map.info.height, map.info.resolution);
        publishMap(map);
        
        // 地图加载成功后，清空并重新加载对应的虚拟墙和区域
        loadCorrespondingFiles(filename);
        
        status_label_->setText("Map loaded: " + QFileInfo(QString::fromStdString(filename)).fileName());
      }
      else
      {
        QString error = QString::fromStdString(file_manager_->getLastError());
        ROS_ERROR("Map load failed: %s", error.toStdString().c_str());
        status_label_->setText("Load failed: " + error);
      }
    }
    else
    {
      QString error = "Unsupported file format: " + QString::fromStdString(ext);
      ROS_ERROR("%s", error.toStdString().c_str());
      status_label_->setText(error);
    }
  }
  catch (const std::exception& e)
  {
    QString error = "Load error: " + QString::fromStdString(e.what());
    ROS_ERROR("%s", error.toStdString().c_str());
    status_label_->setText(error);
  }
}

void MapEditPanel::publishMap(const nav_msgs::OccupancyGrid& map)
{
  // 创建地图发布器 - 使用静态发布器确保持续发布
  static ros::Publisher map_pub;
  static bool initialized = false;
  
  if (!initialized)
  {
    ros::NodeHandle nh;
    // 使用latched=true确保新订阅者能立即收到地图数据
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    initialized = true;
    
    // 等待发布器准备就绪
    ros::Duration(0.5).sleep();
  }
  
  // 设置地图头信息
  nav_msgs::OccupancyGrid map_msg = map;
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = "map";
  
  // 发布地图多次以确保被接收
  for (int i = 0; i < 3; ++i)
  {
    map_pub.publish(map_msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  
  // 输出调试信息
  QString debug_info = QString("Map published: %1x%2 px, resolution %3 m/px")
                      .arg(map_msg.info.width)
                      .arg(map_msg.info.height)
                      .arg(map_msg.info.resolution);
  
  status_label_->setText(debug_info);
  
  // 同时发布到map_metadata话题
  static ros::Publisher metadata_pub;
  static bool metadata_initialized = false;
  
  if (!metadata_initialized)
  {
    ros::NodeHandle nh;
    metadata_pub = nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    metadata_initialized = true;
  }
  
  metadata_pub.publish(map_msg.info);
  
  ROS_INFO("Map published: %dx%d, resolution %.3f", 
           map_msg.info.width, map_msg.info.height, map_msg.info.resolution);
}

void MapEditPanel::loadCorrespondingFiles(const std::string& map_file_path)
{
  // 清空并重新加载对应的虚拟墙和Regions
  try
  {
    // 获取工具管理器实例
    ToolManager& toolManager = ToolManager::getInstance();
    
    // 清空并重新加载虚拟墙
    VirtualWallTool* wallTool = toolManager.getVirtualWallTool();
    if (wallTool)
    {
      ROS_INFO("Clearing and reloading virtual walls...");
      wallTool->loadVirtualWallsForMap(map_file_path);
    }
    
    // 清空并重新加载区域
    RegionTool* regionTool = toolManager.getRegionTool();
    if (regionTool)
    {
      ROS_INFO("Clearing and reloading regions...");
      regionTool->loadRegionsForMap(map_file_path);
    }
    
    ROS_INFO("Cleared and reloaded virtual walls and regions for current map");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error loading associated files: %s", e.what());
  }
}

void MapEditPanel::clearAllMessages()
{
  // 清空所有ROS话题消息，确保加载新地图时不会显示旧数据
  try
  {
    ROS_INFO("Clearing all messages...");
    status_label_->setText("Clearing existing data...");
    
    // 1. 清空地图消息 - 发布空地图
    static ros::Publisher map_pub;
    static ros::Publisher metadata_pub; 
    static ros::Publisher edited_map_pub;
    static bool publishers_initialized = false;
    
    if (!publishers_initialized)
    {
      ros::NodeHandle nh;
      map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
      metadata_pub = nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      edited_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_edited", 1, true);
      publishers_initialized = true;
      
      // 等待发布器准备就绪
      ros::Duration(0.2).sleep();
    }
    
    // 创建空地图消息
    nav_msgs::OccupancyGrid empty_map;
    empty_map.header.stamp = ros::Time::now();
    empty_map.header.frame_id = "map";
    empty_map.info.width = 0;
    empty_map.info.height = 0;
    empty_map.info.resolution = 0.05;
    empty_map.info.origin.position.x = 0.0;
    empty_map.info.origin.position.y = 0.0;
    empty_map.info.origin.position.z = 0.0;
    empty_map.info.origin.orientation.w = 1.0;
    empty_map.data.clear();
    
    // 发布空地图消息
    for (int i = 0; i < 3; ++i)
    {
      map_pub.publish(empty_map);
      edited_map_pub.publish(empty_map);
      metadata_pub.publish(empty_map.info);
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
    
    // 2. 清空虚拟墙和区域消息
    ToolManager& toolManager = ToolManager::getInstance();
    
    // 清空虚拟墙
    VirtualWallTool* wallTool = toolManager.getVirtualWallTool();
    if (wallTool)
    {
      ROS_INFO("Clearing virtual wall messages...");
      wallTool->clearVirtualWalls();
    }
    
    // 清空区域
    RegionTool* regionTool = toolManager.getRegionTool();
    if (regionTool)
    {
      ROS_INFO("Clearing region messages...");
      regionTool->clearRegions();
    }
    
    // 3. 额外确保marker话题清空
    static ros::Publisher wall_marker_pub;
    static ros::Publisher region_marker_pub;
    static bool marker_publishers_initialized = false;
    
    if (!marker_publishers_initialized)
    {
      ros::NodeHandle nh;
      wall_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("virtual_walls_markers", 1, true);
      region_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("region_markers", 1, true);
      marker_publishers_initialized = true;
      
      ros::Duration(0.1).sleep();
    }
    
    // 创建空的marker数组
    visualization_msgs::MarkerArray empty_markers;
    empty_markers.markers.clear();
    
    // 发布空marker数组
    for (int i = 0; i < 3; ++i)
    {
      wall_marker_pub.publish(empty_markers);
      region_marker_pub.publish(empty_markers);
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
    
    ROS_INFO("All messages cleared");
    status_label_->setText("Existing data cleared");
    
    // 短暂等待确保消息被接收
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error clearing messages: %s", e.what());
    status_label_->setText("Error while clearing data");
  }
}

} // end namespace ros_map_edit

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_map_edit::MapEditPanel, rviz::Panel) 