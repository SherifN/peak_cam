// Copyright (c) 2020, Sherif Nekkah
// All rights reserved.
//
// DISCLAMER:
//
//
// This package was created and used within an academic project and should
// be considered as experimental code. There may be bugs and deficiencies in the
// software. Feel free for suggestions, pull requests or any possible issue.
//
//
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "peak_cam/peak_cam_node.hpp"

namespace peak_cam
{

PeakCamNode::PeakCamNode()
{
  // constructor
}

PeakCamNode::~PeakCamNode()
{
  ROS_INFO("Shutting down");
  m_acquisitionTimer.stop();
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->WaitUntilDone();
  // closing camera und peak library
  closeDevice();
  peak::Library::Close();
  ROS_INFO("Peak library closed");
  ros::shutdown();
}

void PeakCamNode::onInit()
{
  // requried for nodelets
  m_nodeHandle = getPrivateNodeHandle();
  m_nodeHandleMT = getMTPrivateNodeHandle();

  std::string camera_topic, camera_info_url, frame_id;
  m_nodeHandle.getParam("frame_id", frame_id);
  m_nodeHandle.getParam("camera_topic", camera_topic);
  m_nodeHandle.getParam("camera_info_url", camera_info_url);

  ROS_INFO("Setting parameters to:");
  ROS_INFO("  frame_id: %s", frame_id.c_str());
  ROS_INFO("  camera_topic: %s", camera_topic.c_str());
  ROS_INFO("  camera_info_url: %s", camera_info_url.c_str());

  m_pubImage = m_nodeHandle.advertise<sensor_msgs::Image>(camera_topic, 1);
  m_pubCameraInfo =
    m_nodeHandle.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
  
  // Initialize header messages
  m_header.reset(new std_msgs::Header());
  m_header->frame_id = frame_id;

  // Initialize Camera Info Manager
  m_cameraInfoManager =
    new camera_info_manager::CameraInfoManager(
      m_nodeHandle,
      frame_id);

  m_cameraInfoManager->setCameraName(frame_id);

  if(m_cameraInfoManager->validateURL(camera_info_url)) {
    m_cameraInfoManager->loadCameraInfo(camera_info_url);
  } else {
    ROS_WARN("The Provided Camera Info URL is invalid or file does not exist:");
    ROS_WARN("  %s", camera_info_url.c_str());
    ROS_WARN("Uncalibrated Camera Info will be published...");
  }
  

  m_handleParams = boost::bind(&PeakCamNode::reconfigureRequest, this, _1, _2);
  m_paramsServer = std::make_shared<dynamic_reconfigure::Server<PeakCamConfig> >(
    m_nodeHandle);
  m_paramsServer->setCallback(m_handleParams);

  // set acqusition callback
  m_acquisitionTimer =
    m_nodeHandleMT.createTimer(ros::Duration(0.1), &PeakCamNode::acquisitionLoop, this); 
  
  peak::Library::Initialize();
  openDevice();
}

void PeakCamNode::openDevice()
{
  auto& deviceManager = peak::DeviceManager::Instance();  
  //Select Device and set Parameters Once
  while (!m_acquisitionLoopRunning)
  {
    try
    {
      // update the device manager
      deviceManager.Update();
     
      // exit program if no device was found
      if (deviceManager.Devices().empty())
      {
        ROS_INFO("No device found. Exiting program");
        // close library before exiting program
        peak::Library::Close();
        return;
      }
      
      // list all available devices
      size_t i = 0;
      ROS_INFO_ONCE("Devices available: ");
      for (const auto& deviceDescriptor : deviceManager.Devices())
      {
        ROS_INFO("  %lu: %s", i, deviceDescriptor->DisplayName().c_str());
        ++i;
      }
        
      // set i back to 0
      i = 0;
      size_t selectedDevice = 0;
      for (const auto& deviceDescriptor : deviceManager.Devices())
      {
        if (m_peakParams.selectedDevice == deviceDescriptor->SerialNumber())
        {
          ROS_INFO_ONCE("SELECTING NEW DEVICE: %lu", i);
          selectedDevice = i;
        }
        ++i;
      }
   
      // get vector of device descriptors
      auto deviceDesrciptors = deviceManager.Devices();
      // open the selected device
      m_device = deviceManager.Devices().at(selectedDevice)->OpenDevice(peak::core::DeviceAccessType::Control);
      ROS_INFO_STREAM("[PeakCamNode]: " << m_device->ModelName() << " found");
      // get the remote device node map
      m_nodeMapRemoteDevice = m_device->RemoteDevice()->NodeMaps().at(0); 
      std::vector<std::shared_ptr<peak::core::nodes::Node>> nodes = m_nodeMapRemoteDevice->Nodes();
      // sets Acquisition Parameters of the camera -> see yaml
      setDeviceParameters();
      // open the first data stream
      m_dataStream = m_device->DataStreams().at(0)->OpenDataStream(); 
      // get payload size
      auto payloadSize = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();
      
      // get number of buffers to allocate
      // the buffer count depends on your application, here the minimum required number for the data stream
      auto bufferCountMax = m_dataStream->NumBuffersAnnouncedMinRequired();
      
      // allocate and announce image buffers and queue them
      for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
      {
          auto buffer = m_dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
          m_dataStream->QueueBuffer(buffer);
      }
      // Lock critical features to prevent them from changing during acqusition
      // m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);

      // start the data stream
      m_dataStream->StartAcquisition();
      // start the device
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->WaitUntilDone();
      ROS_INFO_STREAM("[PeakCamNode]: " << m_device->ModelName() << " connected");
      m_acquisitionLoopRunning = true;
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM_ONCE("[PeakCamNode]: EXCEPTION: " << e.what());
      ROS_ERROR_STREAM("[PeakCamNode]: Device at port " << m_peakParams.selectedDevice << " not connected or must run as root!");
    }
  }
}

void PeakCamNode::setDeviceParameters()
{
  int maxWidth, maxHeight = 0;
  maxWidth = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("WidthMax")->Value();
  // ROS_INFO_STREAM("[PeakCamNode]: maxWidth '" << maxWidth << "'");
  maxHeight = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("HeightMax")->Value();
  // ROS_INFO_STREAM("[PeakCamNode]: maxHeight '" << maxHeight << "'");
  // Set Width, Height
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width")->SetValue(m_peakParams.ImageWidth);
  ROS_INFO_STREAM("[PeakCamNode]: ImageWidth is set to '" << m_peakParams.ImageWidth << "'");
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height")->SetValue(m_peakParams.ImageHeight);
  ROS_INFO_STREAM("[PeakCamNode]: ImageHeight is set to '" << m_peakParams.ImageHeight << "'");
  
  if (m_peakParams.UseOffset) {
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->SetValue(m_peakParams.OffsetWidth);
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->SetValue(m_peakParams.OffsetHeight);
  } else {
    // auto-center if UseOffset is set to False
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->SetValue((maxWidth - m_peakParams.ImageWidth) / 2);
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->SetValue((maxHeight - m_peakParams.ImageHeight) / 2);
  }

  //Set GainAuto Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainAuto")->SetCurrentEntry(m_peakParams.GainAuto);
  ROS_INFO_STREAM("[PeakCamNode]: GainAuto is set to '" << m_peakParams.GainAuto << "'");
  //Set GainSelector Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")->SetCurrentEntry(m_peakParams.GainSelector);
  ROS_INFO_STREAM("[PeakCamNode]: GainSelector is set to '" << m_peakParams.GainSelector << "'");
  //Set ExposureAuto Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ExposureAuto")->SetCurrentEntry(m_peakParams.ExposureAuto);
  ROS_INFO_STREAM("[PeakCamNode]: ExposureAuto is set to '" << m_peakParams.ExposureAuto << "'");

  //Set ExposureTime Parameter
  if(m_peakParams.ExposureAuto == "Off")
  {
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(m_peakParams.ExposureTime);
    ROS_INFO_STREAM("[PeakCamNode]: ExposureTime is set to " << m_peakParams.ExposureTime << " microseconds");
  }
  //Set AcquisitionFrameRate Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->SetValue(m_peakParams.AcquisitionFrameRate);
  ROS_INFO_STREAM("[PeakCamNode]: AcquisitionFrameRate is set to " << m_peakParams.AcquisitionFrameRate << " Hz");
  //Set Gamma Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gamma")->SetValue(m_peakParams.Gamma);
  ROS_INFO_STREAM("[PeakCamNode]: Gamma is set to " << m_peakParams.Gamma);
  //Set PixelFormat Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat")->SetCurrentEntry(m_peakParams.PixelFormat);
  ROS_INFO_STREAM("[PeakCamNode]: PixelFormat is set to '" << m_peakParams.PixelFormat << "'");

  // Set TriggerMode
  if (m_peakParams.TriggerMode == "On" ) {
    // TODO(flynneva): add more parameters for customizing trigger
    // trigger acqusition, delayed trigger, etc.
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
      ->SetCurrentEntry("ExposureStart");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")->SetCurrentEntry("On");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")
      ->SetCurrentEntry("Timer0Active");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerActivation")
      ->SetCurrentEntry("LevelHigh");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerSelector")
      ->SetCurrentEntry("Timer0");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("TimerDuration")->SetValue(500000.0);
    std::string triggerTypeStart = "Timer0";
    // set hardline trigger settings
    std::string lineIn = "Line";
    lineIn.push_back(m_peakParams.TriggerSource);  // GPIO pin number

    // already including some future logic for other trigger types
    if (triggerTypeStart == "Counter0")
    {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("CounterSelector")
       ->SetCurrentEntry("Counter0");
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("CounterTriggerSource")
       ->SetCurrentEntry(lineIn);
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("CounterTriggerActivation")
       ->SetCurrentEntry("RisingEdge");
    }
    else if (triggerTypeStart == "Timer0")
    {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerSelector")
       ->SetCurrentEntry("Timer0");
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerTriggerSource")
       ->SetCurrentEntry(lineIn);
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerTriggerActivation")
       ->SetCurrentEntry("RisingEdge");
    }
    else
    {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
       ->SetCurrentEntry(triggerTypeStart);
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")
       ->SetCurrentEntry(lineIn);
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerActivation")
       ->SetCurrentEntry("RisingEdge");
    }
  } else {
    ROS_INFO_STREAM("[PeakCamNode] No Trigger Specified, running continously");
  }
    
  // Set Parameters for ROS Image
  if (m_peakParams.PixelFormat == "Mono8") {
    m_pixelFormat = peak::ipl::PixelFormatName::Mono8;
    m_image->encoding = sensor_msgs::image_encodings::MONO8;
    m_bytesPerPixel = 1;
  } else if (m_peakParams.PixelFormat == "RGB8") {
    m_pixelFormat = peak::ipl::PixelFormatName::RGB8;
    m_image->encoding = sensor_msgs::image_encodings::RGB8;
    m_bytesPerPixel = 1;
  } else if (m_peakParams.PixelFormat == "BGR8") {
    m_pixelFormat = peak::ipl::PixelFormatName::BGR8;
    m_image->encoding = sensor_msgs::image_encodings::BGR8;
    m_bytesPerPixel = 1;
  }
}

void PeakCamNode::acquisitionLoop(const ros::TimerEvent & event)
{
  while (m_acquisitionLoopRunning) {
    try {
      m_header->stamp = ros::Time::now();
      ROS_INFO_ONCE("[PeakCamNode]: Acquisition started");
      // get buffer from data stream and process it
      auto buffer = m_dataStream->WaitForFinishedBuffer(5000);

      auto ci = m_cameraInfoManager->getCameraInfo();
      sensor_msgs::CameraInfoPtr camera_info_msg(new sensor_msgs::CameraInfo(ci));
      camera_info_msg->header = *m_header;

      const auto imageBufferSize = m_peakParams.ImageWidth * m_peakParams.ImageHeight * m_bytesPerPixel;
      // buffer processing start
      auto image = peak::BufferTo<peak::ipl::Image>(buffer).ConvertTo(m_pixelFormat);
      cv::Mat cvImage;
      if (m_peakParams.PixelFormat == "Mono8")
        cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC1);
      else
        cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC3);
      int sizeBuffer = static_cast<int>(image.ByteCount());
      // Device buffer is being copied into cv_bridge format
      std::memcpy(cvImage.data, image.Data(), static_cast<size_t>(sizeBuffer));
      // cv_bridge Image is converted to sensor_msgs/Image to publish on ROS Topic
      cv_bridge::CvImage cvBridgeImage;
      cvBridgeImage.header = *m_header;
      cvBridgeImage.encoding = m_image->encoding;
      cvBridgeImage.image = cvImage;
      m_pubImage.publish(cvBridgeImage.toImageMsg());
      m_pubCameraInfo.publish(*camera_info_msg);
      ROS_INFO_STREAM_ONCE("[PeakCamNode]: Publishing data");
      // queue buffer
      m_dataStream->QueueBuffer(buffer);
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("[PeakCamNode]: EXCEPTION: " << e.what());
      ROS_ERROR("[PeakCamNode]: Acquisition loop stopped, device may be disconnected!");
      ROS_ERROR("[PeakCamNode]: No device reset available");
      ROS_ERROR("[PeakCamNode]: Restart peak cam node!");
    }
  }
}

void PeakCamNode::closeDevice()
{
  // if device was opened, try to stop acquisition
  if (m_device) {
    try {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();
      ROS_INFO("Executing 'AcquisitionStop'");
      m_acquisitionLoopRunning = false;
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("EXCEPTION: " << e.what());
    }
  }
  // if data stream was opened, try to stop it and revoke its image buffers
  if (m_dataStream) {
    try {
      m_dataStream->KillWait(); //->KillOneWait();
      m_dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
      m_dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
      for (const auto &buffer : m_dataStream->AnnouncedBuffers()) {
        m_dataStream->RevokeBuffer(buffer);
      }
      ROS_INFO("'AcquisitionStop' Succesful");
      m_acquisitionLoopRunning = false;
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("EXCEPTION: " << e.what());
    }
  }
}

void PeakCamNode::reconfigureRequest(const PeakCamConfig &config, uint32_t level)
{
  m_peakParams.ExposureTime = config.ExposureTime;
  m_peakParams.AcquisitionFrameRate = config.AcquisitionFrameRate;
  m_peakParams.Gamma = config.Gamma;
  m_peakParams.selectedDevice = config.selectedDevice;
  m_peakParams.GainAuto = config.GainAuto;
  m_peakParams.GainSelector = config.GainSelector;
  m_peakParams.ExposureAuto = config.ExposureAuto;
  m_peakParams.PixelFormat = config.PixelFormat;
  m_peakParams.ImageHeight = config.ImageHeight;
  m_peakParams.ImageWidth = config.ImageWidth;
  m_peakParams.UseOffset = config.UseOffset;
  m_peakParams.OffsetWidth = config.OffsetWidth;
  m_peakParams.OffsetHeight = config.OffsetHeight;
  m_peakParams.TriggerMode = config.TriggerMode;
}

} // namespace peak_cam

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(peak_cam::PeakCamNode, nodelet::Nodelet)
