
// Copyright 2019 Donal McLaughlin
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "badslam/input_kinect2.h"

#ifdef HAVE_FREENECT2

namespace vis {

Kinect2InputThread::~Kinect2InputThread() {
  exit_ = true;
  if (thread_) {
    thread_->join();
  }
}

void Kinect2InputThread::Start(RGBDVideo<Vec3u8, u16>* rgbd_video, float* depth_scaling) {
  rgbd_video_ = rgbd_video;
  std::string serial;
  
  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return;
  }
  serial = freenect2.getDefaultDeviceSerialNumber();
  int types = libfreenect2::Frame::Color | libfreenect2::Frame::Depth;
  
  pipeline = make_shared<libfreenect2::CpuPacketPipeline>();
  kinect2.reset(freenect2.openDevice(serial, pipeline.get()));
  

  size_color.width = 1920;
  size_color.height = 1080;
  size_ir.width = 512;
  size_ir.height = 424;
  // size_ir = new cv::Size(512, 424);

  listener.reset(new libfreenect2::SyncMultiFrameListener(types));

  kinect2->setColorFrameListener(listener.get());
  kinect2->setIrAndDepthFrameListener(listener.get());

  auto color_params = kinect2->getColorCameraParams();
  auto ir_params = kinect2->getIrCameraParams();
  align_ = make_shared<libfreenect2::Registration>(ir_params, color_params);
  
  // Set color camera
  float color_parameters[4];
  color_parameters[0] = color_params.fx;
  color_parameters[1] = color_params.fy;
  color_parameters[2] = color_params.cx;
  color_parameters[3] = color_params.cy;
  

  // ir_parameters[0] = ir_params.fx;
  // ir_parameters[1] = ir_params.fy;
  // ir_parameters[2] = ir_params.cx;
  // ir_parameters[3] = ir_params.cy;


  rgbd_video->color_camera_mutable()->reset(new PinholeCamera4f(
      size_color.width, size_color.height, color_parameters));
  // Set depth camera to be the same as the color camera
  rgbd_video->depth_camera_mutable()->reset(new PinholeCamera4f(
      size_color.width, size_color.height, color_parameters));

  // rgbd_video->depth_camera_mutable()->reset(new PinholeCamera4f(
  //     size_ir.width, size_ir.height, ir_parameters));

  // Start thread
  exit_ = false;
  kinect2.get()->start();
  std::cout << "Starting thread" << std::endl;
  thread_.reset(new thread(std::bind(&Kinect2InputThread::ThreadMain, this)));
}

void Kinect2InputThread::GetNextFrame() {
  // Wait for the next frame
  unique_lock<mutex> lock(queue_mutex);
  while (depth_image_queue.empty()) {
    new_frame_condition_.wait(lock);
  }
  
  shared_ptr<Image<u16>> depth_image = depth_image_queue.front();
  depth_image_queue.erase(depth_image_queue.begin());
  shared_ptr<Image<Vec3u8>> color_image = color_image_queue.front();
  color_image_queue.erase(color_image_queue.begin());
  
  lock.unlock();
  
  // Add the frame to the RGBDVideo object
  rgbd_video_->depth_frames_mutable()->push_back(
      ImageFramePtr<u16, SE3f>(new ImageFrame<u16, SE3f>(depth_image)));
  rgbd_video_->color_frames_mutable()->push_back(
      ImageFramePtr<Vec3u8, SE3f>(new ImageFrame<Vec3u8, SE3f>(color_image)));
}

void Kinect2InputThread::ThreadMain() {
  libfreenect2::FrameMap frames;
  std::cout << "Main thread first line"<< std::endl;
  while (true) {
    if (exit_ || !listener.get()->waitForNewFrame(frames, 5*1000)) {
        LOG(FATAL) << "No frames from Kinect in 5 second, closing!";
        kinect2.get()->stop(); 
      break;
    }
    cv::Mat depth_mat, color_mat;
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);


    libfreenect2::Frame *rgb_frame = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth_frame = frames[libfreenect2::Frame::Depth];

    align_->apply(rgb_frame, depth_frame, &undistorted, &registered);
    listener.get()->release(frames);
    unique_lock<mutex> lock(queue_mutex);
  
    
    shared_ptr<Image<u16>> depth_image(new Image<u16>(undistorted.width, undistorted.height));
    // undistorted.width * undistorted.bytes_per_pixel

    depth_image->SetTo(reinterpret_cast<const u16*>(undistorted.data), undistorted.width * undistorted.bytes_per_pixel);
    depth_image_queue.push_back(depth_image);


    shared_ptr<Image<Vec3u8>> color_image(new Image<Vec3u8>(registered.width, registered.height));
    color_image->SetTo(reinterpret_cast<const Vec3u8*>(registered.data), registered.width * registered.bytes_per_pixel);

    color_image_queue.push_back(color_image);
    // LOG(INFO) >> std::to_string(color_image.get());
    lock.unlock();
    new_frame_condition_.notify_all();
  }
}

}

#endif  // HAVE_FREENECT2
