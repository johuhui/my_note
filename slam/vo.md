## 前端VO
---
---
### - 主线
  ```cpp
    bool VisualOdometry::addFrame ( Frame::Ptr frame )
    {
    switch ( state_ )
    {
    case INITIALIZING:
    {
        state_ = OK;
        curr_ = ref_ = frame;
        // extract features from first frame and add them into map
        extractKeyPoints();
        computeDescriptors();
        addKeyFrame();      // the first frame is a key-frame
        break;
    }
    case OK:
    {
        curr_ = frame;
        curr_->T_c_w_ = ref_->T_c_w_;
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
        poseEstimationPnP();
        if ( checkEstimatedPose() == true ) // a good estimation
        {
            curr_->T_c_w_ = T_c_w_estimated_;
            optimizeMap();
            num_lost_ = 0;
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame();
            }
        }
        else // bad estimation due to various reasons
        {
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
    }

  ```
  ---
### - 初始化
1. **提取关键点与描述符**
2. **插入到关键帧数据库**
   
   >1. 将第一帧的特征构造成mappoints并插入地图
   >2. 第一帧作为关键帧插入地图，同时第一帧也是参考帧
 ---  
### - 跟踪
1. **当前帧位姿初始化为参考帧的位姿**
2. **提取关键点与描述符**
3. **当前帧关键点与mappoints匹配**
   > 1. 将mappoints投影到当前帧，利用正深度以及投影坐标落在图像内两个条件排除mappoints
4. **用pnp算法求解并优化位姿**
   > 1. 为了保证前端的实时性，只优化位姿不优化mappoints
5. **检查位姿的有效性**
   > 1. 符合所求出的位姿的内点的数量
   > 2. 相对于参考帧的运动量（用se3的模判断）
   ```cpp
   bool VisualOdometry::checkEstimatedPose()
    {
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    return true;
    }
    ```
---
### - 关键帧的插入
1. **关键帧决策**
   > 1. 当前帧与参考帧的相对运动
   ```cpp
   bool VisualOdometry::checkKeyFrame()
    {
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
    }
    ```

  2. **插入关键帧**
     ```cpp
     map_->insertKeyFrame ( curr_ ); //还要构造新的mappoint
     ref_ = curr_;                   //更新参考帧
     ```

   


   



