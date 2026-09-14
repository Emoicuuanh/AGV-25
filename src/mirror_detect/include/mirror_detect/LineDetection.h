#ifndef LINEDETECTION_H
#define LINEDETECTION_H

#include <mirror_detect/Headers.h>

class LineDetection {
public:
  LineDetection() {}
  ~LineDetection() {}

  std_msgs::Header header_;
  //! RANSAC Maximum Iterations
  int RS_max_iter_;
  //! RANSAC Minimum Inliers
  int RS_min_inliers_;
  //! RANSAC Distance Threshold
  double RS_dist_thresh_;
  //! Perform RANSAC after Clustering Points
  bool RANSAC_on_clusters_;

  void setHeader(std_msgs::Header header) { header_ = header; }

  void setParams(int RS_max_iter, int RS_min_inliers, double RS_dist_thresh) {
    RS_max_iter_ = RS_max_iter;
    RS_min_inliers_ = RS_min_inliers;
    RS_dist_thresh_ = RS_dist_thresh;
  }
  mirror_detect::Line
  rosifyLineInten(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudPCLPtr,
                  pcl::PointIndices::Ptr &indicesPCLPtr,
                  pcl::ModelCoefficients::Ptr &coefficientsPCLPtr) {
    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
    pcl_msgs::PointIndicesPtr indicesMsgPtr(new pcl_msgs::PointIndices);
    pcl_msgs::ModelCoefficientsPtr coefficientsMsgPtr(
        new pcl_msgs::ModelCoefficients);

    pcl::toROSMsg(*cloudPCLPtr, *cloudMsgPtr);
    //      pcl_conversions::moveFromPCL(*indicesPCLPtr,*indicesMsgPtr);
    //      pcl_conversions::moveFromPCL(*coefficientsPCLPtr,*coefficientsMsgPtr);

    //    ROS_INFO_STREAM("rosifyLine: CONVERTING CURRENT LINE TO ROS MSG");
    mirror_detect::Line line;
    //    line.lineID.data = lines_.lines.size();
    line.cloud = *cloudMsgPtr;
    line.points.indices = indicesPCLPtr->indices;
    line.coefficients.values = coefficientsPCLPtr->values;
    //    ROS_INFO_STREAM("rosifyLine: GETTING CENTROID");
    Eigen::Vector4f centroidVec4f;
    pcl::compute3DCentroid(*cloudPCLPtr, centroidVec4f);
    geometry_msgs::Pose centroid;
    centroid.position.x = double(centroidVec4f[0]);
    centroid.position.y = double(centroidVec4f[1]);
    centroid.position.z = double(centroidVec4f[2]);
    // Use RANSAC coefficients for orientation as it's better at ignoring outliers
    double vx = coefficientsPCLPtr->values[3];
    double vy = coefficientsPCLPtr->values[4];
    double yaw = std::atan2(vy, vx);

    // --- SỬ DỤNG HƯỚNG RANSAC + CARDINAL SNAPPING + ROBOT REFERENCE ---
    // Tính 2 hướng vuông góc từ RANSAC
    double ang1 = yaw + M_PI / 2.0;
    double ang2 = yaw - M_PI / 2.0;

    // --- TẠM THỜI BỎ LÀM TRÒN GÓC (CARDINAL SNAPPING) ---
    // Dùng trực tiếp hướng từ RANSAC để kiểm tra lý do bị chéo
    double to_robot_x = -centroid.position.x;
    double to_robot_y = -centroid.position.y;
    double dot1 = std::cos(ang1) * to_robot_x + std::sin(ang1) * to_robot_y;
    double dot2 = std::cos(ang2) * to_robot_x + std::sin(ang2) * to_robot_y;
    
    double final_yaw = (dot1 > dot2) ? ang1 : ang2;

    // Chuẩn hóa góc về [-PI, PI]
    while (final_yaw > M_PI) final_yaw -= 2 * M_PI;
    while (final_yaw < -M_PI) final_yaw += 2 * M_PI;

    centroid.orientation.x = 0.0;
    centroid.orientation.y = 0.0;
    centroid.orientation.z = std::sin(final_yaw / 2.0);
    centroid.orientation.w = std::cos(final_yaw / 2.0);
    line.centroid = centroid;
    //    ROS_INFO_STREAM("rosifyLine: GETTING SEGMENT");
    //  line.segment = getSegment(cloudPCLPtr);
    //    ROS_INFO_STREAM("rosifyLine: GETTING EUCLIDEAN DISTANCE");
    //  line.length.data = getEuclideanDistance(line);
    ////    ROS_INFO_STREAM("rosifyLine: UPDATING SEGMENT LIST");
    //    updateSegmentList(line);
    ////    ROS_INFO_STREAM("rosifyLine: UPDATING LINE LIST");
    //    updateLineList(line);
    ////    ROS_INFO_STREAM("rosifyLine: MARKING LINE");
    //    markLine(line);

    line.header = header_;
    return line;
  }
  ///////////////// BEGIN ROSIFY LINE /////////////////
  mirror_detect::Line
  rosifyLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPCLPtr,
             pcl::PointIndices::Ptr &indicesPCLPtr,
             pcl::ModelCoefficients::Ptr &coefficientsPCLPtr) {
    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
    pcl_msgs::PointIndicesPtr indicesMsgPtr(new pcl_msgs::PointIndices);
    pcl_msgs::ModelCoefficientsPtr coefficientsMsgPtr(
        new pcl_msgs::ModelCoefficients);

    pcl::toROSMsg(*cloudPCLPtr, *cloudMsgPtr);
    //      pcl_conversions::moveFromPCL(*indicesPCLPtr,*indicesMsgPtr);
    //      pcl_conversions::moveFromPCL(*coefficientsPCLPtr,*coefficientsMsgPtr);

    //    ROS_INFO_STREAM("rosifyLine: CONVERTING CURRENT LINE TO ROS MSG");
    mirror_detect::Line line;
    //    line.lineID.data = lines_.lines.size();
    line.cloud = *cloudMsgPtr;
    line.points.indices = indicesPCLPtr->indices;
    line.coefficients.values = coefficientsPCLPtr->values;
    //    ROS_INFO_STREAM("rosifyLine: GETTING CENTROID");
    // line.centroid = getCentroid(cloudPCLPtr);
    //    ROS_INFO_STREAM("rosifyLine: GETTING SEGMENT");
    //  line.segment = getSegment(cloudPCLPtr);
    //  ROS_INFO_STREAM("rosifyLine: GETTING EUCLIDEAN DISTANCE");
    //  line.length.data = getEuclideanDistance(line);
    ////    ROS_INFO_STREAM("rosifyLine: UPDATING SEGMENT LIST");
    //    updateSegmentList(line);
    ////    ROS_INFO_STREAM("rosifyLine: UPDATING LINE LIST");
    //    updateLineList(line);
    ////    ROS_INFO_STREAM("rosifyLine: MARKING LINE");
    //    markLine(line);

    line.header = header_;
    return line;
  }
  ///////////////// END ROSIFY LINE /////////////////

  ///////////////// BEGIN RANSAC LINE /////////////////
  /// \brief RansacLine
  /// \param inputCloudPtr
  /// \param maxIterations
  /// \param distanceThreshold
  /// \return lines
  ///
  mirror_detect::LineArray
  getRansacLinesInten(pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudPtr) {
    pcl::PointCloud<pcl::PointXYZI> copyInputCloud = *inputCloudPtr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr copyInputCloudPtr(
        new pcl::PointCloud<pcl::PointXYZI>(copyInputCloud));
    copyInputCloudPtr->header.seq = inputCloudPtr->header.seq;
    pcl::ModelCoefficients::Ptr coefficientsPtr(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(RS_max_iter_);
    seg.setDistanceThreshold(RS_dist_thresh_);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lineVectorPCL;

    mirror_detect::LineArray lines; // Thay đổi LineArray thành CircleArray

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
              pcl::PointCloud<pcl::PointXYZI>::Ptr>
        segResult;

    pcl::PointCloud<pcl::PointXYZI>::Ptr inCloudPtr, outCloudPtr;
    inCloudPtr = copyInputCloudPtr;

    pcl::PointCloud<pcl::PointXYZI> combinedCirclesCloud;

    while (true) {
      if (inCloudPtr->size() <= RS_min_inliers_) {
        ROS_INFO_STREAM(std::endl
                        << "RANSAC: Less than " << RS_min_inliers_
                        << " points left");
        break;
      }

      seg.setInputCloud(inCloudPtr);
      seg.segment(*inliersPtr, *coefficientsPtr);

      if (inliersPtr->indices.size() <= RS_min_inliers_) {
        ROS_INFO_STREAM("RANSAC: Less than " << RS_min_inliers_
                                             << " inliers left");
        break;
      }

      segResult = SeparateCloudsInten(inliersPtr, inCloudPtr);

      pcl::PointCloud<pcl::PointXYZI>::Ptr circleCloudPCLPtr(
          new pcl::PointCloud<pcl::PointXYZI>);
      *circleCloudPCLPtr = *segResult.first;
      circleCloudPCLPtr->width = inliersPtr->indices.size();
      circleCloudPCLPtr->height = 1;
      circleCloudPCLPtr->is_dense = true;

      lineVectorPCL.push_back(circleCloudPCLPtr);
      combinedCirclesCloud += *circleCloudPCLPtr;
      Eigen::Vector3f point_on_line(coefficientsPtr->values[0],
                                    coefficientsPtr->values[1],
                                    coefficientsPtr->values[2]);
      Eigen::Vector3f direction(coefficientsPtr->values[3],
                                coefficientsPtr->values[4],
                                coefficientsPtr->values[5]);

      pcl::PointXYZI min_point, max_point;
      float min_proj = std::numeric_limits<float>::max();
      float max_proj = -std::numeric_limits<float>::max();

      for (const auto &idx : inliersPtr->indices) {
        pcl::PointXYZI point = circleCloudPCLPtr->points[idx];
        Eigen::Vector3f vec_point(point.x, point.y, point.z);

        float projection = direction.dot(vec_point - point_on_line);

        if (projection < min_proj) {
          min_proj = projection;
          min_point = point;
        }
        if (projection > max_proj) {
          max_proj = projection;
          max_point = point;
        }
      }

      float length = sqrt(pow(max_point.x - min_point.x, 2) +
                          pow(max_point.y - min_point.y, 2) +
                          pow(max_point.z - min_point.z, 2));
      mirror_detect::Line line =
          rosifyLineInten(circleCloudPCLPtr, inliersPtr, coefficientsPtr);
      inCloudPtr = segResult.second;
      line.length.data = length;
      // mirror_detect::Line line = rosifyLineInten(lineCloudPCLPtr, inliersPtr,
      // coefficientsPtr);

      line.header = line.cloud.header = header_;
      //      ROS_INFO_STREAM("RANSAC: Adding Line " << line.lineID.data << " at
      //      index " << lines.lines.size() << " for Cluster ID " <<
      //      copyInputCloudPtr->header.seq);
      lines.lines.push_back(line);
    }
    pcl::toROSMsg(combinedCirclesCloud, lines.combinedCloud);

    //    ROS_INFO_STREAM("RANSAC RETURNING " << lines.lines.size() << " FOUND
    //    LINES ");
    lines.header = lines.combinedCloud.header = header_;
    // std::cout << std::endl;
    return lines;
  }
  mirror_detect::LineArray
  getRansacLines(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudPtr) {
    pcl::PointCloud<pcl::PointXYZRGB> copyInputCloud = *inputCloudPtr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr copyInputCloudPtr(
        new pcl::PointCloud<pcl::PointXYZRGB>(copyInputCloud));
    copyInputCloudPtr->header.seq = inputCloudPtr->header.seq;
    // std::cout << std::endl;
    //    ROS_INFO_STREAM("RANSAC Called on Cloud ID " <<
    //    copyInputCloudPtr->header.seq);
    pcl::ModelCoefficients::Ptr coefficientsPtr(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(RS_max_iter_);
    seg.setDistanceThreshold(RS_dist_thresh_);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> lineVectorPCL;

    mirror_detect::LineArray lines;

    std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
        segResult;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr, outCloudPtr;
    inCloudPtr = copyInputCloudPtr;

    pcl::PointCloud<pcl::PointXYZRGB> combinedLinesCloud;

    while (true) {

      if (inCloudPtr->size() <= RS_min_inliers_) {
        //        ROS_INFO_STREAM(std::endl << "RANSAC: Less than " <<
        //        RS_min_inliers_ << " points left");
        break;
      }

      // Segment the current largest line component from the input cloud
      seg.setInputCloud(inCloudPtr);
      seg.segment(*inliersPtr, *coefficientsPtr);

      if (inliersPtr->indices.size() <= RS_min_inliers_) {
        //        ROS_INFO_STREAM("RANSAC: Less than " << RS_min_inliers_ << "
        //        inliers left");
        break;
      }
      //      ROS_INFO_STREAM("RANSAC: Detected Line ID " <<
      //      lines_.lines.size()); ROS_INFO_STREAM("Inlier Points" <<
      //      inliersPtr->indices << "");
      segResult = SeparateClouds(inliersPtr, inCloudPtr);

      // Create new cloud for detected line
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloudPCLPtr(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      *lineCloudPCLPtr = *segResult.first;
      lineCloudPCLPtr->width = inliersPtr->indices.size();
      lineCloudPCLPtr->height = 1;
      lineCloudPCLPtr->is_dense = true;

      //      ROS_INFO_STREAM("RANSAC: Coloring Line at index " <<
      //      lineVectorPCL.size());
      ColorCloud(lineCloudPCLPtr, lineVectorPCL.size());

      // Add detected line to output cloud and lines array
      lineVectorPCL.push_back(lineCloudPCLPtr);
      //      ROS_INFO_STREAM("RANSAC: Added Line to PCLVector at index "<<
      //      lineVectorPCL.size() - 1 << " Total lines in Vector: " <<
      //      lineVectorPCL.size());

      combinedLinesCloud += *lineCloudPCLPtr;

      mirror_detect::Line line =
          rosifyLine(lineCloudPCLPtr, inliersPtr, coefficientsPtr);
      //      line.centroid = getCentroid(lineCloudPCLPtr);
      //      line.segment = getSegment(lineCloudPCLPtr);
      //      line.length.data = getEuclideanDistance(line);
      //      updateSegmentList(line);
      //      updateLineList(line);
      //      markLine(line);

      //        ROS_INFO_STREAM("ROSIFYING LINE-- with " <<
      //        line.points.indices.size() << " points and coefficients " <<
      //        line.coefficients);
      //      line =
      //      printLineInfo(line);
      line.header = line.cloud.header = header_;
      //      ROS_INFO_STREAM("RANSAC: Adding Line " << line.lineID.data << " at
      //      index " << lines.lines.size() << " for Cluster ID " <<
      //      copyInputCloudPtr->header.seq);
      lines.lines.push_back(line);
      //      ROS_INFO_STREAM("RANSAC: Total lines in MSG: " <<
      //      lines.lines.size());

      // Assign cloud of outliers to new input cloud
      inCloudPtr = segResult.second;
      //      currentCloudLineID++;
      // std::cout << std::endl;
    }

    //    ROS_INFO_STREAM("RANSAC: " << lineVectorPCL.size() << " lines found");

    pcl::toROSMsg(combinedLinesCloud, lines.combinedCloud);

    //    ROS_INFO_STREAM("RANSAC RETURNING " << lines.lines.size() << " FOUND
    //    LINES ");
    lines.header = lines.combinedCloud.header = header_;
    // std::cout << std::endl;
    return lines;
  }

  ////////////////////////////////// END RANSAC LINE
  /////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////////////////

  ///////////////// BEGIN RANSAC LINE ON CLUSTERS /////////////////

  void
  getRansacLinesOnClusterInten(mirror_detect::ClusterArray::Ptr clustersPtr,
                               mirror_detect::LineArray::Ptr linesPtr) {
    mirror_detect::LineArray lines;

    if (clustersPtr->clusters.size() == 0) {
      // ROS_WARN_STREAM("RAN-CLUS-- WARNING: NO CLUSTERS AVAILABLE");
      //      return lines;
    } else {
      // ROS_INFO_STREAM("RAN-CLUS-- " << clustersPtr->clusters.size() << "
      // AVAILABLE CLUSTERS");
    }

    //        ROS_INFO_STREAM("lines.combinedCloud.frame_id " <<
    //        lines.combinedCloud.header.frame_id);
    //        sensor_msgs::PointCloud2::Ptr linesCombinedCloudPtr (new
    //        sensor_msgs::PointCloud2(lines.combinedCloud));

    lines.header = lines.combinedCloud.header = header_;
    pcl::PointCloud<pcl::PointXYZI> linesCombinedPCL;
    int lineID = 0;

    // Iterate through clusters
    for (std::vector<mirror_detect::Cluster>::iterator cit =
             clustersPtr->clusters.begin();
         cit != clustersPtr->clusters.end(); cit++) {
      int clusterLineID = 0;
      //        ROS_INFO_STREAM("RAN-CLUS--RANSACING THROUGH CLUSTER ID " <<
      //        cit->clusterID.data);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLPtr(
          new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(cit->cloud, *cloudPCLPtr);
      cloudPCLPtr->header.seq = cit->clusterID.data;

      mirror_detect::LineArray currentLines = getRansacLinesInten(cloudPCLPtr);
      currentLines.header = currentLines.combinedCloud.header = header_;

      ROS_INFO_STREAM("RAN-CLUS-- DETECTED " << currentLines.lines.size()
                                             << " LINES FROM CLUSTER ID "
                                             << cit->clusterID.data);

      for (std::vector<mirror_detect::Line>::iterator clit =
               currentLines.lines.begin();
           clit != currentLines.lines.end();
           clit++, lineID++, clusterLineID++) {
        //            ROS_INFO_STREAM("RAN-CLUS--ADDING DETECTED LINE ID " <<
        //            lineID); currentLines.lines.at(lineID).clusterID =
        //            cit->clusterID;
        clit->clusterID = cit->clusterID;
        currentLines.lines.at(clusterLineID).lineID.data = lineID;
        lines.lines.push_back(*clit);

        cit->lines.lines.push_back(*clit);
      }

      //        ROS_INFO_STREAM("RAN-CLUS--COMBINING CLOUDS OF DETECTED LINES
      //        FROM CLUSTER ID " << cit->clusterID.data);
      //        ROS_INFO_STREAM("RAN-CLUS--CLUSTER ID " << cit->clusterID.data
      //        << " HAS " << cit->lines.lines.size() << " LINES");
      pcl::PointCloud<pcl::PointXYZI> currentLinesCloudPCL;
      pcl::fromROSMsg(currentLines.combinedCloud, currentLinesCloudPCL);
      linesCombinedPCL = linesCombinedPCL + currentLinesCloudPCL;
    }

    //    ROS_INFO_STREAM("RAN-CLUS--RETURNING ALL DETECTED LINES FROM ALL
    //    CLUSTERS");
    pcl::toROSMsg(linesCombinedPCL, lines.combinedCloud);
    lines.header = lines.combinedCloud.header = header_;
    //    ROS_INFO_STREAM("lines.combinedCloud.frame_id " <<
    //    lines.combinedCloud.header.frame_id);
    // std::cout << std::endl;
    *linesPtr = lines;
    //    return lines;
  }
  ///////////////// END RANSAC LINE ON CLUSTERS /////////////////
  void getRansacLinesOnCluster(mirror_detect::ClusterArray::Ptr clustersPtr,
                               mirror_detect::LineArray::Ptr linesPtr) {
    mirror_detect::LineArray lines;

    if (clustersPtr->clusters.size() == 0) {
      // ROS_WARN_STREAM("RAN-CLUS-- WARNING: NO CLUSTERS AVAILABLE");
      //      return lines;
    } else {
      // ROS_INFO_STREAM("RAN-CLUS-- " << clustersPtr->clusters.size() << "
      // AVAILABLE CLUSTERS");
    }

    //        ROS_INFO_STREAM("lines.combinedCloud.frame_id " <<
    //        lines.combinedCloud.header.frame_id);
    //        sensor_msgs::PointCloud2::Ptr linesCombinedCloudPtr (new
    //        sensor_msgs::PointCloud2(lines.combinedCloud));

    lines.header = lines.combinedCloud.header = header_;
    pcl::PointCloud<pcl::PointXYZRGB> linesCombinedPCL;
    int lineID = 0;

    // Iterate through clusters
    for (std::vector<mirror_detect::Cluster>::iterator cit =
             clustersPtr->clusters.begin();
         cit != clustersPtr->clusters.end(); cit++) {
      int clusterLineID = 0;
      //        ROS_INFO_STREAM("RAN-CLUS--RANSACING THROUGH CLUSTER ID " <<
      //        cit->clusterID.data);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCLPtr(
          new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(cit->cloud, *cloudPCLPtr);
      cloudPCLPtr->header.seq = cit->clusterID.data;

      mirror_detect::LineArray currentLines = getRansacLines(cloudPCLPtr);
      currentLines.header = currentLines.combinedCloud.header = header_;

      //        ROS_INFO_STREAM("RAN-CLUS-- DETECTED " <<
      //        currentLines.lines.size() << " LINES FROM CLUSTER ID " <<
      //        cit->clusterID.data);

      for (std::vector<mirror_detect::Line>::iterator clit =
               currentLines.lines.begin();
           clit != currentLines.lines.end();
           clit++, lineID++, clusterLineID++) {
        //            ROS_INFO_STREAM("RAN-CLUS--ADDING DETECTED LINE ID " <<
        //            lineID); currentLines.lines.at(lineID).clusterID =
        //            cit->clusterID;
        clit->clusterID = cit->clusterID;
        currentLines.lines.at(clusterLineID).lineID.data = lineID;
        lines.lines.push_back(*clit);

        cit->lines.lines.push_back(*clit);
      }

      //        ROS_INFO_STREAM("RAN-CLUS--COMBINING CLOUDS OF DETECTED LINES
      //        FROM CLUSTER ID " << cit->clusterID.data);
      //        ROS_INFO_STREAM("RAN-CLUS--CLUSTER ID " << cit->clusterID.data
      //        << " HAS " << cit->lines.lines.size() << " LINES");
      pcl::PointCloud<pcl::PointXYZRGB> currentLinesCloudPCL;
      pcl::fromROSMsg(currentLines.combinedCloud, currentLinesCloudPCL);
      linesCombinedPCL = linesCombinedPCL + currentLinesCloudPCL;
    }

    //    ROS_INFO_STREAM("RAN-CLUS--RETURNING ALL DETECTED LINES FROM ALL
    //    CLUSTERS");
    pcl::toROSMsg(linesCombinedPCL, lines.combinedCloud);
    lines.header = lines.combinedCloud.header = header_;
    //    ROS_INFO_STREAM("lines.combinedCloud.frame_id " <<
    //    lines.combinedCloud.header.frame_id);
    // std::cout << std::endl;
    *linesPtr = lines;
    //    return lines;
  }
  ///////////////// END RANSAC LINE ON CLUSTERS /////////////////
};

#endif // LINEDETECTION_H

