#pragma once

#include<string>

/*!
"D:/projectAir3/"
@note Top directory of my research
*/
const std::string kDirTop = "D:/projectAir3/";

/*!
kDirTop + "BM_calibration/"
@note The directory of calibration and parameter files
*/
const std::string kDirCalib = kDirTop + "BM_calibration/";

/*!
kDirTop + "BM_point_cloud/"
@note The directory of output point cloud data
*/
const std::string kDirPointCloud = kDirTop + "BM_point_cloud/";

/*!
kDirCalib + "scam_param.xml"
@note The file of stereo camera parameters
*/
const std::string kFileCamParam = kDirCalib + "scam_param.xml";

/*!
kDirCalib + "bm_param.xml"
@note The file of stereo block matching parameters
*/
const std::string kFileBmParam = kDirCalib + "bm_param.xml";

/*!
kDirCalib + "extrinsic.xml"
@note The file of extrinsic parameters od this system
*/
const std::string kFileExParam = kDirCalib + "extrinsic.xml";
