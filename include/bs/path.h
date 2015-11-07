#pragma once

#include<string>


const std::string kDirTop = "D:/projectAir3/";
const std::string kDirCalib = kDirTop + "BM_calibration/";
const std::string kDirPointCloud = kDirTop + "BM_point_cloud/";
const std::string kDirFeedbackLog = kDirPointCloud + "feedback_log/";
const std::string kFileCamParam = kDirCalib + "scam_param.xml";
const std::string kFileBmParam = kDirCalib + "bm_param.xml";
const std::string kFileExParam = kDirCalib + "extrinsic.xml";
const std::string kFileHomo = kDirCalib + "homography.xml";
