#ifndef EMSGC_TOOLS_VISUALIZER_H
#define EMSGC_TOOLS_VISUALIZER_H

#include <emsgc/tools/utils.h>
#include <emsgc/container/EventQueueMat.h>
#include <emsgc/core/image_warped_events.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
namespace emsgc
{
using namespace container;
using namespace core;
namespace tools
{
const float r[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00588235294117645, 0.02156862745098032,
                  0.03725490196078418, 0.05294117647058827, 0.06862745098039214, 0.084313725490196,
                  0.1000000000000001, 0.115686274509804, 0.1313725490196078, 0.1470588235294117,
                  0.1627450980392156, 0.1784313725490196, 0.1941176470588235, 0.2098039215686274,
                  0.2254901960784315, 0.2411764705882353, 0.2568627450980392, 0.2725490196078431,
                  0.2882352941176469, 0.303921568627451, 0.3196078431372549, 0.3352941176470587,
                  0.3509803921568628, 0.3666666666666667, 0.3823529411764706, 0.3980392156862744,
                  0.4137254901960783, 0.4294117647058824, 0.4450980392156862, 0.4607843137254901,
                  0.4764705882352942, 0.4921568627450981, 0.5078431372549019, 0.5235294117647058,
                  0.5392156862745097, 0.5549019607843135, 0.5705882352941174, 0.5862745098039217,
                  0.6019607843137256, 0.6176470588235294, 0.6333333333333333, 0.6490196078431372,
                  0.664705882352941, 0.6803921568627449, 0.6960784313725492, 0.7117647058823531,
                  0.7274509803921569, 0.7431372549019608, 0.7588235294117647, 0.7745098039215685,
                  0.7901960784313724, 0.8058823529411763, 0.8215686274509801, 0.8372549019607844,
                  0.8529411764705883, 0.8686274509803922, 0.884313725490196, 0.8999999999999999,
                  0.9156862745098038, 0.9313725490196076, 0.947058823529412, 0.9627450980392158,
                  0.9784313725490197, 0.9941176470588236, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9862745098039216,
                  0.9705882352941178, 0.9549019607843139, 0.93921568627451, 0.9235294117647062,
                  0.9078431372549018, 0.892156862745098, 0.8764705882352941, 0.8607843137254902,
                  0.8450980392156864, 0.8294117647058825, 0.8137254901960786, 0.7980392156862743,
                  0.7823529411764705, 0.7666666666666666, 0.7509803921568627, 0.7352941176470589,
                  0.719607843137255, 0.7039215686274511, 0.6882352941176473, 0.6725490196078434,
                  0.6568627450980391, 0.6411764705882352, 0.6254901960784314, 0.6098039215686275,
                  0.5941176470588236, 0.5784313725490198, 0.5627450980392159, 0.5470588235294116,
                  0.5313725490196077, 0.5156862745098039, 0.5};
const float g[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0.001960784313725483, 0.01764705882352935, 0.03333333333333333,
                  0.0490196078431373, 0.06470588235294117, 0.08039215686274503, 0.09607843137254901,
                  0.111764705882353, 0.1274509803921569, 0.1431372549019607, 0.1588235294117647,
                  0.1745098039215687, 0.1901960784313725, 0.2058823529411764, 0.2215686274509804,
                  0.2372549019607844, 0.2529411764705882, 0.2686274509803921, 0.2843137254901961, 0.3,
                  0.3156862745098039, 0.3313725490196078, 0.3470588235294118, 0.3627450980392157,
                  0.3784313725490196, 0.3941176470588235, 0.4098039215686274, 0.4254901960784314,
                  0.4411764705882353, 0.4568627450980391, 0.4725490196078431, 0.4882352941176471,
                  0.503921568627451, 0.5196078431372548, 0.5352941176470587, 0.5509803921568628,
                  0.5666666666666667, 0.5823529411764705, 0.5980392156862746, 0.6137254901960785,
                  0.6294117647058823, 0.6450980392156862, 0.6607843137254901, 0.6764705882352942,
                  0.692156862745098, 0.7078431372549019, 0.723529411764706, 0.7392156862745098,
                  0.7549019607843137, 0.7705882352941176, 0.7862745098039214, 0.8019607843137255,
                  0.8176470588235294, 0.8333333333333333, 0.8490196078431373, 0.8647058823529412,
                  0.8803921568627451, 0.8960784313725489, 0.9117647058823528, 0.9274509803921569,
                  0.9431372549019608, 0.9588235294117646, 0.9745098039215687, 0.9901960784313726, 1, 1,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                  1, 1, 1, 1, 0.9901960784313726, 0.9745098039215687, 0.9588235294117649,
                  0.943137254901961, 0.9274509803921571, 0.9117647058823528, 0.8960784313725489,
                  0.8803921568627451, 0.8647058823529412, 0.8490196078431373, 0.8333333333333335,
                  0.8176470588235296, 0.8019607843137253, 0.7862745098039214, 0.7705882352941176,
                  0.7549019607843137, 0.7392156862745098, 0.723529411764706, 0.7078431372549021,
                  0.6921568627450982, 0.6764705882352944, 0.6607843137254901, 0.6450980392156862,
                  0.6294117647058823, 0.6137254901960785, 0.5980392156862746, 0.5823529411764707,
                  0.5666666666666669, 0.5509803921568626, 0.5352941176470587, 0.5196078431372548,
                  0.503921568627451, 0.4882352941176471, 0.4725490196078432, 0.4568627450980394,
                  0.4411764705882355, 0.4254901960784316, 0.4098039215686273, 0.3941176470588235,
                  0.3784313725490196, 0.3627450980392157, 0.3470588235294119, 0.331372549019608,
                  0.3156862745098041, 0.2999999999999998, 0.284313725490196, 0.2686274509803921,
                  0.2529411764705882, 0.2372549019607844, 0.2215686274509805, 0.2058823529411766,
                  0.1901960784313728, 0.1745098039215689, 0.1588235294117646, 0.1431372549019607,
                  0.1274509803921569, 0.111764705882353, 0.09607843137254912, 0.08039215686274526,
                  0.06470588235294139, 0.04901960784313708, 0.03333333333333321, 0.01764705882352935,
                  0.001960784313725483, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const float b[] = {0.5, 0.5156862745098039, 0.5313725490196078, 0.5470588235294118, 0.5627450980392157,
                  0.5784313725490196, 0.5941176470588235, 0.6098039215686275, 0.6254901960784314,
                  0.6411764705882352, 0.6568627450980392, 0.6725490196078432, 0.6882352941176471,
                  0.7039215686274509, 0.7196078431372549, 0.7352941176470589, 0.7509803921568627,
                  0.7666666666666666, 0.7823529411764706, 0.7980392156862746, 0.8137254901960784,
                  0.8294117647058823, 0.8450980392156863, 0.8607843137254902, 0.8764705882352941,
                  0.892156862745098, 0.907843137254902, 0.9235294117647059, 0.9392156862745098,
                  0.9549019607843137, 0.9705882352941176, 0.9862745098039216, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                  0.9941176470588236, 0.9784313725490197, 0.9627450980392158, 0.9470588235294117,
                  0.9313725490196079, 0.915686274509804, 0.8999999999999999, 0.884313725490196,
                  0.8686274509803922, 0.8529411764705883, 0.8372549019607844, 0.8215686274509804,
                  0.8058823529411765, 0.7901960784313726, 0.7745098039215685, 0.7588235294117647,
                  0.7431372549019608, 0.7274509803921569, 0.7117647058823531, 0.696078431372549,
                  0.6803921568627451, 0.6647058823529413, 0.6490196078431372, 0.6333333333333333,
                  0.6176470588235294, 0.6019607843137256, 0.5862745098039217, 0.5705882352941176,
                  0.5549019607843138, 0.5392156862745099, 0.5235294117647058, 0.5078431372549019,
                  0.4921568627450981, 0.4764705882352942, 0.4607843137254903, 0.4450980392156865,
                  0.4294117647058826, 0.4137254901960783, 0.3980392156862744, 0.3823529411764706,
                  0.3666666666666667, 0.3509803921568628, 0.335294117647059, 0.3196078431372551,
                  0.3039215686274508, 0.2882352941176469, 0.2725490196078431, 0.2568627450980392,
                  0.2411764705882353, 0.2254901960784315, 0.2098039215686276, 0.1941176470588237,
                  0.1784313725490199, 0.1627450980392156, 0.1470588235294117, 0.1313725490196078,
                  0.115686274509804, 0.1000000000000001, 0.08431372549019622, 0.06862745098039236,
                  0.05294117647058805, 0.03725490196078418, 0.02156862745098032, 0.00588235294117645, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0};

/* Overlay an image on another one with given transparency factor alpha */
inline void overlayImage(cv::Mat* src, cv::Mat* overlay, float alpha)
{
  for (int y = 0; y < src->rows; y++)
  {
    for (int x = 0; x < src->cols; x++)
    {
      for (int c = 0; c < src->channels(); ++c)
      {
        unsigned char overlayPx = overlay->data[y * overlay->step + x * overlay->channels() + c];
        unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
        src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - alpha) + overlayPx * alpha;
      }
    }
  }
}

/* Draw IWE given a ST volume and a motion model. */
/* The output iwe's type is CV_32FC1.               */
void drawIWE(ST_Volome& vol, MotionModel& mm, ros::Time& t_ref, OptionsWarp& opts_warp, cv::Mat* iwe)
{
  OptionsWarp opts_warp_display = opts_warp;
//  LOG(INFO) << "usePolarity (drawIWE): " << opts_warp_display.use_polarity_;
  opts_warp_display.blur_sigma_ = 0.;

  cv::Size img_size(vol.pEvtQueueMat_->width_, vol.pEvtQueueMat_->height_);
  size_t numPoint = computeImageOfWarpedEvents(mm, vol.events_involved_, t_ref.toSec(), img_size, iwe, opts_warp_display);
//  LOG(INFO) << "++++++++++++++ numPoint (drawIWE): " << numPoint;
}

/* Draw IWE given multi ST volumes and their corresponding motion model.
 * The output iwe's type is CV_32FC1.
 */
void drawIWEfromVols(
  std::map<int, ST_Volome>& mVols,
  ros::Time& t_ref,
  OptionsWarp& opts_warp,
  size_t width,
  size_t height,
  cv::Mat* iwe)
{
  OptionsWarp opts_warp_display = opts_warp;
  opts_warp_display.blur_sigma_ = 0.;

  // Create image of the overall warped events (IWE)
  *iwe = cv::Mat::zeros(height, width, CV_32FC1);
  // draw each sub IWE
  cv::Size img_size(width, height);
  cv::Mat iweROI;
  auto it = mVols.begin();
  for(;it != mVols.end(); it++)
  {
    auto & vol = it->second;
    computeImageOfWarpedEvents(
      vol.mm_,
      vol.events_involved_,
      t_ref.toSec(), img_size, &iweROI, opts_warp_display);

    // copy arbitrary ROI to IWE
    for(auto& site:vol.seg_)
    {
      size_t x = site.second.first;
      size_t y = site.second.second;
      iwe->at<float>(y,x) = iweROI.at<float>(y,x);
    }
  }
}

/* Display IWE */
void displayIWE(cv::Mat& image, OptionsWarp& opts_warp, const std::string WINDOW_NAME)
{
  if (opts_warp.use_polarity_)
  {
    // Visualize the image of warped events with the zero always at the mean grayscale level
    const float bmax = 5.0;
    image = (255.0/(2.0*bmax)) * (image + bmax);
//    LOG(INFO) << "Using polarity in display IWE.";
  }
  else
  {
    // Scale the image to full range [0,255]
    cv::normalize(image, image, 0.0, 255.0, cv::NORM_MINMAX, CV_32FC1);
    image = 255.0 - image; // invert "color": dark events over white background
//    LOG(INFO) << "Not using polarity in display IWE.";
  }

  image.convertTo(image, CV_8UC1);
  cv::imshow( WINDOW_NAME, image );
}

/* Draw a colorful circle at the given coordinate.
 * The color is determined by the label and the overall
 * number of motions. */
inline void drawPoint(
  int label,
  int max_num_motions,
  size_t x,
  size_t y,
  cv::Mat& img)
{
  if(label >= max_num_motions)
  {
    LOG(INFO) << "The label should be smaller than max_num_motion: " << label << " " << max_num_motions;
    exit(-1);
  }
  int index = floor(255.f * label / max_num_motions);
  CvScalar color = CV_RGB(255.0f * r[index], 255.0f * g[index], 255.0f * b[index]);
  //draw the point
  cv::Point point;
  point.x = x;
  point.y = y;
  cv::circle(img, point, 1, color, CV_FILLED);
}

/* Draw the colorful label map. The output's type is CV_32FC3. */
inline void drawSegmentation(
  cv::Mat& labelMap,
  const int NUM_MOTIONS,
  cv::Mat& output
)
{
  size_t width = labelMap.cols;
  size_t height = labelMap.rows;
  output = cv::Mat::zeros(height, width, CV_32FC1);
  cv::cvtColor(output,output,CV_GRAY2BGR);
  for(size_t y = 0; y < height; y++)
    for(size_t x = 0; x < width; x++)
    {
      int label = labelMap.at<uchar>(y,x);
      drawPoint(label, NUM_MOTIONS, x, y, output);
    }
//  LOG(INFO) << "(drawSegmentation) output's type: " << output.type();
}

/* Draw labeled IWE. */
inline void drawColorIWE_with_Vols_and_Labels(
  std::map<int, ST_Volome>& mVols,
  ros::Time& t_ref,
  OptionsWarp& opts_warp,
  size_t width,
  size_t height,
  cv::Mat& output
  )
{
  OptionsWarp opts_warp_display = opts_warp;
  opts_warp_display.blur_sigma_ = 0.;

  // Create image of warped events (IWE)
  output = cv::Mat::zeros(height, width, CV_32FC1);
  cv::cvtColor(output, output,CV_GRAY2BGR);

  // draw each sub IWE
  size_t num_motions = mVols.size();
  cv::Size img_size(width, height);
  cv::Mat iweROI;
  auto it = mVols.begin();
  for(;it != mVols.end(); it++)
  {
    auto & vol = it->second;
    computeImageOfWarpedEvents(
      vol.mm_,
      vol.events_involved_,
      t_ref.toSec(), img_size, &iweROI, opts_warp_display);

    cv::normalize(iweROI, iweROI, 0.0, 255.0, cv::NORM_MINMAX, CV_32FC1);

    // copy arbitrary ROI to IWE
    for(auto& site:vol.seg_)
    {
      size_t x = site.second.first;
      size_t y = site.second.second;
      int index = floor(255.f * it->first / num_motions);
      float intensity = iweROI.at<float>(y ,x);
      CvScalar color;
//      if(intensity < 0.1)
//        color = CvScalar(255.f,255.f,255.f);
//      else
      color = CvScalar(intensity * b[index], intensity * g[index], intensity * r[index]);

      //draw the point
      cv::Point point;
      point.x = x;
      point.y = y;
      cv::circle(output, point, 1, color, CV_FILLED);
    }
  }
}

/* Draw labeled IWE. */
inline void drawColorIWE_with_EventClustersGMM(
  std::map<int, EventClusterGMM>& mEvtClustersGMM,
  ros::Time& t_ref,
  OptionsWarp& opts_warp,
  size_t width,
  size_t height,
  cv::Mat& output
)
{
  OptionsWarp opts_warp_display = opts_warp;
  opts_warp_display.blur_sigma_ = 0.;

  // Create image of warped events (IWE)
  output = cv::Mat::zeros(height, width, CV_32FC1);
  cv::cvtColor(output, output,CV_GRAY2BGR);
  output = cv::Scalar::all(0.f);

  // draw each sub IWE
  size_t numLabels = mEvtClustersGMM.size();
  cv::Size img_size(width, height);
  cv::Mat iweROI;
  for(auto it = mEvtClustersGMM.begin(); it != mEvtClustersGMM.end(); it++)
  {
    auto & ec = it->second;
    std::vector<UndistortedEvent> vUndistEvents;
    vUndistEvents.reserve(ec.dqEvtSites_.size());
    for(auto& es : ec.dqEvtSites_)
    {
      vUndistEvents.push_back(es.ev_undistort_);
    }

    computeImageOfWarpedEvents_GMM_undistortion(ec.gmm_, vUndistEvents, t_ref.toSec(), img_size, &iweROI, opts_warp_display);

    cv::normalize(iweROI, iweROI, 0.0, 255.0, cv::NORM_MINMAX, CV_32FC1);

    // copy arbitrary ROI to IWE
    for(size_t y = 0; y < height; y++)
      for(size_t x = 0; x < width; x++)
      {
        int index = floor(125.f * it->first / numLabels + 130.f);
        float intensity = iweROI.at<float>(y ,x);
        CvScalar color;
        if(intensity > 10) // TODO: tune this criteria
        {
          color = CvScalar(intensity * b[index], intensity * g[index], intensity * r[index]);
          output.at<cv::Vec3f>(y,x) = cv::Vec3f(color.val[0], color.val[1], color.val[2]);
        }
      }
  }
  output.convertTo(output, CV_8UC3);
  // Turn the background white
  cv::Mat whiteBackground = cv::Mat(output.rows, output.cols, CV_8UC3, cv::Scalar(255,255,255));
  output = whiteBackground - output;
}

/* Draw seg overlaying iwe. */
inline void segOverlayingIWE(
  cv::Mat iwe,
  cv::Mat segMap,
  cv::Mat & output
  )
{
  cv::normalize(iwe, iwe, 0.0, 255.0, cv::NORM_MINMAX, CV_32FC1);
  iwe = 255.0 - iwe; // invert "color": dark events over white background
  cv::cvtColor(iwe, output,CV_GRAY2BGR);

//  segMap = segMap - cv::Scalar(0, 0, 0, 127);
  output.convertTo(output, CV_8UC3);
  segMap.convertTo(segMap, CV_8UC3);
  overlayImage(&output, &segMap, 0.5f);
}

/**
 * \brief Concatenate two matrices horizontally
 * \param[in] Mat A and B
 * \param[out] Mat C = [A, B]
*/
void concatHorizontal(const cv::Mat& A, const cv::Mat& B, cv::Mat* C)
{
  CHECK_EQ(A.rows, B.rows) << "Input arguments must have same number of rows";
  CHECK_EQ(A.type(), B.type()) << "Input arguments must have the same type";

  *C = cv::Mat(A.rows, A.cols + B.cols, A.type());
  A.copyTo((*C)(cv::Rect(0,0,A.cols,A.rows)));
  B.copyTo((*C)(cv::Rect(A.cols,0,B.cols,B.rows)));
}

void saveIWEtoTXT(cv::Mat& img, const std::string & savePath)
{
  std::ofstream of;
  of.open(savePath, std::ofstream::out);
  if(of.is_open())
  {
    size_t width = img.cols;
    size_t height = img.rows;
    for(size_t y = 0; y < height; y++)
    {
      for(size_t x = 0; x < width; x++)
      {
        of << img.at<double>(y,x);
        if(x == width - 1)
        {
          if(y != height - 1)
            of << "\n";
        }
        else
          of << " ";
      }
    }
  }
  of.close();
}

}
}

#endif //EMSGC_TOOLS_VISUALIZER_H