/*********************************************************************************
 *  Mesh Mapping- Code reference for the paper Lucas and Chli - IROS 2016
 *  Copyright (c) 2017, Vision for Robotics Lab / ETH Zurich
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Created on: Jan 1, 2017
 *      Author: Lucas Teixeira (lteixeira@mavt.ethz.ch)
 *
 *
 *********************************************************************************/


#ifndef MESH_BASED_MAPPING_MESH_MAPPING_H_
#define MESH_BASED_MAPPING_MESH_MAPPING_H_




//#define FADE2D_EXPORT
//#define GEOM_PSEUDO3D GEOM_FALSE
//#include <Fade_2D.h>
#include <Fade_2D.h>

#define MIN_CAM_DISTANCE 0.05

#include <opencv2/core/core.hpp>


typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >
VecPoint3F;

namespace mesh_based_mapping {

/**

*/
void filteringAndSmoothing(VecPoint3F &points3d,
                           std::vector<GEOM_FADE2D::Triangle2 *> triangles,
                           std::vector<bool> &blacklist,
                           const double laplaceAlpha = 0.1,
                           const unsigned int smoothingIteration = 3,
                           const double maxDelta = 20) {



  if (triangles.size() < 2) {
    return;
  }

  double mean = 0;
  double std = 0;

  std::vector<double> maxEdge(triangles.size());
  std::vector<bool> borderPoint(points3d.size(), false);
  std::vector<bool> borderTriangle(triangles.size(), false);

  //TODO put back the border detection.

  //remove the big edges in 2D
  for (unsigned int i = 0; i < triangles.size();
       i++) {
    GEOM_FADE2D::Triangle2 *t = triangles[i];
    maxEdge[i] = cv::max(t->getSquaredEdgeLength(0),
                         cv::max(t->getSquaredEdgeLength(1), t->getSquaredEdgeLength(2)));
    mean += maxEdge[i] ;
  }

  mean /=  triangles.size();

  for (unsigned int i = 0; i < triangles.size(); i++) {
    double delta = maxEdge[i] - mean;
    std += delta * delta;
  }

  std = sqrt(std / triangles.size());
  double threshold = std;

  for (unsigned int i = 0; i < triangles.size(); i++) {
    GEOM_FADE2D::Triangle2 *t = triangles[i];
    double diff = maxEdge[i] - mean;

    if ((std::abs(diff) > threshold) || (borderTriangle[i] && diff > 0.6 * std)
        || (borderTriangle[i] &&
            (std::abs((points3d[t->getCorner(0)->getCustomIndex()] - points3d[t->getCorner(
                         1)->getCustomIndex()]).norm())) > 0.3)
        || (borderTriangle[i] &&
            (std::abs((points3d[t->getCorner(1)->getCustomIndex()] - points3d[t->getCorner(
                         2)->getCustomIndex()]).norm())) > 0.3)
        || (borderTriangle[i] &&
            (std::abs((points3d[t->getCorner(2)->getCustomIndex()] - points3d[t->getCorner(
                         0)->getCustomIndex()]).norm())) > 0.3)) {
      blacklist[i] = true;
    }
  }

  //smooth
  std::vector<std::set<unsigned int>> adjTable(points3d.size());
  VecPoint3F points3dNew(points3d.size());

  for (unsigned int i = 0; i < triangles.size(); i++) { //build adjcense table
    if (blacklist[i]) {
      continue;
    }

    for (unsigned int j = 0; j < 3 ; j++) {
      GEOM_FADE2D::Point2 *ptA = triangles[i]->getCorner(j);
      GEOM_FADE2D::Point2 *ptB = triangles[i]->getCorner((j + 1) % 3);
      adjTable[ptA->getCustomIndex()].insert(ptB->getCustomIndex());
      adjTable[ptB->getCustomIndex()].insert(ptA->getCustomIndex());
    }
  }



  for (unsigned int k = 0; k < smoothingIteration ; k++) {
    for (unsigned int i = 0; i < adjTable.size(); i++) {
      std::set<unsigned int> &l = adjTable[i];
      Eigen::Vector3f meanPt(0, 0, 0);

      if (!borderPoint[i] && l.size() > 0) {
        for (std::set<unsigned int>::iterator it = l.begin(); it != l.end(); ++it) {
          meanPt += points3d[*it];
        }

        meanPt /= l.size();

        Eigen::Vector3f delta = meanPt - points3d[i];

        if (delta.norm() < maxDelta) {
          points3dNew[i] = (laplaceAlpha * (meanPt - points3d[i])) +
                           points3d[i]; // smoothing
        } else {
          if (points3d[i][2] != 0) {
            points3dNew[i] =  points3d[i] * (meanPt[2] /
                                             points3d[i][2]);// replaces point3D Z by meanPt Z
          } else {
            points3dNew[i] =
              meanPt;//replace the point by the meanPt instead of to blacklist it for better visualization.
          }
        }
      } else {
        //do not move point in the border
        points3dNew[i] =  points3d[i];
      }
    }

    points3d = points3dNew;
  }
}

/**

*/
void projectLandmarks(const double focalU, const double focalV,
                      const double centerU, const double centerV,
                      const double dimU, const double dimV,
                      const double minQuality,
                      okvis::kinematics::Transformation &T_WCRef,
                      const okvis::MapPointVector &matchedLandmarks,
                      VecPoint3F &filteredLandmarks,
                      std::vector<GEOM_FADE2D::Point2> &landmarks2D) {

  okvis::kinematics::Transformation T_CRefW =  T_WCRef.inverse();

  for (unsigned int i = 0 ; i < matchedLandmarks.size() ; i++) {

    if (matchedLandmarks[i].point(3) < 1.0e-8) {
      continue;
    }

    if (matchedLandmarks[i].quality < minQuality) {
      continue;
    }

    Eigen::Vector4d pt_CRef = T_CRefW * matchedLandmarks[i].point;



    pt_CRef(0) = pt_CRef(0) / pt_CRef(3);
    pt_CRef(1) = pt_CRef(1) / pt_CRef(3);
    pt_CRef(2) = pt_CRef(2) / pt_CRef(3);


    if (pt_CRef(2) > MIN_CAM_DISTANCE) {
      double x = ((pt_CRef(0) / pt_CRef(2)) * focalU) + centerU;
      double y = ((pt_CRef(1) / pt_CRef(2)) * focalV) + centerV;

      if (x <= 0 || y <= 0 || x >= dimU || y >= dimV) {
        continue;
      }

      filteredLandmarks.push_back(Eigen::Vector3f(pt_CRef(0), pt_CRef(1),
                                  pt_CRef(2)));
      landmarks2D.push_back(GEOM_FADE2D::Point2(x, y));
      landmarks2D.back().setCustomIndex(filteredLandmarks.size() - 1);
    }
  }
}


/**

*/
void writeCSV(std::string filename, cv::Mat m) {
  cv::Formatter const *c_formatter(cv::Formatter::get("CSV"));
  std::ofstream myfile;
  myfile.open(filename.c_str());
  c_formatter->write(myfile, m);
  myfile.close();
}

/**

*/
void saveObj(std::string filepath, VecPoint3F &points3d,
             std::vector<GEOM_FADE2D::Triangle2 *> triangles, std::vector<bool> &blacklist) {
  std::ofstream ofs;
  ofs.open(filepath, std::ofstream::out);


  for (unsigned int i = 0; i < points3d.size(); i++) {
    Eigen::Vector3f hPoint = points3d[i];
    ofs << "v "  << hPoint[0] << " " << hPoint[1] << " " << hPoint[2]  << std::endl;
  }

  for (unsigned int i = 0; i < triangles.size(); i++) {
    if (blacklist[i]) {
      continue;
    }

    GEOM_FADE2D::Triangle2 *t = triangles[i];
    ofs << "f "  << t->getCorner(2)->getCustomIndex() + 1 << " " << t->getCorner(
          1)->getCustomIndex() + 1 << " " << t->getCorner(0)->getCustomIndex() + 1  <<
        std::endl;
  }

  ofs.close();
}

/*
 *   Beginning of the code adapted from scratchapixel
 *   Copyright (C) 2012  www.scratchapixel.com - GPLv3
 */

float min3(const float &a, const float &b, const float &c) {
  return std::min(a, std::min(b, c));
}

float max3(const float &a, const float &b, const float &c) {
  return std::max(a, std::max(b, c));
}

void maxIndex3(const float &a, const float &b, const float &c, float &maxValue,
               int &maxIndex) {
  if (a > b) {
    if (a > c) {
      maxValue = a;
      maxIndex = 0;
    } else {
      maxValue = c;
      maxIndex = 2;
    }
  } else {
    if (b > c) {
      maxValue = b;
      maxIndex = 1;
    } else {
      maxValue = c;
      maxIndex = 2;
    }
  }
}

float edgeFunction(const cv::Point2f &a, const cv::Point2f &b,
                   const cv::Point2f &c) {
  return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

void rasterTriangle(const std::vector<cv::Point> &points,
                    const  std::vector<float> &zs,
                    const GEOM_FADE2D::Triangle2 *itri,
                    cv::Mat &resultMap) {

  cv::Point2f v0Raster = points[0];
  cv::Point2f v1Raster = points[1];
  cv::Point2f v2Raster = points[2];
  float x0 = min3(v0Raster.x, v1Raster.x, v2Raster.x);
  float y0 = min3(v0Raster.y, v1Raster.y, v2Raster.y);
  float x1 = max3(v0Raster.x, v1Raster.x, v2Raster.x);
  float y1 = max3(v0Raster.y, v1Raster.y, v2Raster.y);


  float area = edgeFunction(v0Raster, v1Raster, v2Raster);

  float iz0 = 1.0f / zs[0];
  float iz1 = 1.0f / zs[1];
  float iz2 = 1.0f / zs[2];

  for (uint32_t y = y0; y <= y1; ++y) {
    for (uint32_t x = x0; x <= x1; ++x) {
      cv::Point2f pixelSample(x + 0.5f, y + 0.5f);
      float w0 = edgeFunction(v1Raster, v2Raster, pixelSample);
      float w1 = edgeFunction(v2Raster, v0Raster, pixelSample);
      float w2 = edgeFunction(v0Raster, v1Raster, pixelSample);
      w0 /= area;
      w1 /= area;
      w2 /= area;

      if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
        float oneOverZ = iz0 * w0 + iz1 * w1 + iz2 * w2;
        float z = 1.0f / oneOverZ;
        resultMap.at<cv::Vec2f>(y, x)[0] = z;
        int index;
        float maxVal;
        maxIndex3(w0, w1, w2, maxVal, index);
        resultMap.at<cv::Vec2f>(y, x)[1] = itri->getCorner(index)->getCustomIndex();
      }
    }
  }
}

/*
 *   End of the code adapted from scratchapixel
 *   Copyright (C) 2012  www.scratchapixel.com - GPLv3
 */

/**

*/
//#define MM_DEBUG

void buildMeshDepthMap(const double focalU,
                       const double focalV,
                       const double centerU,
                       const double centerV,
                       const double dimU,
                       const double dimV,
                       const double minQuality,
                       const okvis::MapPointVector &matchedLandmarks,
                       okvis::kinematics::Transformation &T_WCRef,
                       cv::Mat &resultDepthMap,
                       const double laplaceAlpha = 0.1,
                       const unsigned int smoothingIteration = 3,
                       const double maxDelta = 0.2) {

  std::vector<GEOM_FADE2D::Point2> points2D;
  VecPoint3F points3D;

  projectLandmarks(focalU, focalV, centerU, centerV,
                   dimU, dimV, minQuality, T_WCRef,
                   matchedLandmarks, points3D, points2D);

  GEOM_FADE2D::Fade_2D dt;
  dt.insert(points2D);
  std::vector<GEOM_FADE2D::Triangle2 *> vAllTriangles;
  dt.getTrianglePointers(vAllTriangles);

  std::vector<bool> blacklist(vAllTriangles.size(), false);

#ifdef MM_DEBUG
  saveObj("meshProj_before.obj", points3D, vAllTriangles, blacklist);
#endif


  filteringAndSmoothing(points3D, vAllTriangles, blacklist, laplaceAlpha,
                        smoothingIteration, maxDelta);


#ifdef MM_DEBUG
  saveObj("meshProj_after.obj", points3D, vAllTriangles, blacklist);
#endif

  //clean the result map
  resultDepthMap.setTo(cv::Vec2f(0, 0));

  //write on the result map
  for (uint i = 0; i < vAllTriangles.size() ; i++) {
    if (blacklist[i]) {
      continue;
    }

    GEOM_FADE2D::Triangle2 *itri = vAllTriangles[i];
    std::vector<cv::Point> tpts(3);
    std::vector<float> zs(3);

    tpts[0]    = cv::Point(itri->getCorner(0)->x(), itri->getCorner(0)->y());
    zs[0]      = (points3D[itri->getCorner(0)->getCustomIndex()])(2);

    tpts[1]    = cv::Point(itri->getCorner(1)->x(), itri->getCorner(1)->y());
    zs[1]      = points3D[itri->getCorner(1)->getCustomIndex()](2);

    tpts[2]    = cv::Point(itri->getCorner(2)->x(), itri->getCorner(2)->y());
    zs[2]      = points3D[itri->getCorner(2)->getCustomIndex()](2);

    rasterTriangle(tpts, zs, itri, resultDepthMap);
  }

#ifdef MM_DEBUG

  for (int i = 0; i < 2; i++) {
    cv::Mat labels;
    cv::extractChannel(resultDepthMap, labels, i);
    writeCSV("meshProj_nn_" + std::to_string(i) + ".csv", labels);
  }

#endif

  return;
}




}


#endif
