#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

// Finds ORB features in two images and matches them.
// Results are written into keypoints_1, keypoints_2 and good_matches.
// const Type & is a fast way to pass big data read-only, Type & allow you to modify the passed input 
void feature_extraction(const Mat &img_1, const Mat &img_2,
                        vector<KeyPoint> &keypoints_1,
                        vector<KeyPoint> &keypoints_2,
                        vector<DMatch> &good_matches) {
  Ptr<ORB> orb = ORB::create(); // creates an ORB tool
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

  //-- detect Oriented FAST points and compute their BRIEF descriptors
  Mat descriptors_1, descriptors_2;
  orb->detectAndCompute(img_1, noArray(), keypoints_1, descriptors_1);
  orb->detectAndCompute(img_2, noArray(), keypoints_2, descriptors_2);

  //-- match the descriptors using Hamming distance
  good_matches.clear();
  if (descriptors_1.empty() || descriptors_2.empty()) return;  // no points in an image: nothing to match

  //-- for each point, find its 2 closest candidates in the other image (in both directions)
  vector<vector<DMatch>> knn_12, knn_21;
  matcher->knnMatch(descriptors_1, descriptors_2, knn_12, 2);
  matcher->knnMatch(descriptors_2, descriptors_1, knn_21, 2);

  const float ratio = 0.75f;      // best must be clearly better than the second best
  const float max_dist = 30.0f;   // and good enough on its own

  for (const vector<DMatch> &candidates : knn_12) {
    if (candidates.size() < 2) continue;
    const DMatch &best = candidates[0];
    const DMatch &second = candidates[1];

    // 1. ratio test: skip ambiguous points (e.g. repeated shelves)
    if (best.distance >= ratio * second.distance) continue;

    // 2. fixed threshold: skip matches that are too different
    if (best.distance > max_dist) continue;

    // 3. cross-check: image 2's point must also choose this point as its best
    const vector<DMatch> &back = knn_21[best.trainIdx];
    if (back.empty() || back[0].trainIdx != best.queryIdx) continue;

    good_matches.push_back(best);
  }
}