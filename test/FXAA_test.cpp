#include <iostream>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>
#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/FXAA.hpp>

#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BOOST_TEST_MODULE "FXAA_test"
#include <boost/test/unit_test.hpp>

using namespace vizkit3d_normal_depth_map;

// osg::Image *cvImage2OsgImage(cv::Mat){
//
// }


cv::Mat3f plotSonarTest( cv::Mat3f image, double maxRange,
                    double maxAngleX, cv::Mat1f cv_depth) {

  cv::Mat3b imagePlotMap = cv::Mat3b::zeros(1000, 1000);
  cv::Mat1b imagePlot = cv::Mat1b::zeros(1000, 1000);
  cv::Point2f centerImage(image.cols / 2, image.rows / 2);
  cv::Point2f centerPlot(imagePlot.cols / 2, 0);
  double factor = 1000/maxRange;
  double pointSize = factor / 3;
  cv::Point2f halfSize(pointSize / 2, pointSize / 2);

  double slope = 2 * maxAngleX * (1.0 / (image.cols - 1));
  double constant = - maxAngleX;

  for (int j = 0; j < image.rows; ++j)
    for (int i = 0; i < image.cols; ++i) {
      // double distance = image[j][i][1] * maxRange;
      double distance = cv_depth[j][i] * maxRange;
      double alpha = slope * i + constant;

      cv::Point2f tempPoint(distance * sin(alpha), distance * cos(alpha));
      tempPoint = tempPoint * factor;
      tempPoint += centerPlot;
      // cv::circle(imagePlot, tempPoint, pointSize, cv::Scalar(255 *
      //  image[j][i][0]), -1);
      // cv::rectangle(imagePlot, tempPoint + halfSize, tempPoint
      //  - halfSize, cv::Scalar(255 * image[j][i][0]), -1);
      imagePlot[(uint) tempPoint.y][(uint) tempPoint.x] = 255 * image[j][i][0];
    }

  cv::Mat plotProcess;
  cv::applyColorMap(imagePlot, plotProcess, cv::COLORMAP_HOT);
  cv::applyColorMap(imagePlot, imagePlotMap, cv::COLORMAP_HOT);

  cv::line(imagePlotMap, centerPlot,
           cv::Point2f(maxRange * sin(maxAngleX) * factor,
                       maxRange * cos(maxAngleX) * factor) +  centerPlot,
           cv::Scalar(255), 1, CV_AA);

  cv::line(imagePlotMap, centerPlot,
           cv::Point2f(maxRange * sin(-maxAngleX) * factor,
                       maxRange * cos(maxAngleX) * factor) + centerPlot,
           cv::Scalar(255), 1, CV_AA);

  return imagePlotMap;
}


osg::ref_ptr<osg::Group> createSquare(float textureCoordMax = 1.0f) {
    // set up the Geometry.
    osg::Geometry* geom = new osg::Geometry;

    osg::Vec3Array* coords = new osg::Vec3Array(4);
    (*coords)[0].set(-1.0f, 1.0f, 1.0f);
    (*coords)[1].set(-1.0f, 1.0f, -1.0f);
    (*coords)[2].set(1.0f, 1.0f, -1.0f);
    (*coords)[3].set(1.0f, 1.0f, 1.0f);
    geom->setVertexArray(coords);

    osg::Vec3Array* norms = new osg::Vec3Array(1);
    (*norms)[0].set(0.0f, -1.0f, 0.0f);
    geom->setNormalArray(norms, osg::Array::BIND_OVERALL);

    osg::Vec2Array* tcoords = new osg::Vec2Array(4);
    (*tcoords)[0].set(0.0f, 0.0f);
    (*tcoords)[1].set(0.0f, textureCoordMax);
    (*tcoords)[2].set(textureCoordMax, textureCoordMax);
    (*tcoords)[3].set(textureCoordMax, 0.0f);
    geom->setTexCoordArray(0, tcoords);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(geom);

    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild(geode);

    return root;
}


void makeSimpleScene(osg::ref_ptr<osg::Group> root) {

  osg::Geode *sphere = new osg::Geode();
  sphere->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
  root->addChild(sphere);

  osg::Geode *cylinder = new osg::Geode();
  cylinder->addDrawable(
    new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(30, 0, 10), 10, 10)));
  root->addChild(cylinder);

  osg::Geode *cone = new osg::Geode();
  cylinder->addDrawable(
    new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 30, 0), 10, 10)));
  root->addChild(cone);

  osg::Geode *box = new osg::Geode();
  cylinder->addDrawable(
    new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, -30, -10), 10)));
  root->addChild(box);
}

void viewPointsFromScene(std::vector<osg::Vec3d> *eyes,
                          std::vector<osg::Vec3d> *centers,
                          std::vector<osg::Vec3d> *ups) {

  // view1 - near from the ball with the cylinder in back
  eyes->push_back(osg::Vec3d(-8.77105, -4.20531, -3.24954));
  centers->push_back(osg::Vec3d(-7.84659, -4.02528, -2.91345));
  ups->push_back(osg::Vec3d(-0.123867, -0.691871, 0.711317));

  // view2 - near from the ball with the cube in back
  eyes->push_back(osg::Vec3d(3.38523, 10.093, 1.12854));
  centers->push_back(osg::Vec3d(3.22816, 9.12808, 0.918259));
  ups->push_back(osg::Vec3d(-0.177264, -0.181915, 0.967204));

  // view3 - near the cone in up side
  eyes->push_back(osg::Vec3d(-10.6743, 38.3461, 26.2601));
  centers->push_back(osg::Vec3d(-10.3734, 38.086, 25.3426));
  ups->push_back(osg::Vec3d(0.370619, -0.854575, 0.36379));

  // view4 - Faced the cube plane
  eyes->push_back(osg::Vec3d(0.0176255, -56.5841, -10.0666));
  centers->push_back(osg::Vec3d(0.0176255, -55.5841, -10.0666));
  ups->push_back(osg::Vec3d(0, 0, 1));
}



// Main test
// Good example to follow
BOOST_AUTO_TEST_CASE(applyShaderNormalDepthMap_TestCase) {

  std::vector<cv::Scalar> gt_scalar;
  gt_scalar.push_back(cv::Scalar(1123.89, 198.498, 485.374, 0));
  gt_scalar.push_back(cv::Scalar(1243.45, 263.631, 342.008, 0));
  gt_scalar.push_back(cv::Scalar(1081.27, 170.659, 344.832, 0));
  gt_scalar.push_back(cv::Scalar(1120.92, 71.1569, 71.1569, 0));

  std::vector<osg::Vec3d> eyes, centers, ups;

  float maxRange = 50;
  float maxAngleX = M_PI * 1.0 / 6; // 30 degrees
  float maxAngleY = M_PI * 1.0 / 6; // 30 degrees

  uint height = 2000;
  NormalDepthMap normalDepthMap(maxRange, maxAngleX * 0.5, maxAngleY * 0.5);
  ImageViewerCaptureTool capture(maxAngleY, maxAngleX, height);

  capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

  osg::ref_ptr<osg::Group> root = new osg::Group();
  viewPointsFromScene(&eyes, &centers, &ups);
  makeSimpleScene(root);
  normalDepthMap.addNodeChild(root);


  // configure fxaa shader
  FXAA fxaa;
  osg::Vec3d fxaa_eye, fxaa_center, fxaa_up;
  fxaa.getCameraPosition(fxaa_eye, fxaa_center, fxaa_up);

  ImageViewerCaptureTool fxaa_capture(1000, 1000);
  fxaa_capture.setCameraPosition(fxaa_eye, fxaa_center, fxaa_up);

  for (uint i = 0; i < eyes.size(); ++i) {
    capture.setCameraPosition(eyes[i], centers[i], ups[i]);

    normalDepthMap.setDrawNormal(true);
    normalDepthMap.setDrawDepth(true);

    osg::ref_ptr<osg::Image> osgImage =
      capture.grabImage(normalDepthMap.getNormalDepthMapNode());
    cv::Mat3f cv_image(osgImage->t(), osgImage->s());
    cv_image.data = osgImage->data();
    cv_image = cv_image.clone();
    cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR, CV_32FC3);
    cv::flip(cv_image, cv_image, 0);

    // get depth buffer
    osg::ref_ptr<osg::Image> osg_depth =  capture.getDepthBuffer();
    cv::Mat1f cv_depth(osg_depth->t(), osg_depth->s());
    cv_depth.data = osg_depth->data();
    cv_depth = cv_depth.clone();
    cv::flip(cv_depth, cv_depth, 0);
    cv_depth = cv_depth.mul( cv::Mat1f(cv_depth < 1)/255);

    cv::Mat3b image_plot = plotSonarTest( cv_image, maxRange,
                                          maxAngleX/2.0, cv_depth);

    // cv::imshow("image Plot ORIGINAL", image_plot);


    osg::ref_ptr<osg::Image> osg_plot = new osg::Image;
    osg_plot->allocateImage(image_plot.cols, image_plot.rows, 3,
                            GL_BGR, GL_BYTE);

    osg_plot->setImage(image_plot.cols, image_plot.rows, 3, GL_RGB, GL_RGB,
                       GL_UNSIGNED_BYTE, image_plot.data,
                       osg::Image::NO_DELETE,1);

    fxaa.addImageToFxaa(osg_plot, osg_plot->t(), osg_plot->s());
    osg_plot = fxaa_capture.grabImage(fxaa.getFxaaShaderNode());

    cv::Mat3f cv_img_plot(osg_plot->t(), osg_plot->s());
    cv_img_plot.data = osg_plot->data();
    cv::flip(cv_img_plot, cv_img_plot, 0);
    // cv::imshow("image Plot FXAA", cv_img_plot);

    cv::Mat3f temp_plot;
    image_plot.convertTo(temp_plot, CV_32FC3, 1.0/255);

    cv::absdiff(temp_plot, cv_img_plot, temp_plot);
    // cv::imshow("image Plot DIFFER",temp_plot);

    // cv::waitKey();
    cv::Scalar scalar_sum;
    scalar_sum = cv::sum(temp_plot);
    // std::cout<<" OUT "<< scalar_sum << std::endl;
    for (int j=0; j < 4; j++) {
      BOOST_CHECK_CLOSE(scalar_sum[j], gt_scalar[i][j], 1.0);
    }
  }

}

// BOOST_AUTO_TEST_CASE(TempTest){
//
//
//   std::string image_path = "/home/trocoli/Pictures/baiana_system.jpg";
//   osg::ref_ptr<osg::Image> osg_image = osgDB::readImageFile(image_path);
//
//   FXAA fxaa;
//   fxaa.addImageToFxaa(osg_image, 1366, 768);
//
//   osgViewer::Viewer bumpViewer;
//   // bumpViewer.setSceneData(createSquare());
//   bumpViewer.setSceneData(fxaa.getFxaaShaderNode());
//   bumpViewer.setCameraManipulator(new osgGA::TrackballManipulator());
//
//   //bumpViewer.run();
//
//   osg::Vec3d eye, center, up;
//
//   while(!bumpViewer.done()){
//     bumpViewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);
//     std::cout<<" eye "<<
//       eye.x() <<","<< eye.y()<<","<< eye.z()<<std::endl;
//     std::cout<<" center "<<
//       center.x() <<","<< center.y()<<","<< center.z()<<std::endl;
//     std::cout<<" up "<<
//       up.x() <<","<< up.y()<<","<< up.z()<<std::endl;
//     bumpViewer.frame();
//   }
// }
