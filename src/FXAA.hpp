#include <osg/Group>
#include <osg/Geode>
#include <osg/Image>
#include <osg/ref_ptr>

namespace vizkit3d_normal_depth_map {

/**
 * @brief
 *
 */
class FXAA {
public:
  FXAA();
  FXAA(float fxaa_span_max, float fxaa_reduce_mul, float fxaa_reduce_min);

  // variables to adjust FXAA quality
  float getFxaaSpanMax();
  void setFxaaSpanMax( float fxaa_span_max);

  float getFxaaReduceMul();
  void setFxaaReduceMul( float fxaa_reduce_mul);

  float getFxaaReduceMin();
  void setFxaaReduceMin( float fxaa_reduce_min);

  void setFinalImageResolution(float width, float height);

  void addImageToFxaa( osg::ref_ptr<osg::Image> image, float final_width,
                       float final_height);

  const osg::ref_ptr<osg::Group> getFxaaShaderNode() const {
    return _fxaaShaderNode;
  }


private:
  osg::ref_ptr<osg::Group> createFXAAShaderNode(
                                          float fxaa_span_max = 8.0,
                                          float fxaa_reduce_mul = 1.0/8.0,
                                          float fxaa_reduce_min = 1.0/128.0);

  osg::ref_ptr<osg::Geode> insertTextureInGoede( osg::Image *image );



  osg::ref_ptr<osg::Group> _fxaaShaderNode;
};
}
