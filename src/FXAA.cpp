#include "FXAA.hpp"

#include <osg/Node>
#include <osg/Program>
#include <osg/ref_ptr>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/Texture>
#include <osg/Texture2D>
#include <osg/ShapeDrawable>

#include <osgDB/FileUtils>

namespace vizkit3d_normal_depth_map {

#define PATH_SHADER_FXAA_VERT "vizkit3d_normal_depth_map/shaders/fxaa.vert"
#define PATH_SHADER_FXAA_FRAG "vizkit3d_normal_depth_map/shaders/fxaa.frag"

#define IMAGE_TEXTTURE_UNIT 0


osg::Geometry *createFlatGeometry(float textureCoordMax = 1.0f) {
    // set up the Geometry.
    osg::Geometry* geom = new osg::Geometry;

    osg::Vec3Array* coords = new osg::Vec3Array(4);
    (*coords)[0].set(-1.0f, 0.0f, 1.0f);
    (*coords)[1].set(-1.0f, 0.0f, -1.0f);
    (*coords)[2].set(1.0f, 0.0f, -1.0f);
    (*coords)[3].set(1.0f, 0.0f, 1.0f);
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

    return geom;
}


FXAA::FXAA(){
  _fxaaShaderNode = createFXAAShaderNode();
}

FXAA::FXAA(float fxaa_span_max, float fxaa_reduce_mul, float fxaa_reduce_min){
  _fxaaShaderNode = createFXAAShaderNode( fxaa_span_max, fxaa_reduce_mul,
                                          fxaa_reduce_min);
};

float FXAA::getFxaaSpanMax(){
  return 0;
}

void FXAA::setFxaaSpanMax( float fxaa_span_max){
  _fxaaShaderNode->getOrCreateStateSet()->
    getUniform("fxaa_span_max")->set(fxaa_span_max);
}

float FXAA::getFxaaReduceMul(){
  return 0;
}
void FXAA::setFxaaReduceMul( float fxaa_reduce_mul){
  _fxaaShaderNode->getOrCreateStateSet()->
    getUniform("fxaa_reduce_mul")->set(fxaa_reduce_mul);
}

float FXAA::getFxaaReduceMin(){
  return 0;
}

void FXAA::setFxaaReduceMin( float fxaa_reduce_min){
  _fxaaShaderNode->getOrCreateStateSet()->
    getUniform("fxaa_reduce_min")->set(fxaa_reduce_min);
}

void FXAA::setFinalImageResolution(float width, float height){
  _fxaaShaderNode->getOrCreateStateSet()->
    getUniform("image_resolution")->set(osg::Vec2(width, height));
}

void FXAA::addImageToFxaa( osg::ref_ptr<osg::Image> image, float final_width,
                           float final_height){

  osg::ref_ptr<osg::Geode> geode_tex = insertTextureInGoede(image);

  if( _fxaaShaderNode->getNumChildren())
    _fxaaShaderNode->setChild(0,geode_tex);
  else
    _fxaaShaderNode->addChild(geode_tex);

    setFinalImageResolution(final_width,final_height);
};


osg::ref_ptr<osg::Group> FXAA::createFXAAShaderNode(  float fxaa_span_max,
                                                      float fxaa_reduce_mul,
                                                      float fxaa_reduce_min) {

  osg::ref_ptr<osg::Group> local_root = new osg::Group();
  osg::ref_ptr<osg::Program> program(new osg::Program());

  osg::ref_ptr<osg::Shader> shader_vertex =
    osg::Shader::readShaderFile(osg::Shader::VERTEX,
                                osgDB::findDataFile(PATH_SHADER_FXAA_VERT));

  osg::ref_ptr<osg::Shader> shader_fragment =
    osg::Shader::readShaderFile(osg::Shader::FRAGMENT,
                                osgDB::findDataFile(PATH_SHADER_FXAA_FRAG));

  program->addShader(shader_vertex);
  program->addShader(shader_fragment);

  osg::ref_ptr<osg::StateSet> ss = local_root->getOrCreateStateSet();
  ss->addUniform( new osg::Uniform("original_image", IMAGE_TEXTTURE_UNIT));
  ss->addUniform( new osg::Uniform ("image_resolution", osg::Vec2(1,1)));
  ss->setAttribute(program);



  // osg::ref_ptr<osg::Uniform> fxaa_span_max_uniform(
  //                         new osg::Uniform("fxaa_span_max", fxaa_span_max));
  // ss->addUniform(fxaa_span_max_uniform);
  //
  // osg::ref_ptr<osg::Uniform> fxaa_reduce_mul_uniform(
  //                         new osg::Uniform("fxaa_reduce_mul", fxaa_reduce_mul));
  // ss->addUniform(fxaa_reduce_mul_uniform);
  //
  // osg::ref_ptr<osg::Uniform> fxaa_reduce_min_uniform(
  //                         new osg::Uniform("fxaa_reduce_min", fxaa_reduce_min));
  // ss->addUniform(fxaa_reduce_min_uniform);


  return local_root;
}




osg::ref_ptr<osg::Geode> FXAA::insertTextureInGoede( osg::Image *image ){

  osg::ref_ptr<osg::Texture2D> image_tex = new osg::Texture2D();

  image_tex->setImage(image);
  image_tex->setDataVariance(osg::Object::DYNAMIC);
  image_tex->setFilter( osg::Texture::MIN_FILTER,
                        osg::Texture::LINEAR_MIPMAP_LINEAR);
  image_tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
  image_tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
  image_tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
  image_tex->setResizeNonPowerOfTwoHint(false);
  image_tex->setMaxAnisotropy(8.0f);

  osg::ref_ptr<osg::StateSet> state_tex = new osg::StateSet();
  state_tex->setTextureAttributeAndModes( IMAGE_TEXTTURE_UNIT, image_tex,
                                          osg::StateAttribute::ON);

  osg::ref_ptr<osg::Geode> geode_tex = new osg::Geode;
  geode_tex->setStateSet(state_tex);

  osg::Geometry *text_geometry = createFlatGeometry();

  geode_tex->addDrawable(text_geometry);

  return geode_tex;
}

} // namespace vizkit3d_normal_depth_map
