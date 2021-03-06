#ifndef vizkit3d_normal_depth_map_NormalDepthMapVisualization_H
#define vizkit3d_normal_depth_map_NormalDepthMapVisualization_H

#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/samples/RigidBodyState.hpp>

using namespace vizkit3d_normal_depth_map;

namespace vizkit3d {
class NormalDepthMapVisualization: public vizkit3d::Vizkit3DPlugin<base::samples::RigidBodyState>, boost::noncopyable {
    Q_OBJECT
    Q_PROPERTY(double maxRange READ getMaxRange WRITE setMaxRange)
    Q_PROPERTY(bool drawNormal READ getDrawNormal WRITE setDrawNormal)
    Q_PROPERTY(bool drawDepth READ getDrawDepth WRITE setDrawDepth)

public:
    NormalDepthMapVisualization();
    ~NormalDepthMapVisualization();

    void setMaxRange(float maxRange);
    float getMaxRange();

    void setDrawNormal(bool drawNormal);
    bool getDrawNormal();

    void setDrawDepth(bool drawDepth);
    bool getDrawDepth();

    void addNodeChild(osg::ref_ptr<osg::Node> node);

    Q_INVOKABLE
    void updateData(base::samples::RigidBodyState const &sample) {
        vizkit3d::Vizkit3DPlugin<base::samples::RigidBodyState>::updateData(sample);
    }

protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(base::samples::RigidBodyState const& plan);

private:
    NormalDepthMap _normalDepthMap;
    struct Data;
    Data* p;
};
}
#endif
