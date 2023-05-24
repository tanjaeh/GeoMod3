
#include <iostream>

#include "scenario.h"
//#include "testtorus.h"
#include"LoftedSurfaces/gmploftedsurf.h"
#include"../../gmlib-master/modules/parametrics/curves/gmpbeziercurve.h"
#include"../../gmlib-master/modules/parametrics/curves/gmpcircle.h"
#include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfnormalsvisualizer.h"
#include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfderivativesvisualizer.h"


// hidmanager
#include "hidmanager/defaulthidmanager.h"

// gmlib
#include <scene/light/gmpointlight.h>
#include <scene/sceneobjects/gmpointlightg.h>
#include <scene/sceneobjects/gmpathtrack.h>
#include <scene/sceneobjects/gmpathtrackarrows.h>

// qt
#include <QQuickItem>


template <typename T>
inline
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
  out << v.size() << std::endl;
  for(uint i=0; i<v.size(); i++) out << " " << v[i];
  out << std::endl;
  return out;
}




void Scenario::initializeScenario() {
  // Insert a light
  GMlib::Point<GLfloat,3> init_light_pos( 2.0, 4.0, 10 );
  GMlib::PointLightG *light = new GMlib::PointLightG(  GMlib::GMcolor::white(), GMlib::GMcolor::white(),
                                                     GMlib::GMcolor::white(), init_light_pos );
  light->setAttenuation(0.8f, 0.002f, 0.0008f);
  this->scene()->insertLight( light, true );

  // Insert Sun
  this->scene()->insertSun();

  // Default camera parameters
  int init_viewport_size = 600;
  GMlib::Point<float,3>  init_cam_pos( 0.0f, 0.0f, 0.0f );
  GMlib::Vector<float,3> init_cam_dir( 0.0f, 1.0f, 0.0f );
  GMlib::Vector<float,3> init_cam_up(  1.0f, 0.0f, 0.0f );

  // Projection cam
  auto proj_rcpair = createRCPair("Projection");
  proj_rcpair.camera->set(init_cam_pos,init_cam_dir,init_cam_up);
  proj_rcpair.camera->setCuttingPlanes( 1.0f, 8000.0f );
  proj_rcpair.camera->rotateGlobal( GMlib::Angle(-45), GMlib::Vector<float,3>( 1.0f, 0.0f, 0.0f ) );
  proj_rcpair.camera->translateGlobal( GMlib::Vector<float,3>( 0.0f, -20.0f, 20.0f ) );
  scene()->insertCamera( proj_rcpair.camera.get() );
  proj_rcpair.renderer->reshape( GMlib::Vector<int,2>(init_viewport_size, init_viewport_size) );


  /***************************************************************************
   *                                                                         *
   * Standar example, including path track and path track arrows             *
   *                                                                         *
   ***************************************************************************/

  GMlib::Material mm(GMlib::GMmaterial::polishedBronze());
  mm.set(45.0);

bool bezierCurves = false;
bool circleCurves = false;
bool mix = true;
bool vClosedBezier = false;

bool V_closed = false;
bool blending = true;
bool showSubSurf = false;
bool showCurves = true;


// Lofted base curves
std::vector<GMlib::PCurve<float, 3>*> loftedBaseCurves(4);

if(bezierCurves){
    //  Lofted
      GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve1(5);
      controlPointsCurve1[0] = GMlib::Vector<float,3>(0, 0, 0);
      controlPointsCurve1[1] = GMlib::Vector<float,3>(3, 0, 1);
      controlPointsCurve1[2] = GMlib::Vector<float,3>(1, 0, 3);
      controlPointsCurve1[3] = GMlib::Vector<float,3>(3, 0, 5);
      controlPointsCurve1[4] = GMlib::Vector<float,3>(0, 0, 6);


      GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve2(5);
      controlPointsCurve2[0] = GMlib::Vector<float,3>(-1, 0, -4);
      controlPointsCurve2[1] = GMlib::Vector<float,3>(6, 0, 1);
      controlPointsCurve2[2] = GMlib::Vector<float,3>(5, 0, 3);
      controlPointsCurve2[3] = GMlib::Vector<float,3>(7, 0, 5);
      controlPointsCurve2[4] = GMlib::Vector<float,3>(3, 0, 6);


      GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve3(5);
      controlPointsCurve3[0] = GMlib::Vector<float,3>(-3, 0, 1);
      controlPointsCurve3[1] = GMlib::Vector<float,3>(5, -2, 2);
      controlPointsCurve3[2] = GMlib::Vector<float,3>(2, 0, 3);
      controlPointsCurve3[3] = GMlib::Vector<float,3>(4, 0, 4);
      controlPointsCurve3[4] = GMlib::Vector<float,3>(3, -2, 8);


      GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve4(5);
      controlPointsCurve4[0] = GMlib::Vector<float,3>(-2, 0, -2);
      controlPointsCurve4[1] = GMlib::Vector<float,3>(0, 0, -2);
      controlPointsCurve4[2] = GMlib::Vector<float,3>(2, 2, 2);
      controlPointsCurve4[3] = GMlib::Vector<float,3>(4, 2, 2);
      controlPointsCurve4[4] = GMlib::Vector<float,3>(4, -2, 4);


    //  Lofted test
      auto bezier1 = new GMlib::PBezierCurve<float>(controlPointsCurve1);
      bezier1->toggleDefaultVisualizer();
      bezier1->sample(60,0);


      auto bezier2 = new GMlib::PBezierCurve<float>(controlPointsCurve2);
      bezier2->translate(GMlib::Vector<float,3>(0.0f, 3.0f, 0.0f));
      bezier2->toggleDefaultVisualizer();
      bezier2->sample(60,0);


      auto bezier3 = new GMlib::PBezierCurve<float>(controlPointsCurve3);
      bezier3->translate(GMlib::Vector<float,3>(0.0f, 8.0f, 0.0f));
      bezier3->toggleDefaultVisualizer();
      bezier3->sample(60,0);


      auto bezier4 = new GMlib::PBezierCurve<float>(controlPointsCurve4);
      bezier4->translate(GMlib::Vector<float,3>(0.0f, 10.0f, 0.0f));
      bezier4->toggleDefaultVisualizer();
      bezier4->sample(60,0);


      if(showCurves){
          this->scene()->insert(bezier1);
          this->scene()->insert(bezier2);
          this->scene()->insert(bezier3);
          this->scene()->insert(bezier4);
      }


      loftedBaseCurves[0] = bezier1;
      loftedBaseCurves[1] = bezier2;
      loftedBaseCurves[2] = bezier3;
      loftedBaseCurves[3] = bezier4;
}


if(circleCurves){
    float radius = 5.2f;

    auto circle1 = new GMlib::PCircle<float>(radius);
    circle1->toggleDefaultVisualizer();
    circle1->sample(60,0);

    auto circle2 = new GMlib::PCircle<float>(radius/2);
    circle2->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 2.0f));
    circle2->setColor(GMlib::GMcolor::orange());
    circle2->toggleDefaultVisualizer();
    circle2->sample(60,0);

    auto circle3 = new GMlib::PCircle<float>(radius/3);
    circle3->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 6.0f));
    circle3->setColor(GMlib::GMcolor::yellow());
    circle3->toggleDefaultVisualizer();
    circle3->sample(60,0);

    auto circle4 = new GMlib::PCircle<float>(radius*1.5);
    circle4->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 15.0f));
    circle4->setColor(GMlib::GMcolor::green());
    circle4->toggleDefaultVisualizer();
    circle4->sample(60,0);

    if(showCurves){
        this->scene()->insert(circle1);
        this->scene()->insert(circle2);
        this->scene()->insert(circle3);
        this->scene()->insert(circle4);
    }





    loftedBaseCurves[0] = circle1;
    loftedBaseCurves[1] = circle2;
    loftedBaseCurves[2] = circle3;
    loftedBaseCurves[3] = circle4;
}


if(mix){
    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve2(5);
    controlPointsCurve2[0] = GMlib::Vector<float,3>(-1, -4, 0);
    controlPointsCurve2[1] = GMlib::Vector<float,3>(6, 1, 0);
    controlPointsCurve2[2] = GMlib::Vector<float,3>(5, 3, 0);
    controlPointsCurve2[3] = GMlib::Vector<float,3>(7, 5, 0);
    controlPointsCurve2[4] = GMlib::Vector<float,3>(3, 6, 0);

    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve3(5);
    controlPointsCurve3[0] = GMlib::Vector<float,3>(-3, 1, 0);
    controlPointsCurve3[1] = GMlib::Vector<float,3>(5, 2, 0);
    controlPointsCurve3[2] = GMlib::Vector<float,3>(2, 3, 0);
    controlPointsCurve3[3] = GMlib::Vector<float,3>(4, 4, 0);
    controlPointsCurve3[4] = GMlib::Vector<float,3>(3, 8, 0);


    float radius = 2.3f;
    auto circle1 = new GMlib::PCircle<float>(radius);
    circle1->rotate(M_PI, GMlib::Vector<float, 3>(0.0f, 0.0f, 1.0f));
    circle1->toggleDefaultVisualizer();
    circle1->sample(60,0);

    auto bezier2 = new GMlib::PBezierCurve<float>(controlPointsCurve2);
    bezier2->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 3.0f));
    bezier2->toggleDefaultVisualizer();
    bezier2->sample(60,0);

    auto bezier3 = new GMlib::PBezierCurve<float>(controlPointsCurve3);
    bezier3->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 6.0f));
    bezier3->toggleDefaultVisualizer();
    bezier3->sample(60,0);

    auto circle4 = new GMlib::PCircle<float>(radius*1.2f);
    circle4->rotate(M_PI*0.8, GMlib::Vector<float, 3>(0.0f, 0.0f, 1.0f));
    circle4->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 15.0f));
    circle4->toggleDefaultVisualizer();
    circle4->sample(60,0);


    if(showCurves){
        this->scene()->insert(circle1);
        this->scene()->insert(bezier2);
        this->scene()->insert(bezier3);
        this->scene()->insert(circle4);
    }


    loftedBaseCurves[0] = circle1;
    loftedBaseCurves[1] = bezier2;
    loftedBaseCurves[2] = bezier3;
    loftedBaseCurves[3] = circle4;


}


if(vClosedBezier){
    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve1(3);
    controlPointsCurve1[0] = GMlib::Vector<float,3>(0.0f, 0.0f, 0.0f);
    controlPointsCurve1[1] = GMlib::Vector<float,3>(1.0f, 0.0f, 0.0f);
    controlPointsCurve1[2] = GMlib::Vector<float,3>(0.0f, 0.0f, 1.0f);


    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve2(3);
    controlPointsCurve2[0] = GMlib::Vector<float,3>(0.0f, 1.0f, 0.0f);
    controlPointsCurve2[1] = GMlib::Vector<float,3>(0.5f, 2.0f, 0.0f);
    controlPointsCurve2[2] = GMlib::Vector<float,3>(1.0f, 1.0f, 1.0f);


    //  Lofted test
      auto closedBezier1 = new GMlib::PBezierCurve<float>(controlPointsCurve1);
      closedBezier1->toggleDefaultVisualizer();
      closedBezier1->sample(60,0);

      auto closedBezier2 = new GMlib::PBezierCurve<float>(controlPointsCurve2);
//      bezier2->rotate(M_PI*0.5f, GMlib::Vector<float,3>(0.0f, 0.0f, 1.0f));
      closedBezier2->translate(GMlib::Vector<float,3>(-1.0f, 0.0f, 0.0f));
      closedBezier2->toggleDefaultVisualizer();
      closedBezier2->sample(60,0);

      auto closedBezier3 = new GMlib::PBezierCurve<float>(controlPointsCurve1);
      closedBezier3->rotate(M_PI, GMlib::Vector<float,3>(0.0f, 0.0f, 1.0f));
      closedBezier3->translate(GMlib::Vector<float,3>(3.0f, 0.0f, 0.0f));
      closedBezier3->toggleDefaultVisualizer();
      closedBezier3->sample(60,0);

      auto closedBezier4 = new GMlib::PBezierCurve<float>(controlPointsCurve2);
      closedBezier4->rotate(M_PI, GMlib::Vector<float,3>(0.0f, 0.0f, 1.0f));
      closedBezier4->translate(GMlib::Vector<float,3>(1.5f, 0.0f, 0.0f));
      closedBezier4->toggleDefaultVisualizer();
      closedBezier4->sample(60,0);


      if(showCurves){
          this->scene()->insert(closedBezier1);
          this->scene()->insert(closedBezier2);
          this->scene()->insert(closedBezier3);
          this->scene()->insert(closedBezier4);
      }


      loftedBaseCurves[0] = closedBezier1;
      loftedBaseCurves[1] = closedBezier2;
      loftedBaseCurves[2] = closedBezier3;
      loftedBaseCurves[3] = closedBezier4;
}

auto n_viz = new GMlib::PSurfNormalsVisualizer<float,3>();
auto d_viz = new GMlib::PSurfDerivativesVisualizer<float,3>(0,1);

int sample = 60;
auto loftedsurf = new PLoftedSurf<float>(loftedBaseCurves, sample, V_closed, blending);
loftedsurf->toggleDefaultVisualizer();
//loftedsurf->insertVisualizer(n_viz);
loftedsurf->sample(sample,sample,1,1);
loftedsurf->showControlSubSurf();
this->scene()->insert(loftedsurf);

std::vector<GMlib::Material> rainbow;
rainbow.push_back(GMlib::GMmaterial::ruby());
rainbow.push_back(GMlib::GMmaterial::bronze());
rainbow.push_back(GMlib::GMmaterial::gold());
rainbow.push_back(GMlib::GMmaterial::emerald());
rainbow.push_back(GMlib::GMmaterial::sapphire());
rainbow.push_back(GMlib::GMmaterial::obsidian());
rainbow.push_back(GMlib::GMmaterial::pearl());

//        (GMlib::GMcolor::red(), GMlib::GMcolor::orange(), GMlib::GMcolor::yellow(), GMlib::GMcolor::green(), GMlib::GMcolor::blue(), GMlib::GMcolor::purple(), GMlib::GMcolor::white(), GMlib::GMcolor::black());

if(showSubSurf){
    for(int i = 0; i < loftedsurf->_subS.size(); i++){
        loftedsurf->_subS[i]->toggleDefaultVisualizer();
//        loftedsurf->_subS[i]->insertVisualizer(n_viz);
        loftedsurf->_subS[i]->insertVisualizer(d_viz);
        loftedsurf->_subS[i]->setMaterial(rainbow[i]);
        loftedsurf->_subS[i]->sample(sample,sample,1,1);
        loftedsurf->_subS[i]->showSelectors();
        this->scene()->insert(loftedsurf->_subS[i]);
    }
}



}




void Scenario::cleanupScenario() {

}




void Scenario::callDefferedGL() {

  GMlib::Array< const GMlib::SceneObject*> e_obj;
  this->scene()->getEditedObjects(e_obj);

  for(int i=0; i < e_obj.getSize(); i++)
    if(e_obj(i)->isVisible()) e_obj[i]->replot();
}

