
#include <iostream>

#include "scenario.h"
//#include "testtorus.h"
#include"LoftedSurfaces/gmploftedsurf.h"
#include"LoftedSurfaces/astroidCurve.h"
#include"../../gmlib-master/modules/parametrics/curves/gmpbeziercurve.h"
#include"../../gmlib-master/modules/parametrics/curves/gmpcircle.h"
#include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfnormalsvisualizer.h"
#include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfderivativesvisualizer.h"
#include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfnormalsvisualizer.h"
#include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfdefaultvisualizer.h"
#include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfcontoursvisualizer.h"
#include "Visualizer/zebra_visualizer.h"
#include "Visualizer/Curvature_analysis_visualizer.h"
#include "Visualizer/test_visualizer.h"
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
  proj_rcpair.camera->translateGlobal( GMlib::Vector<float,3>( 0.0f, -50.0f, 50.0f ) );
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
bool mix = false;
bool vClosedBezier = true;
bool longsurf = false;
bool astroid_cir = false;
bool closed_closed = false;
bool complex_surf = false;

bool V_closed = true;
bool blending = true;
bool showSubSurf = false;
bool showCurves = false;
bool affine_trans = false;
bool extra_surf = false;
bool default_vizu = false;


// Insert a light
GMlib::Point<GLfloat,3> init_light_pos( 2.0, -30.0, 10 );
if(bezierCurves){
    init_light_pos = GMlib::Point<GLfloat,3>(2.0, 0.0, -25.0);
}
if(circleCurves){
    init_light_pos = GMlib::Point<GLfloat,3>(2.0, -25.0, 0.0);
}
if(mix || longsurf || astroid_cir){
    init_light_pos = GMlib::Point<GLfloat,3>(-10.0, 0.0, 0.0);
}
if(complex_surf){
    init_light_pos = GMlib::Point<GLfloat,3>(-10.0, 0.0, 0.0);
}


GMlib::PointLightG *light = new GMlib::PointLightG(  GMlib::GMcolor::white(), GMlib::GMcolor::white(),
                                                   GMlib::GMcolor::white(), init_light_pos );
light->setAttenuation(0.8f, 0.002f, 0.0008f);
this->scene()->insertLight( light, false ); //true, the light can be seen as an object, false it is "hidden"


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
//    circle2->setColor(GMlib::GMcolor::orange());
    circle2->toggleDefaultVisualizer();
    circle2->sample(60,0);

    auto circle3 = new GMlib::PCircle<float>(radius/3);
    circle3->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 6.0f));
//    circle3->setColor(GMlib::GMcolor::yellow());
    circle3->toggleDefaultVisualizer();
    circle3->sample(60,0);

    auto circle4 = new GMlib::PCircle<float>(radius*1.5);
    circle4->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 15.0f));
//    circle4->setColor(GMlib::GMcolor::green());
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
      closedBezier2->translate(GMlib::Vector<float,3>(-2.0f, 2.0f, 0.0f));
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


if(longsurf){
    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve1(5);
    controlPointsCurve1[0] = GMlib::Vector<float,3>(-3, -1, 0);
    controlPointsCurve1[1] = GMlib::Vector<float,3>(2, 2, 0);
    controlPointsCurve1[2] = GMlib::Vector<float,3>(7, 7, 0);
    controlPointsCurve1[3] = GMlib::Vector<float,3>(6, 8, 0);
    controlPointsCurve1[4] = GMlib::Vector<float,3>(-1, 10, 0);


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

    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve4(5);
    controlPointsCurve4[0] = GMlib::Vector<float,3>(-5, -1, 0);
    controlPointsCurve4[1] = GMlib::Vector<float,3>(-2, 0, 0);
    controlPointsCurve4[2] = GMlib::Vector<float,3>(3, 4, 0);
    controlPointsCurve4[3] = GMlib::Vector<float,3>(-4, 5, 0);
    controlPointsCurve4[4] = GMlib::Vector<float,3>(1, 7, 0);

    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve5(5);
    controlPointsCurve5[0] = GMlib::Vector<float,3>(-2, -2, 0);
    controlPointsCurve5[1] = GMlib::Vector<float,3>(4, 0, 0);
    controlPointsCurve5[2] = GMlib::Vector<float,3>(3, 1, 0);
    controlPointsCurve5[3] = GMlib::Vector<float,3>(7, 4, 0);
    controlPointsCurve5[4] = GMlib::Vector<float,3>(1, 5, 0);

    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve6(5);
    controlPointsCurve6[0] = GMlib::Vector<float,3>(3, 1, 0);
    controlPointsCurve6[1] = GMlib::Vector<float,3>(-4, -3, 0);
    controlPointsCurve6[2] = GMlib::Vector<float,3>(-8, -6, 0);
    controlPointsCurve6[3] = GMlib::Vector<float,3>(-5, -7, 0);
    controlPointsCurve6[4] = GMlib::Vector<float,3>(1, -10, 0);

    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve7(5);
    controlPointsCurve7[0] = GMlib::Vector<float,3>(-5, -1, 0);
    controlPointsCurve7[1] = GMlib::Vector<float,3>(-2, 0, 0);
    controlPointsCurve7[2] = GMlib::Vector<float,3>(3, 4, 0);
    controlPointsCurve7[3] = GMlib::Vector<float,3>(-4, 5, 0);
    controlPointsCurve7[4] = GMlib::Vector<float,3>(1, 7, 0);

    float radius = 2.3f;


    auto bezier1 = new GMlib::PBezierCurve<float>(controlPointsCurve1);
    bezier1->translate(GMlib::Vector<float,3>(0.0f, 0.0f, -5.0f));
    bezier1->toggleDefaultVisualizer();
    bezier1->sample(60,0);

    auto circle1 = new GMlib::PCircle<float>(radius);
    circle1->rotate(M_PI, GMlib::Vector<float, 3>(0.0f, 0.0f, 1.0f));
    circle1->toggleDefaultVisualizer();
    circle1->sample(60,0);

    auto bezier2 = new GMlib::PBezierCurve<float>(controlPointsCurve2);
    bezier2->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 3.0f));
    bezier2->toggleDefaultVisualizer();
    bezier2->sample(60,0);

    auto bezier3 = new GMlib::PBezierCurve<float>(controlPointsCurve3);
    bezier3->rotate(M_PI*0.2, GMlib::Vector<float, 3>(0.0f, 0.0f, 1.0f));
    bezier3->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 6.0f));
    bezier3->toggleDefaultVisualizer();
    bezier3->sample(60,0);

    auto circle4 = new GMlib::PCircle<float>(radius*1.2f);
    circle4->rotate(M_PI*0.8, GMlib::Vector<float, 3>(0.0f, 0.0f, 1.0f));
    circle4->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 15.0f));
    circle4->toggleDefaultVisualizer();
    circle4->sample(60,0);

    auto bezier4 = new GMlib::PBezierCurve<float>(controlPointsCurve4);
    bezier4->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 17.0f));
    bezier4->toggleDefaultVisualizer();
    bezier4->sample(60,0);

    auto bezier5 = new GMlib::PBezierCurve<float>(controlPointsCurve5);
    bezier5->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 23.0f));
    bezier5->toggleDefaultVisualizer();
    bezier5->sample(60,0);

    auto bezier6 = new GMlib::PBezierCurve<float>(controlPointsCurve6);
    bezier6->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 28.0f));
    bezier6->toggleDefaultVisualizer();
    bezier6->sample(60,0);




    if(showCurves){
        this->scene()->insert(bezier1);
        this->scene()->insert(circle1);
        this->scene()->insert(bezier2);
        this->scene()->insert(bezier3);
        this->scene()->insert(circle4);
        this->scene()->insert(bezier4);
        this->scene()->insert(bezier5);
    }

    loftedBaseCurves[0] = bezier1;
    loftedBaseCurves[1] = circle1;
    loftedBaseCurves[2] = bezier2;
    loftedBaseCurves[3] = bezier3;
    loftedBaseCurves.push_back(circle4);
    loftedBaseCurves.push_back(bezier4);
    loftedBaseCurves.push_back(bezier5);


}

if(astroid_cir){
    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve2(5);
    controlPointsCurve2[0] = GMlib::Vector<float,3>(-1, -4, 0);
    controlPointsCurve2[1] = GMlib::Vector<float,3>(6, 1, 0);
    controlPointsCurve2[2] = GMlib::Vector<float,3>(5, 3, 0);
    controlPointsCurve2[3] = GMlib::Vector<float,3>(7, 5, 0);
    controlPointsCurve2[4] = GMlib::Vector<float,3>(3, 6, 0);


    float radius = 2.3f;
    auto circle1 = new GMlib::PCircle<float>(radius);
    circle1->rotate(M_PI, GMlib::Vector<float, 3>(0.0f, 0.0f, 1.0f));
    circle1->toggleDefaultVisualizer();
    circle1->sample(60,0);


    auto circle2 = new GMlib::PCircle<float>(radius);
    circle2->rotate(M_PI, GMlib::Vector<float, 3>(0.0f, 0.0f, 1.0f));
    circle2->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 5.0f));
    circle2->toggleDefaultVisualizer();
    circle2->sample(60,0);



    auto astroid = new AstroidCurve<float>(4.0f, 4.0f);
    astroid->rotate(M_PI * -0.5, GMlib::Vector<float, 3>(1.0f, 0.0f, 0.0f));
    astroid->rotate(M_PI * 0.75, GMlib::Vector<float, 3>(0.0f, 1.0f, 0.0f));
    astroid->translate(GMlib::Vector<float,3>(0.0f, -10.0f, 0.0f));
    astroid->toggleDefaultVisualizer();
    astroid->sample(60,60);

    auto circle4 = new GMlib::PCircle<float>(radius*1.2f);
    circle4->rotate(M_PI*1.3, GMlib::Vector<float, 3>(0.0f, 0.0f, 1.0f));
    circle4->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 15.0f));
    circle4->toggleDefaultVisualizer();
    circle4->sample(60,0);


    if(showCurves){
        this->scene()->insert(circle1);
        this->scene()->insert(circle2);
        this->scene()->insert(astroid);
        this->scene()->insert(circle4);
    }


    loftedBaseCurves[0] = circle1;
    loftedBaseCurves[1] = circle2;
    loftedBaseCurves[2] = astroid;
    loftedBaseCurves[3] = circle4;


}


if(closed_closed){
    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve2(5);
    controlPointsCurve2[0] = GMlib::Vector<float,3>(-1, -4, 0);
    controlPointsCurve2[1] = GMlib::Vector<float,3>(6, 1, 0);
    controlPointsCurve2[2] = GMlib::Vector<float,3>(5, 3, 0);
    controlPointsCurve2[3] = GMlib::Vector<float,3>(7, 5, 0);
    controlPointsCurve2[4] = GMlib::Vector<float,3>(3, 6, 0);


    float radius = 2.3f;
    auto circle1 = new GMlib::PCircle<float>(radius);
    circle1->rotate(M_PI, GMlib::Vector<float, 3>(0.0f, 1.0f, 0.0f));
    circle1->translate(GMlib::Vector<float,3>(10.0f, 0.0f, 0.0f));
    circle1->toggleDefaultVisualizer();
    circle1->sample(60,0);


    auto circle2 = new GMlib::PCircle<float>(radius);
    circle2->rotate(M_PI*-0.5, GMlib::Vector<float, 3>(0.0f, 1.0f, 0.0f));
    circle2->translate(GMlib::Vector<float,3>(10.0f, 0.0f, 5.0f));
    circle2->toggleDefaultVisualizer();
    circle2->sample(60,0);



    auto astroid = new AstroidCurve<float>(4.0f, 4.0f);
    astroid->rotate(M_PI * -0.5, GMlib::Vector<float, 3>(1.0f, 0.0f, 0.0f));
    // astroid->rotate(M_PI * 0.75, GMlib::Vector<float, 3>(0.0f, 1.0f, 0.0f));
    // astroid->translate(GMlib::Vector<float,3>(0.0f, -10.0f, 0.0f));
    astroid->toggleDefaultVisualizer();
    astroid->sample(60,60);

    auto circle4 = new GMlib::PCircle<float>(radius*1.2f);
    circle4->rotate(M_PI*0.5, GMlib::Vector<float, 3>(0.0f, 1.0f, 0.0f));
    circle4->translate(GMlib::Vector<float,3>(10.0f, 0.0f, -5.0f));
    circle4->toggleDefaultVisualizer();
    circle4->sample(60,0);


    if(showCurves){
        this->scene()->insert(circle1);
        this->scene()->insert(circle2);
        this->scene()->insert(astroid);
        this->scene()->insert(circle4);
    }


    loftedBaseCurves[0] = circle1;
    loftedBaseCurves[1] = circle2;
    loftedBaseCurves[2] = astroid;
    loftedBaseCurves[3] = circle4;


}


if(complex_surf){
    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve1(15);
    controlPointsCurve1[0] = GMlib::Vector<float,3>(3, 2, 0);
    controlPointsCurve1[1] = GMlib::Vector<float,3>(2, 4, 0);
    controlPointsCurve1[2] = GMlib::Vector<float,3>(3, 5, 0);
    controlPointsCurve1[3] = GMlib::Vector<float,3>(4.5, 3, 0);
    controlPointsCurve1[4] = GMlib::Vector<float,3>(5.5, 2, 0);
    controlPointsCurve1[5] = GMlib::Vector<float,3>(6.5, 3, 0);
    controlPointsCurve1[6] = GMlib::Vector<float,3>(6, 4.5, 0);
    controlPointsCurve1[7] = GMlib::Vector<float,3>(5, 7, 0);
    controlPointsCurve1[8] = GMlib::Vector<float,3>(6.5, 7, 0);
    controlPointsCurve1[9] = GMlib::Vector<float,3>(7, 5, 0);
    controlPointsCurve1[10] = GMlib::Vector<float,3>(9, 5.5, 0);
    controlPointsCurve1[11] = GMlib::Vector<float,3>(8, 9, 0);
    controlPointsCurve1[12] = GMlib::Vector<float,3>(9.5, 9.5, 0);
    controlPointsCurve1[13] = GMlib::Vector<float,3>(11, 5, 0);
    controlPointsCurve1[14] = GMlib::Vector<float,3>(12, 7.5, 0);

    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve2(15);
    controlPointsCurve2[0] = GMlib::Vector<float,3>(3, 3.5, 2);
    controlPointsCurve2[1] = GMlib::Vector<float,3>(4.5, 3, 2 );
    controlPointsCurve2[2] = GMlib::Vector<float,3>(5.5, 2, 2);
    controlPointsCurve2[3] = GMlib::Vector<float,3>(6.5, 3, 2);
    controlPointsCurve2[4] = GMlib::Vector<float,3>(6, 4.5, 2);
    controlPointsCurve2[5] = GMlib::Vector<float,3>(5, 7, 2);
    controlPointsCurve2[6] = GMlib::Vector<float,3>(6.5, 7, 2);
    controlPointsCurve2[7] = GMlib::Vector<float,3>(7, 5, 2);
    controlPointsCurve2[8] = GMlib::Vector<float,3>(9, 5.5, 2);
    controlPointsCurve2[9] = GMlib::Vector<float,3>(8, 9, 2);
    controlPointsCurve2[10] = GMlib::Vector<float,3>(9.5, 9.5, 2);
    controlPointsCurve2[11] = GMlib::Vector<float,3>(11, 5, 2);
    controlPointsCurve2[12] = GMlib::Vector<float,3>(12.5, 6, 2);
    controlPointsCurve2[13] = GMlib::Vector<float,3>(13, 7.5, 2);
    controlPointsCurve2[14] = GMlib::Vector<float,3>(12, 9.5, 2);


    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve3(16);
    controlPointsCurve3[0] = GMlib::Vector<float,3>(4, 3.5, 5);
    controlPointsCurve3[1] = GMlib::Vector<float,3>(5.5, 2, 5);
    controlPointsCurve3[2] = GMlib::Vector<float,3>(6.5, 3, 5);
    controlPointsCurve3[3] = GMlib::Vector<float,3>(6, 4.5, 5);
    controlPointsCurve3[4] = GMlib::Vector<float,3>(5, 7, 5);
    controlPointsCurve3[5] = GMlib::Vector<float,3>(6.5, 7, 5);
    controlPointsCurve3[6] = GMlib::Vector<float,3>(7, 5, 5);
    controlPointsCurve3[7] = GMlib::Vector<float,3>(9, 5.5, 5);
    controlPointsCurve3[8] = GMlib::Vector<float,3>(8, 9, 5);
    controlPointsCurve3[9] = GMlib::Vector<float,3>(9.5, 9.5, 5);
    controlPointsCurve3[10] = GMlib::Vector<float,3>(11, 5, 5);
    controlPointsCurve3[11] = GMlib::Vector<float,3>(12.5, 6, 5);
    controlPointsCurve3[12] = GMlib::Vector<float,3>(13, 7.5, 5);
    controlPointsCurve3[13] = GMlib::Vector<float,3>(12, 13, 5);
    controlPointsCurve3[14] = GMlib::Vector<float,3>(13, 13, 5);
    controlPointsCurve3[15] = GMlib::Vector<float,3>(13.5, 9.5, 5);


    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve4(17);
    controlPointsCurve4[0] = GMlib::Vector<float,3>(5, 3.5, 7);
    controlPointsCurve4[1] = GMlib::Vector<float,3>(6, 4.5, 7);
    controlPointsCurve4[2] = GMlib::Vector<float,3>(5, 7, 7);
    controlPointsCurve4[3] = GMlib::Vector<float,3>(6.5, 7, 7);
    controlPointsCurve4[4] = GMlib::Vector<float,3>(7, 5, 7);
    controlPointsCurve4[5] = GMlib::Vector<float,3>(9, 5.5, 7);
    controlPointsCurve4[6] = GMlib::Vector<float,3>(8, 9, 7);
    controlPointsCurve4[7] = GMlib::Vector<float,3>(9.5, 9.5, 7);
    controlPointsCurve4[8] = GMlib::Vector<float,3>(11, 5, 7);
    controlPointsCurve4[9] = GMlib::Vector<float,3>(12.5, 6, 7);
    controlPointsCurve4[10] = GMlib::Vector<float,3>(13, 7.5, 7);
    controlPointsCurve4[11] = GMlib::Vector<float,3>(12, 13, 7);
    controlPointsCurve4[12] = GMlib::Vector<float,3>(13, 13, 7);
    controlPointsCurve4[13] = GMlib::Vector<float,3>(14, 7, 7);
    controlPointsCurve4[14] = GMlib::Vector<float,3>(15, 6, 7);
    controlPointsCurve4[15] = GMlib::Vector<float,3>(16, 7, 7);
    controlPointsCurve4[16] = GMlib::Vector<float,3>(16, 9, 7);


    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve5(7);
    controlPointsCurve5[0] = GMlib::Vector<float,3>(5, 5, 7);
    controlPointsCurve5[1] = GMlib::Vector<float,3>(4, 9, 7);
    controlPointsCurve5[2] = GMlib::Vector<float,3>(6, 10, 7);
    controlPointsCurve5[3] = GMlib::Vector<float,3>(9, 4, 7);
    controlPointsCurve5[4] = GMlib::Vector<float,3>(9, 6, 7);
    controlPointsCurve5[5] = GMlib::Vector<float,3>(10, 12, 7);
    controlPointsCurve5[6] = GMlib::Vector<float,3>(12, 10, 7);

    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve6(6);
    controlPointsCurve6[0] = GMlib::Vector<float,3>(3, 2, 7);
    controlPointsCurve6[1] = GMlib::Vector<float,3>(4, 10, 7);
    controlPointsCurve6[2] = GMlib::Vector<float,3>(8, 17, 7);
    controlPointsCurve6[3] = GMlib::Vector<float,3>(12, 17, 7);
    controlPointsCurve6[4] = GMlib::Vector<float,3>(16, 16, 7);
    controlPointsCurve6[5] = GMlib::Vector<float,3>(19, 10, 7);

    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve7(9);
    controlPointsCurve7[0] = GMlib::Vector<float,3>(3, 3, 7);
    controlPointsCurve7[1] = GMlib::Vector<float,3>(1, 7, 7);
    controlPointsCurve7[2] = GMlib::Vector<float,3>(4, 9, 7);
    controlPointsCurve7[3] = GMlib::Vector<float,3>(8, 1, 7);
    controlPointsCurve7[4] = GMlib::Vector<float,3>(13, 5, 7);
    controlPointsCurve7[5] = GMlib::Vector<float,3>(5, 15, 7);
    controlPointsCurve7[6] = GMlib::Vector<float,3>(8, 18, 7);
    controlPointsCurve7[7] = GMlib::Vector<float,3>(15, 12, 7);
    controlPointsCurve7[8] = GMlib::Vector<float,3>(16, 17, 7);


    GMlib::DVector<GMlib::Vector<float,3>> controlPointsCurve8(7);
    controlPointsCurve8[0] = GMlib::Vector<float,3>(3, 3, 7);
    controlPointsCurve8[1] = GMlib::Vector<float,3>(4, -1, 7);
    controlPointsCurve8[2] = GMlib::Vector<float,3>(4, 13, 7);
    controlPointsCurve8[3] = GMlib::Vector<float,3>(11, 5, 7);
    controlPointsCurve8[4] = GMlib::Vector<float,3>(12, 8, 7);
    controlPointsCurve8[5] = GMlib::Vector<float,3>(12, 19, 7);
    controlPointsCurve8[6] = GMlib::Vector<float,3>(15, 15, 7);



    //  Lofted test
    auto bezier1 = new GMlib::PBezierCurve<float>(controlPointsCurve1);
    bezier1->toggleDefaultVisualizer();
    bezier1->translate(GMlib::Vector<float,3>(-2.0f, 5.0f, -7.0f));
    bezier1->sample(60,0);


    auto bezier2 = new GMlib::PBezierCurve<float>(controlPointsCurve2);
    bezier2->translate(GMlib::Vector<float,3>(0.0f, -2.0f, -5.0f));
    bezier2->toggleDefaultVisualizer();
    bezier2->sample(60,0);


    auto bezier3 = new GMlib::PBezierCurve<float>(controlPointsCurve3);
    bezier3->translate(GMlib::Vector<float,3>(5.0f, -3.0f, -4.0f));
    bezier3->toggleDefaultVisualizer();
    bezier3->sample(60,0);


    auto bezier4 = new GMlib::PBezierCurve<float>(controlPointsCurve4);
    bezier4->translate(GMlib::Vector<float,3>(-2.0f, 3.0f, -3.0f));
    bezier4->toggleDefaultVisualizer();
    bezier4->sample(60,0);

    auto bezier5 = new GMlib::PBezierCurve<float>(controlPointsCurve5);
    bezier5->translate(GMlib::Vector<float,3>(2.0f, -4.0f, -1.0f));
    bezier5->toggleDefaultVisualizer();
    bezier5->sample(60,0);

    auto bezier6 = new GMlib::PBezierCurve<float>(controlPointsCurve6);
    bezier6->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 2.0f));
    bezier6->toggleDefaultVisualizer();
    bezier6->sample(60,0);

    auto bezier7 = new GMlib::PBezierCurve<float>(controlPointsCurve7);
    bezier7->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 5.0f));
    bezier7->toggleDefaultVisualizer();
    bezier7->sample(60,0);

    auto bezier8 = new GMlib::PBezierCurve<float>(controlPointsCurve8);
    bezier8->translate(GMlib::Vector<float,3>(0.0f, 0.0f, 9.0f));
    bezier8->toggleDefaultVisualizer();
    bezier8->sample(60,0);


    if(showCurves){
        this->scene()->insert(bezier1);
        this->scene()->insert(bezier2);
        this->scene()->insert(bezier3);
        this->scene()->insert(bezier4);
        this->scene()->insert(bezier5);
        this->scene()->insert(bezier6);
        this->scene()->insert(bezier7);
        this->scene()->insert(bezier8);
    }


    loftedBaseCurves[0] = bezier1;
    loftedBaseCurves[1] = bezier2;
    loftedBaseCurves[2] = bezier3;
    loftedBaseCurves[3] = bezier4;
    loftedBaseCurves.push_back(bezier5);
    loftedBaseCurves.push_back(bezier6);
    loftedBaseCurves.push_back(bezier7);
    loftedBaseCurves.push_back(bezier8);
}



auto n_viz = new GMlib::PSurfNormalsVisualizer<float,3>();
auto d_viz = new GMlib::PSurfDerivativesVisualizer<float,3>(0,1);

int sample = 100;
auto loftedsurf = new PLoftedSurf<float>(loftedBaseCurves, sample, V_closed, blending);

if(/*showSubSurf||*/default_vizu){
   loftedsurf->toggleDefaultVisualizer();
}
else{
    loftedsurf->insertVisualizer(new PSurfZebraVisualizer<float, 3>());
    // loftedsurf->insertVisualizer(new PSurfCurvatureVisualizer<float, 3>());
    // loftedsurf->insertVisualizer(new GMlib::PSurfContoursVisualizer<float, 3>());
}
//
// loftedsurf->insertVisualizer(new GMlib::PSurfZebraVisualizerTEST<float, 3>());
// loftedsurf->insertVisualizer(new GMlib::PSurfNormalsVisualizer<float, 3>());
//loftedsurf->insertVisualizer(n_viz);
loftedsurf->sample(sample,sample,1,1);
 if(affine_trans){
    loftedsurf->showControlSubSurf(); //On to be able to apply affine transformation
 }
this->scene()->insert(loftedsurf);

 if(extra_surf){
    auto extrasurf = new PLoftedSurf<float>(loftedBaseCurves, sample, V_closed, blending);
    extrasurf->toggleDefaultVisualizer();
    //loftedsurf->insertVisualizer(n_viz);
    extrasurf->sample(sample,sample,1,1);
    if(affine_trans){
        extrasurf->showControlSubSurf(); //On to be able to apply affine transformation
    }
    extrasurf->translate(GMlib::Vector<float,3>(10.0f, 0.0f, 0.0f));
    this->scene()->insert(extrasurf);
}



std::vector<GMlib::Material> rainbow;
rainbow.push_back(GMlib::GMmaterial::chrome());
rainbow.push_back(GMlib::GMmaterial::emerald());
rainbow.push_back(GMlib::GMmaterial::ruby());
rainbow.push_back(GMlib::GMmaterial::sapphire());
rainbow.push_back(GMlib::GMmaterial::obsidian());
rainbow.push_back(GMlib::GMmaterial::bronze());
rainbow.push_back(GMlib::GMmaterial::turquoise());
rainbow.push_back(GMlib::GMmaterial::chrome());
rainbow.push_back(GMlib::GMmaterial::emerald());
rainbow.push_back(GMlib::GMmaterial::ruby());
rainbow.push_back(GMlib::GMmaterial::sapphire());
rainbow.push_back(GMlib::GMmaterial::plastic());
rainbow.push_back(GMlib::GMmaterial::obsidian());
rainbow.push_back(GMlib::GMmaterial::pearl());

//        (GMlib::GMcolor::red(), GMlib::GMcolor::orange(), GMlib::GMcolor::yellow(), GMlib::GMcolor::green(), GMlib::GMcolor::blue(), GMlib::GMcolor::purple(), GMlib::GMcolor::white(), GMlib::GMcolor::black());

if(showSubSurf){
    for(int i = 0; i < loftedsurf->_subS.size(); i++){
        // loftedsurf->_subS[i]->toggleDefaultVisualizer();
        loftedsurf->_subS[i]->insertVisualizer(new GMlib::PSurfDefaultVisualizer<float, 3>());
        // loftedsurf->_subS[i]->insertVisualizer(new GMlib::PSurfNormalsVisualizer<float, 3>());
       // loftedsurf->_subS[i]->insertVisualizer(n_viz);
       // loftedsurf->_subS[i]->insertVisualizer(d_viz);
        // loftedsurf->_subS[i]->enableDefaultVisualizer();
        loftedsurf->_subS[i]->setMaterial(rainbow[i]);
        loftedsurf->_subS[i]->sample(sample,sample,1,1);
        loftedsurf->_subS[i]->showSelectors();
         if(!affine_trans){
             loftedsurf->_subS[i]->translate(GMlib::Vector<float,3>(0.0f, 14.0f * std::pow(-1,i), 0.0f));
         }
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

