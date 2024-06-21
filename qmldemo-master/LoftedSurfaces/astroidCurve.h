#ifndef ASTROIDCURVE_H
#define ASTROIDCURVE_H

#include "../../gmlib-master/modules/parametrics/gmpcurve.h"


template <typename T>
class AstroidCurve : public GMlib::PCurve<T,3> {
    GM_SCENEOBJECT(AstroidCurve)

public:
    AstroidCurve( T radius = T(20) );
    AstroidCurve(  T radius1, T radius2 );
    AstroidCurve( const AstroidCurve<T>& copy );
    virtual ~AstroidCurve();

    // Public local functions
    T               getRadius(int i=1) const;
    void            setRadius( T radius = T(20) );
    void            setRadius( T radius1, T radius2);

    //****************************************
    //****** Virtual public functions   ******
    //****************************************

    // from PCurve
    bool            isClosed() const override;

protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void            eval(T t, int d, bool l) const override;
    T               getStartP() const override;
    T               getEndP()   const override;

    void            computeSurroundingSphere( const std::vector<GMlib::DVector<GMlib::Vector<T,3>>>& /*p*/, GMlib::Sphere<T,3>& s ) const override;

    // Protected data for the curve
    T               _rx;
    T               _rw;

}; // END class astroidCurve

// Include modelCurve class function implementations
#include "astroidCurve.c"

#endif // ASTROIDCURVE_H
