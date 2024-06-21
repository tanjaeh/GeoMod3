#include "astroidCurve.h"


template <typename T>
inline
AstroidCurve<T>::AstroidCurve( T radius ) : GMlib::PCurve<T,3>(20, 0, 7) {
    //Default default radius = 20
    // Note that the last parameter in the PCurve constructor is 7,
    // this because 7 derivatives in eval() is implemented!!!!
    _rx = radius;
    _rw = radius;
}


template <typename T>
inline
AstroidCurve<T>::AstroidCurve(  T radius1, T radius2 ) : PCurve<T,3>(20, 0, 7) {
    //Default default radius = 20
    // Note that the last parameter in the PCurve constructor is 7,
    // this because 7 derivatives in eval() is implemented!!!!
    _rx = radius1;
    _rw = radius2;
}


template <typename T>
inline
AstroidCurve<T>::AstroidCurve( const AstroidCurve<T>& copy ) : PCurve<T,3>(copy) {}


//The destructor clean up and destroy all private data
template <typename T>
AstroidCurve<T>::~AstroidCurve() {}



template <typename T>
inline
T AstroidCurve<T>::getRadius(int i) const {
    if(i==1)
        return _rx;
    else {
        return _rw;
    }
}


template <typename T>
inline
void AstroidCurve<T>::setRadius( T radius ) {
    _rx = _rw = radius;
}


template <typename T>
inline
void AstroidCurve<T>::setRadius( T radius1, T radius2 ) {
    _rx = radius1;
    _rw = radius2;
}




template <typename T>
bool AstroidCurve<T>::isClosed() const {
    return true;
}


template <typename T>
void AstroidCurve<T>::eval( T t, int d, bool /*l*/ ) const { //bool l tells if it eval from left (default True)
    //Next line must join all eval functions
    this->_p.setDim( d + 1 );


    auto b_x = _rx/4;
    auto b_y = _rw/4;
    const T ct_x = 3 * b_x * cos(t) + b_x * cos(3*t);
    const T st_w = 3 * b_y * sin(t) - b_y * sin(3*t);

    this->_p[0][0] = ct_x;
    this->_p[0][1] = T(0);
    this->_p[0][2] = st_w;

    if( this->_dm == GMlib::GM_DERIVATION_EXPLICIT ) {
      const T st_x = _rx * sin(t);
      const T ct_y = _rw * cos(t);
      if( d > 0 ) {
        this->_p[1][0] = -st_x;
        this->_p[1][1] =  ct_y;
        this->_p[1][2] =  T(0);
      }
      if( d > 1 ) this->_p[2] = -this->_p[0];
      if( d > 2 ) this->_p[3] = -this->_p[1];
      if( d > 3 ) this->_p[4] =  this->_p[0];
      if( d > 4 ) this->_p[5] =  this->_p[1];
      if( d > 5 ) this->_p[6] =  this->_p[2];
      if( d > 6 ) this->_p[7] =  this->_p[3];
    }
}


template <typename T>
T AstroidCurve<T>::getStartP() const {
    return T(0);
}


template <typename T>
T AstroidCurve<T>::getEndP()const {
    return T( M_2PI );
    // return T( M_PI );
}


template <typename T>
void AstroidCurve<T>::computeSurroundingSphere( const std::vector<GMlib::DVector<GMlib::Vector<T,3>>>& /*p*/, GMlib::Sphere<T,3>& s ) const {

    s.resetPos(GMlib::Point<T,3>(T(0)));
    if(_rx > _rw)
        s.resetRadius(_rx);
    else
        s.resetRadius(_rw);
}

