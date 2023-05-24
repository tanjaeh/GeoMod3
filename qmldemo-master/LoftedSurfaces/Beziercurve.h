#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H

#include"../../gmlib-master/modules/parametrics/gmpcurve.h"
#include "../../gmlib-master/modules/parametrics/surfaces/gmpsphere.h"

template <typename T>
class Bezier : public GMlib::PCurve<T,3> {
  GM_SCENEOBJECT(Bezier)


public:
  Bezier( const GMlib::DVector< GMlib::Vector<T,3> >& c );
  Bezier( const Bezier<T>& copy );
  virtual ~Bezier();

  int             getDegree() const;

  const GMlib::DVector<GMlib::Vector<T,3>>&
                  getControlPoints() const;
  void            setControlPoints( const GMlib::DVector<GMlib::Vector<T,3>>& cv );


protected:
  // Virtual functions from PCurve, which have to be implemented locally
  void            eval( T t, int d = 0, bool l = false ) const override;
  T               getStartP() const override;
  T               getEndP()   const override;


  // Protected intrinsic data for the curve
  GMlib::DVector< GMlib::Vector<T, 3> >     _c; //Control points


  // Pre-evaluation of bernstein polynomials at the sample values (basis functions)
  mutable std::vector<GMlib::DMatrix<T>> _pre;        //!< Pre-evaluated basis functions

  // Local help functions
  void       makeBernsteinMat( int m, int d = 0, T scale = T(1) ) const;
  void       multEval(GMlib::DVector<GMlib::Vector<T,3>>& p, const GMlib::DMatrix<T>& bsh, int d) const;
}; // END class Bezier



// Include Bezier class function implementations
    template <typename T>
    inline
    Bezier<T>::Bezier( const GMlib::DVector<GMlib::Vector<T,3>>& c ):GMlib::PCurve<T,3>(0,0,10){
      setControlPoints(c);
    }

    template <typename T>
    inline
    Bezier<T>::Bezier( const Bezier<T>& copy ) : GMlib::PCurve<T,3>( copy ) {
      _c = copy._c;
    }

    template <typename T>
    Bezier<T>::~Bezier() {}



    template <typename T>
    inline
    int Bezier<T>::getDegree() const {
      return _c.getDim() - 1;
    }


    template <typename T>
    inline
    void Bezier<T>::setControlPoints( const GMlib::DVector< GMlib::Vector<T,3> >& c ) {
      _c = c;
    }


    template <typename T>
    inline
    const GMlib::DVector< GMlib::Vector<T,3> >& Bezier<T>::getControlPoints() const {
      return _c;
    }



    template <typename T>
    void Bezier<T>::eval( T t, int d, bool /*l*/ ) const {
      // Compute the Bernstein-Hermite Polynomials
      GMlib::DMatrix< T > bhp;
      GMlib::EvaluatorStatic<T>::evaluateBhp( bhp, getDegree(), t, 1/this->_sc );

      multEval(this->_p, bhp, d);

    }

    template <typename T>
    T Bezier<T>::getStartP() const {
      return T(0);
    }

    template <typename T>
    T Bezier<T>::getEndP() const {
      return T(1);
    }


    template <typename T>
    inline
    void Bezier<T>::makeBernsteinMat( int m, int d, T scale ) const {
        _pre.resize(m);
        for(int i=0; i<m; i++ )
            GMlib::EvaluatorStatic<T>::evaluateBhp(_pre[i], d, this->_map(this->_visu[0][i]), 1/scale);
    }


    template <typename T>
    inline
    void Bezier<T>::multEval(GMlib::DVector<GMlib::Vector<T,3>>& p, const GMlib::DMatrix<T>& B, int d) const {

        p.setDim(d+1);
        // We do this manually because we only want to compute the d first rows of B.
        for(int i=0; i<=d; i++) {
            p[i] = B(i)(0)*_c[0];
            for(int k=1; k<_c.getDim(); k++)
               p[i] += B(i)(k)*_c[k];
        }
    }


#endif // BEZIERCURVE_H
