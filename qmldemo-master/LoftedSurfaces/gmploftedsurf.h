#ifndef GMPLOFTEDSURF_H
#define GMPLOFTEDSURF_H
//GM parametrics surface


//#include"../../gmlib-master/modules/parametrics/curves/gmpbeziercurve.h"
//#include"../../gmlib-master/modules/parametrics/curves/gmpcircle.h"
#include "../../gmlib-master/modules/parametrics/gmpcurve.h"
#include "../../gmlib-master/modules/parametrics/gmpsurf.h"
//#include "subpatch.h"

template <typename T>
class PLoftedSurf : public GMlib::PSurf<T,3>{
GM_SCENEOBJECT(PLoftedSurf)

public:
    PLoftedSurf(const std::vector<GMlib::PCurve<T,3>*>& curves, int sample, bool closedV = false, bool blend = false);
    PLoftedSurf(const std::vector<GMlib::PCurve<T,3>*>& curves, std::vector<T> Vvalues); //Constructor only used to create sub surfaces of lofted type.
    PLoftedSurf(const PLoftedSurf<T>& copy );
    virtual ~PLoftedSurf();

    // from PSurf
    bool isClosedU() const override;
    bool isClosedV() const override;

    void showControlSubSurf();
    void edit(GMlib::SceneObject *lp) override;

protected:
    // Virtual function from PSurf that has to be implemented locally
    void eval(T u, T v, int d1, int d2, bool lu = true, bool lv = true ) const override;
    T getStartPU() const override;
    T getEndPU()   const override;
    T getStartPV() const override;
    T getEndPV()   const override;
    T getLagrangePolynom(int i, T t, int k, int d) const;
    T getDLagrangePolynom(int i, T t, int d) const; // Derevatives of lagrange polynom
    std::vector<T> setVvalues() const;
    GMlib::Point<T,3> getAvgPointOfCurve(GMlib::PCurve<T,3>* curve) const;

    //Blending related
    void generateSubSurfs();
    GMlib::Vector<T,3> blend(T t, int i) const;
    T W(int i, T t) const;
    T dW(int i, T t) const;
    GMlib::Vector<T,2> B_polynomial_function(T t) const;
    int getIndex(T t) const;



    // Protected data for the surface
    std::vector<GMlib::PCurve<T,3>*> _curves;
    std::vector<T> _Vvalues;
    std::vector<float> _curvePosT {0, 0.25, 0.5, 0.75, 1};
    bool _closedV;
    int _sample;
    T _start;
    T _end;


    //Blending related
    bool _blend;
    GMlib::Vector<float,3> _trans = GMlib::Vector<float,3>(0.0f, 0.0f, 0.0f);

public:
    std::vector<PLoftedSurf<T>*> _subS;

};










// Include PSweepSurf class function implementations
//#include "gmploftedsurf.c"

template <typename T>
inline
PLoftedSurf<T>::PLoftedSurf(const std::vector<GMlib::PCurve<T,3>*>& curves, int sample, bool closedV, bool blend){
    _curves = curves;
    _blend = blend;
    _closedV = closedV;
    _sample = sample;

    for(int i = 0; i < _curves.size(); i++){
        _curves[i]->setDomain(0.0f, 1.0f);
        std::cout << _curves[i]->getPos() << std::endl;
    }

    if(_closedV){
        _curves.push_back(_curves[0]);
    }

    _Vvalues = setVvalues(); //Must be at the end, so all the curves are taken into consideration
    if(/*closedV &&*/ !blend){
        _start = _Vvalues[0];
        _end = _Vvalues[_Vvalues.size()-1];
    }
    else{
        _start = _Vvalues[1];
        _end = _Vvalues[_Vvalues.size()-2];
    }


    if(_blend){
        generateSubSurfs();
    }
}

template <typename T>
inline
PLoftedSurf<T>::PLoftedSurf(const std::vector<GMlib::PCurve<T,3>*>& curves, std::vector<T> Vvalues){
    //Constructor only used to create sub surfaces of lofted type.
    _curves = curves;
    _blend = false;
    _closedV = false;
    _Vvalues = Vvalues;

    _start = _Vvalues[0];
    _end = _Vvalues[_Vvalues.size()-1];

    bool condition = (Vvalues[0] == 0)&&(Vvalues.size() == 2); //if first subS origo at, if last subS origo at end, else subS origo in middel
    int index = (condition ? 0 : 1);
    GMlib::PCurve<T,3>* curve = curves[index];

    T m_curve = curve->getParEnd()/T(2.0); //Is 0.5, since we hardcode that the curve domain is 0 to 1;

    GMlib::DVector<GMlib::Vector<T,3> > tr = curve->evaluateParent(m_curve, 0);
    _trans = tr[0];
    this->translateParent( _trans );
}

template <typename T>
inline
PLoftedSurf<T>::PLoftedSurf(const PLoftedSurf<T>& copy) : GMlib::PSurf<T,3>(copy) {}

template <typename T>
PLoftedSurf<T>::~PLoftedSurf() {}


template<typename T>
inline
bool PLoftedSurf<T>::isClosedU() const {
    for(int i = 0; i < _curves.size(); i++){
        if(!_curves[i]->isClosed()){ //If one curve is not closed the surface can not be closed.
            return _curves[i]->isClosed();
        }
    }
    return _curves[0]->isClosed(); //If all curves are closed, then the surface is closed in U.
}


template<typename T>
inline
bool PLoftedSurf<T>::isClosedV() const {
    return _closedV;
}


template <typename T>
void PLoftedSurf<T>::showControlSubSurf() {

    for(int i = 0; i < _subS.size(); i++){
        _subS[i]->toggleDefaultVisualizer();
        _subS[i]->sample(_sample, _sample, 1, 1);
        _subS[i]->setCollapsed(true);
        this->insert(_subS[i]);
    }
}

template <typename T>
void PLoftedSurf<T>::edit(GMlib::SceneObject* /*lp*/) {
  setEditDone();
  sample(_sample, _sample);
}




template<typename T>
void PLoftedSurf<T>::eval(T u, T v, int d1, int d2, bool lu = true, bool lv = true ) const{
    this->_p.setDim(d1+1,d2+1);

    if(_blend){
        int i = getIndex(v);
        auto b = blend(v, i); // [0] = 1-bow, [1] = bow, [2] = dbow derivative of bow

        auto subS1 = _subS[i-1]->evaluateParent(u, v, d1, d2);

        if(i == _subS.size()){
            i = 0;
            v -= this->getParDeltaV();
        }
        auto subS2 = _subS[i]->evaluateParent(u, v, d1, d2);

        auto S = b[0] * subS1[0][0] + b[1] * subS2[0][0];
        auto Su = b[0] * subS1[1][0] + b[1] * subS2[1][0];
        auto Sv = -b[2] * subS1[0][0] + b[0] * subS1[0][1] + b[2] * subS2[0][0] + b[1] * subS2[0][1];


        this->_p[0][0] = S;
        this->_p[1][0] = Su;
        this->_p[0][1] = Sv;

    }
    else{
        int order = _curves.size();
        auto S  = GMlib::Vector<T,3>(0,0,0);// = L1*c1+L2*c2...Ln*cn
        auto Su = GMlib::Vector<T,3>(0,0,0);
        auto Sv = GMlib::Vector<T,3>(0,0,0);


        for(int i = 0; i < order; i++){ // SUM(bi(v)*ci(u) = S       SUM(bi'(v)*ci(u) = Sv         SUM(bi(v)*ci'(u) = Su
            auto L  = getLagrangePolynom(i, v, i, order);
            auto ci = _curves[i]->evaluateParent(u,d1);
            S +=  L*ci[0];



            //Derivatives
            if( this->_dm == GMlib::GM_DERIVATION_EXPLICIT){
                if(d1){
                    Su +=  L*ci[1]; //L(i)*ci'(u)
                }
                if(d2){
                    auto dL = getDLagrangePolynom(i, v, order);
                    Sv +=  dL*ci[0]; //L'(i)*ci(u)
                }
            }
        }


        this->_p[0][0] = S;
        this->_p[1][0] = Su;
        this->_p[0][1] = Sv;

        this->_p[0][0] -= _trans;
    }
}


template <typename T>
T PLoftedSurf<T>::getStartPU() const {
  return _curves[0]->getParStart();
}


template <typename T>
T PLoftedSurf<T>::getEndPU() const {
  return _curves[0]->getParEnd();
}


template <typename T>
T PLoftedSurf<T>::getStartPV() const {
    return _start;
}


template <typename T>
T PLoftedSurf<T>::getEndPV() const {
    return _end;
}

template <typename T>
GMlib::Point<T,3> PLoftedSurf<T>::getAvgPointOfCurve(GMlib::PCurve<T,3>* curve) const{
//    This would be best in PCurve, but would prefer to not touch GMlib too much
    GMlib::Point<T,3> p = curve->getPosition(_curvePosT[0]);
//    GMlib::Point<T,3> p = curve->getPos();

    for(int i = 1; i < _curvePosT.size(); i++){
        p = p + curve->getPosition(_curvePosT[i]);
    }

    return p/_curvePosT.size();
}


template <typename T>
std::vector<T> PLoftedSurf<T>::setVvalues() const{
//    Need to get avg coordinate of each curve and add all
//    their distences to a tot length

//    Then divide each lenght with the tot length to get the "prosent" of each curve

//    if _blend is true _Vvalues turn into a "knotvector" in v-direction
    std::vector<T> V_values;
    V_values.push_back(0);

    if(_blend){
        V_values.push_back(0);
    }

    for(int i = 1; i < _curves.size(); i++){
//        auto p_i = getAvgPointOfCurve(_curves[i]);
//        auto p_i1 = getAvgPointOfCurve(_curves[i-1]);
        auto p_i = _curves[i]->getPos();
        auto p_i1 = _curves[i-1]->getPos();


        GMlib::Vector<T,3> d = p_i-p_i1;
        auto dLength = d.getLength();
        auto Vvalue_i =  dLength + V_values[V_values.size()-1];
        V_values.push_back(Vvalue_i);
    }

    if(_blend){
        V_values.push_back(V_values[V_values.size()-1]);

        if(_closedV){
            V_values[0] = V_values[0] - (V_values[V_values.size()-2] - V_values[V_values.size()-3]); //start - (distance between 2nd last and 3nd last knot)
            V_values[V_values.size()-1] = V_values[V_values.size()-1] + (V_values[2] - V_values[1]); //end + (distance between 2nd and 3th knot)
        }
    }

    return V_values;


//    GMlib::DVector<T> V_values;
//    V_values.append(0);

//    if(_blend){
//        V_values.append(0);
//    }

//    for(int i = 1; i < _curves.size(); i++){
//        auto p_i = getAvgPointOfCurve(_curves[i]);
//        auto p_i1 = getAvgPointOfCurve(_curves[i-1]);

//        GMlib::Vector<T,3> d = p_i-p_i1;
//        auto dLength = d.getLength();
//        auto Vvalue_i =  dLength + V_values[i-1];
//        V_values.append(Vvalue_i);
//    }

//    if(_blend){
//        V_values.append(V_values[V_values.getDim()-1]);

//        if (_closedV) {
////            V_values[0] = V_values[V_values.getDim()-1]; //start - (distance between last and 2nd last knot)
////            V_values[V_values.getDim()-1] = V_values[1]; //end + (distance between first and 2nd knot)
//            V_values[0] = V_values[0] - (V_values[V_values.getDim()-2] - V_values[V_values.getDim()-3]); //start - (distance between last and 2nd last knot)
//            V_values[V_values.getDim()-1] = V_values[V_values.getDim()-1] + (V_values[2] - V_values[1]); //end + (distance between first and 2nd knot)
//        }
//    }


//    V_values = V_values/V_values[V_values.getDim()-2]; //Why i choose Dvector and not std::vector

//    std::vector<T> out;
//    for(int i = 0; i < V_values.getDim(); i++){
//        out.push_back(V_values[i]);
//    }

//    return  out;
}


template <typename T>
T PLoftedSurf<T>::getLagrangePolynom(int i, T t, int k, int order) const {
//    Gets the lagrange polynomial for L(d,i)=(t-Vj)/(Vi-Vj)
//    k is only !=i, if this is used in derivative of L
    T L = 1;

    for(int j = 0; j < order; j++){
        if(j==i || j==k){
            continue;
        }
        auto L_part = (t - _Vvalues[j])/(_Vvalues[i] - _Vvalues[j]);
        L = L * L_part;
    }
    return L;
}




template <typename T>
T PLoftedSurf<T>::getDLagrangePolynom(int i, T t, int order) const {
//    Gets the lagrange polynomial for L'(d,i)=(1)/(t-Vj)
    T dL = 0;

    for(int j = 0; j < order; j++){
        if(j!=i){
            auto dL_part = T(1)/(_Vvalues[i] - _Vvalues[j]);
            auto L = getLagrangePolynom(i, t, j, order);

            dL += dL_part * L;
        }
    }
    return dL;
}

template <typename T>
void PLoftedSurf<T>::generateSubSurfs(){
    _subS.clear();
    if(_closedV){
        int n = _Vvalues.size()-3; // _curve.size()-1
//        std::cout << "n = " << n << std::endl;

        std::vector<GMlib::PCurve<T,3>*> lapCurves{_curves[_curves.size()-2], _curves[0], _curves[1]};
        _subS.push_back(new PLoftedSurf<T>(lapCurves, std::vector<T>{_Vvalues[0], _Vvalues[1], _Vvalues[2]}));

        for(int i = 1; i < n; i++){
            auto Vvalues = std::vector<T>{_Vvalues[i], _Vvalues[i+1], _Vvalues[i+2]};
            auto curves_sub = std::vector<GMlib::PCurve<T,3>*>{_curves[i-1], _curves[i], _curves[i+1]};

            _subS.push_back(new PLoftedSurf<T>(curves_sub, Vvalues)); //Normal surfs

//            std::cout << i << std::endl;
        }
    }
    else{
        _subS.push_back(new PLoftedSurf<T>(std::vector<GMlib::PCurve<T,3>*>{_curves[0], _curves[1]}, std::vector<T>{_Vvalues[1], _Vvalues[2]})); //First short surf

        auto n = _curves.size()-2;
        auto v_last = _Vvalues.size()-1;

        for(int i = 0; i < n; i++){
            std::vector<GMlib::PCurve<T,3>*> curves_sub_3;
            curves_sub_3.push_back(_curves[i]);
            curves_sub_3.push_back(_curves[i+1]);
            curves_sub_3.push_back(_curves[i+2]);


            _subS.push_back(new PLoftedSurf<T>(curves_sub_3, std::vector<T>{_Vvalues[i+1], _Vvalues[i+2], _Vvalues[i+3]})); //Normal surfs
        }
        _subS.push_back(new PLoftedSurf<T>(std::vector<GMlib::PCurve<T,3>*>{_curves[n], _curves[n+1]}, std::vector<T>{_Vvalues[v_last-2], _Vvalues[v_last-1]})); //Last short surf
    }

}


template <typename T>
GMlib::Vector<T,3> PLoftedSurf<T>::blend(T t, int i) const{
    GMlib::Vector<T,2> Bow1 = B_polynomial_function(W(i, t));
    GMlib::Vector<T,2> Bodw1 = B_polynomial_function(dW(i, t));

    return GMlib::Vector<T,3>(1 - Bow1[0], Bow1[0], Bow1[1] + Bodw1[0]);
}

template <typename T>
GMlib::Vector<T,2> PLoftedSurf<T>::B_polynomial_function(T t) const {
    return GMlib::Vector<T,2>(3*t*t - 2*t*t*t, 6*t - 6*t*t);
}

template <typename T>
T PLoftedSurf<T>::W(int i, T t) const {
    return (t - _Vvalues[i])/(_Vvalues[i+1] - _Vvalues[i]);
}

template <typename T>
T PLoftedSurf<T>::dW(int i, T t) const {
    return T(1)/(_Vvalues[i+1] - _Vvalues[i]);
}

template <typename T>
int PLoftedSurf<T>::getIndex(T t) const{
    if (t >= this->getEndPV()) {
        return _Vvalues.size() - 3;
    }
    else {
        auto const next = std::upper_bound(_Vvalues.begin(), _Vvalues.end(), t);
        auto n = (_closedV ? 1 : 1);
        return std::distance(_Vvalues.begin(), next) - n;
    }
}


#endif // GMPLOFTEDSURF_H
