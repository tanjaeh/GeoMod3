#ifndef TEST_VISUALIZER_H
#define TEST_VISUALIZER_H

#include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfvisualizer.h"

// // gmlib
// #include <../../gmlib-master/modules/core/types/gmpoint.h>
// #include <../../gmlib-master/modules/core/containers/gmdvector.h>
// #include <../../gmlib-master/modules/core/containers/gmdmatrix.h>
// #include <../../gmlib-master/modules/core/utils/gmcolor.h>
// #include <../../gmlib-master/modules/opengl/gmprogram.h>
// #include <../../gmlib-master/modules/opengl/bufferobjects/gmvertexbufferobject.h>


namespace GMlib {

enum GM_SURF_NORMALSVISUALIZER_MODE2 {
    GM_SURF_NORMALSVISUALIZER_ALL2,
    GM_SURF_NORMALSVISUALIZER_INTERIOR2,
    GM_SURF_NORMALSVISUALIZER_BOUNDARY2
};

template <typename T, int n>
class PSurfZebraVisualizerTEST : public PSurfVisualizer<T,n> {
    GM_VISUALIZER(PSurfZebraVisualizerTEST)
public:
    PSurfZebraVisualizerTEST();
    PSurfZebraVisualizerTEST( const PSurfZebraVisualizerTEST<T,n>& copy );

    void                              render( const SceneObject* obj, const DefaultRenderer* renderer) const override;

    void                              update() override;

    const Color&                      getColor() const;
    void                              setColor( const Color& color );

    GM_SURF_NORMALSVISUALIZER_MODE2    getMode() const;
    void                              setMode( GM_SURF_NORMALSVISUALIZER_MODE2 mode );

    double                            getSize() const;
    void                              setSize( double size = 1.0 );

protected:
    GL::Program                       _prog;

    GL::VertexBufferObject            _vbo;
    int                               _no_elements;

    Color                             _color;
    double                            _size;

    GM_SURF_NORMALSVISUALIZER_MODE2    _mode;

    void                              makePlotAll( const DMatrix< DMatrix< Vector<T, 3> > >& p, const DMatrix< Vector<float, 3> >& normals );
    void                              makePlotInterior( const DMatrix< DMatrix< Vector<T, 3> > >& p, const DMatrix< Vector<float, 3> >& normals );
    void                              makePlotBoundary( const DMatrix< DMatrix< Vector<T, 3> > >& p, const DMatrix< Vector<float, 3> >& normals );


};

} // END namespace GMlib



namespace GMlib {

template <typename T, int n>
PSurfZebraVisualizerTEST<T,n>::PSurfZebraVisualizerTEST()
    : _no_elements(0),_color( GMcolor::black() ),
    _size(1.0), _mode(GM_SURF_NORMALSVISUALIZER_ALL2) {

    _prog.acquire("color");
    _vbo.create();
}


template <typename T, int n>
PSurfZebraVisualizerTEST<T,n>::PSurfZebraVisualizerTEST(const PSurfZebraVisualizerTEST<T,n>& copy)
    : PSurfVisualizer<T,n>(copy),
    _no_elements(0), _color(copy._color), _size(copy._size), _mode(copy._mode) {

    _prog.acquire("color");
    _vbo.create();
}


template <typename T, int n>
inline
    void PSurfZebraVisualizerTEST<T,n>::render( const SceneObject* obj, const DefaultRenderer *renderer) const {

    const HqMatrix<float,3> &mvpmat = obj->getModelViewProjectionMatrix(renderer->getCamera());

    _prog.bind(); {

        _prog.uniform( "u_mvpmat", mvpmat );
        _prog.uniform( "u_color", _color );

        GL::AttributeLocation vert_loc = _prog.getAttributeLocation( "in_vertex" );

        _vbo.bind();
        _vbo.enable( vert_loc, 3, GL_FLOAT, GL_FALSE, 0, static_cast<const GLvoid*>(0x0) );

        // Draw
        GL_CHECK(glDrawArrays( GL_LINES, 0, _no_elements ));

        _vbo.disable( vert_loc );
        _vbo.unbind();

    } _prog.unbind();
}


template <typename T, int n>
const Color& PSurfZebraVisualizerTEST<T,n>::getColor() const {

    return _color;
}


/*! GM_SURF_NORMALSVISUALIZER_MODE PSurfNormalsVisualizer<T,n>::getMode() const
   *
   *  Returns the displaymode of the visualizer.
   *  All       - Shows all normals.
   *  Boundary  - Shows only the normals on the boundary.
   *  Interior  - Shows only the interior normals.
   *
   *  \return Visualizer normals mode.
   */
template <typename T, int n>
GM_SURF_NORMALSVISUALIZER_MODE2 PSurfZebraVisualizerTEST<T,n>::getMode() const {

    return _mode;
}


template <typename T, int n>
double PSurfZebraVisualizerTEST<T,n>::getSize() const {

    return _size;
}


/*! void PSurfNormalsVisualizer<T,n>::makePlotAll( DMatrix< DMatrix< Vector<T, 3> > >& p, DMatrix< Vector<float, 3> >& normals )
   *
   *  Generates the plot data for all normals.
   *
   *  \param[in]  p         Evaluated position data.
   *  \param[in]  normals   Evaluated Normal data.
   */
template <typename T, int n>
void PSurfZebraVisualizerTEST<T,n>::makePlotAll( const DMatrix< DMatrix< Vector<T, 3> > >& p, const DMatrix< Vector<float, 3> >& normals ) {

    int no_normals = p.getDim1() * p.getDim2();
    _no_elements = no_normals * 2;

    _vbo.bind();
    _vbo.bufferData( no_normals * 2 * sizeof(GL::GLVertex), 0x0, GL_STATIC_DRAW );

    GL::GLVertex *ptr = _vbo.mapBuffer<GL::GLVertex>();
    if( ptr ) {

        for( int i = 0; i < p.getDim1(); i++ ) {
            for( int j = 0; j < p.getDim2(); j++ ) {

                const Point<T,3> &pos = p(i)(j)(0)(0);
                (*ptr).x = float(pos(0));
                (*ptr).y = float(pos(1));
                (*ptr).z = float(pos(2));
                ptr++;

                const Vector<T,3> N = normals(i)(j).getNormalized() * _size;
                (*ptr).x = float(pos(0)) + N(0);
                (*ptr).y = float(pos(1)) + N(1);
                (*ptr).z = float(pos(2)) + N(2);
                ptr++;
            }
        }
    }

    _vbo.unmapBuffer();
    _vbo.unbind();
}


/*! void PSurfNormalsVisualizer<T,n>::makePlotInterior( DMatrix< DMatrix< Vector<T, 3> > >& p, DMatrix< Vector<float, 3> >& normals )
   *
   *  Generates the plot data for all interior normals.
   *
   *  \param[in]  p         Evaluated position data.
   *  \param[in]  normals   Evaluated Normal data.
   */
template <typename T, int n>
void PSurfZebraVisualizerTEST<T,n>::makePlotInterior( const DMatrix< DMatrix< Vector<T, 3> > >& p, const DMatrix< Vector<float, 3> >& normals ) {

    int no_normals = ( p.getDim1() - 2 ) * ( p.getDim2() - 2 );
    _no_elements = no_normals * 2;

    _vbo.bufferData( no_normals * 2 * sizeof(GL::GLVertex), 0x0, GL_STATIC_DRAW );

    GL::GLVertex *ptr = _vbo.mapBuffer<GL::GLVertex>();
    if( ptr ) {

        for( int i = 1; i < p.getDim1()-1; i++ ) {
            for( int j = 1; j < p.getDim2()-1; j++ ) {

                const Point<T,3> &pos = p(i)(j)(0)(0);
                (*ptr).x = float(pos(0));
                (*ptr).y = float(pos(1));
                (*ptr).z = float(pos(2));
                ptr++;

                const Vector<T,3> N = normals(i)(j).getNormalized() * _size;
                (*ptr).x = float(pos(0)) + N(0);
                (*ptr).y = float(pos(1)) + N(1);
                (*ptr).z = float(pos(2)) + N(2);
                ptr++;
            }
        }
    }

    _vbo.unmapBuffer();
}


/*! void PSurfNormalsVisualizer<T,n>::makePlotBoundary( DMatrix< DMatrix< Vector<T, 3> > >& p, DMatrix< Vector<float, 3> >& normals )
   *
   *  Generates the plot data for all boundary normals.
   *
   *  \param[in]  p         Evaluated position data.
   *  \param[in]  normals   Evaluated Normal data.
   */
template <typename T, int n>
void PSurfZebraVisualizerTEST<T,n>::makePlotBoundary( const DMatrix< DMatrix< Vector<T, 3> > >& p, const DMatrix< Vector<float, 3> >& normals ) {

    int no_normals = ( p.getDim1() + p.getDim2() ) * 2 - 4;
    _no_elements = no_normals * 2;

    _vbo.bind();
    _vbo.bufferData( no_normals * 2 * sizeof(GL::GLVertex), 0x0, GL_STATIC_DRAW );

    GL::GLVertex *ptr = _vbo.mapBuffer<GL::GLVertex>();
    if( ptr ) {

        for( int i = 0, j; i < p.getDim1(); i++ ) {

            // j = 0
            j = 0;

            const Point<T,3> &pos = p(i)(j)(0)(0);
            (*ptr).x = float(pos(0));
            (*ptr).y = float(pos(1));
            (*ptr).z = float(pos(2));
            ptr++;

            const Vector<T,3> N1 = normals(i)(j).getNormalized() * _size;
            (*ptr).x = float(pos(0)) + N1(0);
            (*ptr).y = float(pos(1)) + N1(1);
            (*ptr).z = float(pos(2)) + N1(2);
            ptr++;

            // j = p.getDim2() -1
            j = p.getDim2() - 1;

            const Point<T,3> &pos2 = p(i)(j)(0)(0);
            (*ptr).x = float(pos2(0));
            (*ptr).y = float(pos2(1));
            (*ptr).z = float(pos2(2));
            ptr++;

            const Vector<T,3> N2 = normals(i)(j).getNormalized() * _size;
            (*ptr).x = float(pos2(0)) + N2(0);
            (*ptr).y = float(pos2(1)) + N2(1);
            (*ptr).z = float(pos2(2)) + N2(2);
            ptr++;
        }

        for( int i, j = 1; j < p.getDim2()-1; j++ ) {

            // i = 0
            i = 0;

            const Point<T,3> &pos = p(i)(j)(0)(0);
            (*ptr).x = float(pos(0));
            (*ptr).y = float(pos(1));
            (*ptr).z = float(pos(2));
            ptr++;

            const Vector<T,3> N1 = normals(i)(j).getNormalized() * _size;
            (*ptr).x = float(pos(0)) + N1(0);
            (*ptr).y = float(pos(1)) + N1(1);
            (*ptr).z = float(pos(2)) + N1(2);
            ptr++;

            // j = p.getDim1() -1
            i = p.getDim1() - 1;

            const Point<T,3> &pos2 = p(i)(j)(0)(0);
            (*ptr).x = float(pos2(0));
            (*ptr).y = float(pos2(1));
            (*ptr).z = float(pos2(2));
            ptr++;

            const Vector<T,3> N2 = normals(i)(j).getNormalized() * _size;
            (*ptr).x = float(pos2(0)) + N2(0);
            (*ptr).y = float(pos2(1)) + N2(1);
            (*ptr).z = float(pos2(2)) + N2(2);
            ptr++;
        }
    }

    glUnmapBuffer( GL_ARRAY_BUFFER );
    glBindBuffer( GL_ARRAY_BUFFER, 0x0 );
}





template <typename T, int n>
void PSurfZebraVisualizerTEST<T,n>::update() {

    switch( _mode ) {

    case GM_SURF_NORMALSVISUALIZER_INTERIOR:
        makePlotInterior( *(this->_p), *(this->_n) );
        break;

    case GM_SURF_NORMALSVISUALIZER_BOUNDARY:
        makePlotBoundary( *(this->_p), *(this->_n) );
        break;

    case GM_SURF_NORMALSVISUALIZER_ALL:
    default:
        makePlotAll( *(this->_p), *(this->_n) );
        break;
    }
}




template <typename T, int n>
void PSurfZebraVisualizerTEST<T,n>::setColor( const Color& color ) {

    _color = color;
}

/*! void PSurfNormalsVisualizer<T,n>::setMode( GM_SURF_NORMALSVISUALIZER_MODE mode )
   *
   *  Sets the normals mode for the visualizer.
   *
   *  All       - Shows all normals.
   *  Boundary  - Shows only the normals on the boundary.
   *  Interior  - Shows only the interior normals.
   *
   *  \param[in]  mode  The normals mode.
   *
   *  \see PSurfNormalsVisualizer<T,n>::GM_SURF_NORMALSVISUALIZER_MODE
   */
template <typename T, int n>
void PSurfZebraVisualizerTEST<T,n>::setMode( GM_SURF_NORMALSVISUALIZER_MODE2 mode ) {

    _mode = mode;
}

template <typename T, int n>
void PSurfZebraVisualizerTEST<T,n>::setSize( double size ) {

    _size = size;
}



} // END namespace GMlib
#endif // TEST_VISUALIZER_H
