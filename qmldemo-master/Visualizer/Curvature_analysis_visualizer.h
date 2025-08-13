#ifndef CURVATURE_ANALYSIS_VISUALIZER_H
#define CURVATURE_ANALYSIS_VISUALIZER_H

// #include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfvisualizer.h"
#include "../../gmlib-master/modules/parametrics/visualizers/gmpsurfdefaultvisualizer.h"

// gmlib
#include <core/types/gmpoint.h>
#include <core/containers/gmdvector.h>
#include <core/containers/gmdmatrix.h>
#include <core/utils/gmcolor.h>

#include <iostream>
#include <fstream>
#include <direct.h>



template <typename T, int n>
class PSurfCurvatureVisualizer : public GMlib::PSurfVisualizer<T,n> {
    GM_VISUALIZER(PSurfCurvatureVisualizer)
public:
    PSurfCurvatureVisualizer();
    PSurfCurvatureVisualizer( GMlib::DMatrix<GMlib::DMatrix<GMlib::Vector<T,n>>>& p, GMlib::DMatrix<GMlib::Vector<float,n>>& no );
    PSurfCurvatureVisualizer( const PSurfCurvatureVisualizer<T,n>& copy );

    void    render( const GMlib::SceneObject* obj, const GMlib::DefaultRenderer* renderer ) const override;
    void    renderGeometry( const GMlib::SceneObject* obj, const GMlib::Renderer* renderer, const GMlib::Color& color ) const override;

    void    update() override;

protected:
    GMlib::GL::Program                 _prog;
    GMlib::GL::Program                 _color_prog;

    GMlib::GL::VertexBufferObject      _vbo;
    GMlib::GL::IndexBufferObject       _ibo;
    GMlib::GL::Texture                 _nmap;

    GLuint                      _no_strips;
    GLuint                      _no_strip_indices;
    GLsizei                     _strip_size;

    GLenum                      _mode;

    virtual void                draw() const;

    void                        initShaderProgram();

    void                        _init();

}; // END class PSurfDefaultVisualizer


template <typename T, int n>
inline
    PSurfCurvatureVisualizer<T,n>::PSurfCurvatureVisualizer()
    : _no_strips(0), _no_strip_indices(0), _strip_size(0) {

    _mode = GL_TRIANGLE_STRIP;
    _init();
}


template <typename T, int n>
inline
    PSurfCurvatureVisualizer<T,n>::PSurfCurvatureVisualizer(GMlib::DMatrix<GMlib::DMatrix<GMlib::Vector<T,n>>>& p, GMlib::DMatrix<GMlib::Vector<float,n>>& no)
    : GMlib::PSurfVisualizer<T,n>(p, no), _no_strips(0), _no_strip_indices(0), _strip_size(0) {

    _mode = GL_TRIANGLE_STRIP;
    _init();
}


template <typename T, int n>
inline
    PSurfCurvatureVisualizer<T,n>::PSurfCurvatureVisualizer(const PSurfCurvatureVisualizer<T,n>& copy)
    : GMlib::PSurfVisualizer<T,n>(copy), _no_strips(0), _no_strip_indices(0), _strip_size(0) {

    _mode = copy._mode;
    _init();
}


template <typename T, int n>
void PSurfCurvatureVisualizer<T,n>::render( const GMlib::SceneObject* obj, const GMlib::DefaultRenderer* renderer ) const {

    const GMlib::Camera* cam = renderer->getCamera();
    const GMlib::HqMatrix<float,3> &mvmat = obj->getModelViewMatrix(cam);
    const GMlib::HqMatrix<float,3> &pmat  = obj->getProjectionMatrix(cam);
    //    const SqMatrix<float,3> &nmat = obj->getNormalMatrix(cam);

    GMlib::SqMatrix<float,3>        nmat = mvmat.getRotationMatrix();
    //    nmat.invertOrthoNormal();
    //    nmat.transpose();

    this->glSetDisplayMode();

    _prog.bind(); {

        // Model view and projection matrices
        _prog.uniform( "u_mvmat", mvmat );
        _prog.uniform( "u_mvpmat", pmat * mvmat );
        _prog.uniform( "u_nmat", nmat );

        // Lights
        _prog.bindBufferBase( "DirectionalLights",  renderer->getDirectionalLightUBO(), 0 );
        _prog.bindBufferBase( "PointLights",        renderer->getPointLightUBO(), 1 );
        _prog.bindBufferBase( "SpotLights",         renderer->getSpotLightUBO(), 2 );

        // Material
        const GMlib::Material &m = obj->getMaterial();
        _prog.uniform( "u_mat_amb", m.getAmb() );
        _prog.uniform( "u_mat_dif", m.getDif() );
        _prog.uniform( "u_mat_spc", m.getSpc() );
        _prog.uniform( "u_mat_shi", m.getShininess() );

        // Normal map
        _prog.uniform( "u_nmap", _nmap, GLenum(GL_TEXTURE0), 0 );

        // Get vertex and texture attrib locations
        GMlib::GL::AttributeLocation vert_loc = _prog.getAttributeLocation( "in_vertex" );
        GMlib::GL::AttributeLocation  tex_loc = _prog.getAttributeLocation( "in_tex" );

        // Bind and draw
        _vbo.bind();
        _vbo.enable( vert_loc, 3, GL_FLOAT, GL_FALSE, sizeof(GMlib::GL::GLVertexTex2D), reinterpret_cast<const GLvoid *>(0x0) );
        _vbo.enable( tex_loc,  2, GL_FLOAT, GL_FALSE, sizeof(GMlib::GL::GLVertexTex2D), reinterpret_cast<const GLvoid *>(3*sizeof(GLfloat)) );
        draw();
        _vbo.disable( vert_loc );
        _vbo.disable( tex_loc );
        _vbo.unbind();

    } _prog.unbind();
}



template <typename T, int n>
void PSurfCurvatureVisualizer<T,n>::renderGeometry( const GMlib::SceneObject* obj, const GMlib::Renderer* renderer, const GMlib::Color& color ) const {

    _color_prog.bind();
    _color_prog.uniform( "u_color", color );
    _color_prog.uniform( "u_mvpmat", obj->getModelViewProjectionMatrix(renderer->getCamera()) );
    GMlib::GL::AttributeLocation vertice_loc = _color_prog.getAttributeLocation( "in_vertex" );

    _vbo.bind();
    _vbo.enable( vertice_loc, 3, GL_FLOAT, GL_FALSE, sizeof(GMlib::GL::GLVertexTex2D), reinterpret_cast<const GLvoid *>(0x0) );
    draw();
    _vbo.disable( vertice_loc );
    _vbo.unbind();

    _color_prog.unbind();
}





template <typename T, int n>
void PSurfCurvatureVisualizer<T,n>::update() {

    PSurfCurvatureVisualizer<T,n>::fillStandardVBO( _vbo, *(this->_p) );
    PSurfCurvatureVisualizer<T,n>::fillTriangleStripIBO( _ibo, this->_p->getDim1(), this->_p->getDim2(), _no_strips, _no_strip_indices, _strip_size );
    PSurfCurvatureVisualizer<T,n>::fillNMap( _nmap, *(this->_n), this->_closed[0], this->_closed[1] );
}



template <typename T, int n>
inline
    void PSurfCurvatureVisualizer<T,n>::draw() const {

    _ibo.bind();
    for( unsigned int i = 0; i < _no_strips; ++i )
        _ibo.drawElements( _mode, _no_strip_indices, GL_UNSIGNED_INT, reinterpret_cast<const GLvoid *>(i * _strip_size) );
    _ibo.unbind();
}



template<typename T,int n>
void PSurfCurvatureVisualizer<T,n>::initShaderProgram() {

    const std::string prog_name    = "psurf_default_prog";
    if( _prog.acquire(prog_name) ) return;


    std::string vs_src =
        GMlib::GL::OpenGLManager::glslDefHeaderVersionSource() +

        "uniform mat4 u_mvmat, u_mvpmat;\n"
        "\n"
        "in vec4 in_vertex;\n"
        "in vec2 in_tex;\n"
        "\n"
        "out vec4 gl_Position;\n"
        "\n"
        "smooth out vec3 ex_pos;\n"
        "smooth out vec2 ex_tex;\n"
        "\n"
        "void main() {\n"
        "\n"
        "  vec4 v_pos = u_mvmat * in_vertex;\n"
        "  ex_pos = v_pos.xyz * v_pos.w;\n"
        "\n"
        "  ex_tex = in_tex;\n"
        "\n"
        "  gl_Position = u_mvpmat * in_vertex;\n"
        "}\n"
        ;


    std::ifstream file("../../qmldemo-master/Visualizer/curvature_visualizer_string.txt");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << std::endl;
    }

    // Read the entire file content into a string
    std::string fs_string((std::istreambuf_iterator<char>(file)),
                          std::istreambuf_iterator<char>());

    file.close(); // Close the file stream

    std::string fs_src =
        GMlib::GL::OpenGLManager::glslDefHeaderVersionSource() + fs_string;


    // std::cout << fs_src << std::endl;
    bool compile_ok, link_ok;

    GMlib::GL::VertexShader vshader;
    vshader.create("psurf_default_vs");
    vshader.setPersistent(true);
    vshader.setSource(vs_src);
    compile_ok = vshader.compile();
    if( !compile_ok ) {
        std::cout << "Src:" << std::endl << vshader.getSource() << std::endl << std::endl;
        std::cout << "Error: " << vshader.getCompilerLog() << std::endl;
    }
    assert(compile_ok);

    GMlib::GL::FragmentShader fshader;
    fshader.create("psurf_default_fs");
    fshader.setPersistent(true);
    fshader.setSource(fs_src);
    compile_ok = fshader.compile();
    if( !compile_ok ) {
        std::cout << "Src:" << std::endl << fshader.getSource() << std::endl << std::endl;
        std::cout << "Error: " << fshader.getCompilerLog() << std::endl;
    }
    assert(compile_ok);

    _prog.create(prog_name);
    _prog.setPersistent(true);
    _prog.attachShader(vshader);
    _prog.attachShader(fshader);
    link_ok = _prog.link();
    if( !link_ok ) {
        std::cout << "Error: " << _prog.getLinkerLog() << std::endl;
    }
    assert(link_ok);
}


template <typename T, int n>
inline
    void PSurfCurvatureVisualizer<T,n>::_init() {

    initShaderProgram();

    _color_prog.acquire("color");
    assert(_color_prog.isValid());

    _vbo.create();
    _ibo.create();
    _nmap.create(GL_TEXTURE_2D);
}




#endif // CURVATURE_ANALYSIS_VISUALIZER_H
