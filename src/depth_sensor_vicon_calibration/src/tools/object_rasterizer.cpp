#include "object_rasterizer.hpp"

#include <stdio.h>
#include <stdlib.h>
#include "shader.hpp"
#include <GL/glew.h>
#include <GL/glfw.h>
#include <GL/freeglut.h>
#include <iostream>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtc/matrix_integer.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "GL/glx.h"
#include "Eigen/Geometry"
#include <omp.h>


using namespace std;
using namespace Eigen;


ObjectRasterizer::ObjectRasterizer()
{
}

ObjectRasterizer::ObjectRasterizer(const std::vector<Eigen::Vector3f> vertices,
                                   const std::vector<std::vector<int> > indices) :
    n_rows_(WINDOW_HEIGHT),
    n_cols_(WINDOW_WIDTH),
    // TODO How do I make this relative? Have no idea what the current path is.
    vertex_shader_path_("/Network/Servers/boudin/Volumes/boudin/cpfreundt/ARM/mandy/perception/state_filter/src/shaders/VertexShader.vertexshader"),
    fragment_shader_path_("/Network/Servers/boudin/Volumes/boudin/cpfreundt/ARM/mandy/perception/state_filter/src/shaders/FragmentShader.fragmentshader"),
    vertices_(vertices),
    indices_(indices),
    projection_matrix_(Matrix4f::Identity()),
    view_matrix_(Matrix4f::Identity()),
    model_matrix_(Matrix4f::Identity()),
    mvp_matrix_(Matrix4f::Identity()),
    framebuffer_(0)
{
    // ========== FOR TESTING ===========

//        vertices_.resize(0);
//        vertices_.push_back(Vector3f(0.1, 0.1, 0));
//        vertices_.push_Fback(Vector3f(-0.1, 0.1, 0));
//        vertices_.push_back(Vector3f(0.1, -0.1, 0));

//        indices_list_.resize(0);
//        indices_list_.push_back(0);
//        indices_list_.push_back(1);
//        indices_list_.push_back(2);

    // ==================================

    // change indices format from vector<vector<int> > to vector<int>, because OpenGL wants a list
    // also changing type from int to uint, because indices are generally positive.
    for (int i = 0; i < indices_.size(); i++) {
        // we are assuming that indices_[0].size() equals indices_[1].size(), indices_[2].size() etc.
        for (int j = 0; j < indices_[0].size(); j++) {
            if (indices_[i][j] >= 0) {
                indices_list_.push_back(indices_[i][j]);
            } else {
                cout << "Error: Index number [" << i << "][" << j << "] is negative: " << indices_[i][j] << " and thus does not refer to a valid vertex!";
            }
        }
    }


    typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);
    typedef Bool (*glXMakeContextCurrentARBProc)(Display*, GLXDrawable, GLXDrawable, GLXContext);
    static glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
    static glXMakeContextCurrentARBProc glXMakeContextCurrentARB = 0;

    static int visual_attribs[] = {
       None
    };
    int context_attribs[] = {
           GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
           GLX_CONTEXT_MINOR_VERSION_ARB, 0,
           None
    };

    Display* dpy = XOpenDisplay(0);
    int fbcount = 0;
    GLXFBConfig* fbc = NULL;
    GLXContext ctx;
    GLXPbuffer pbuf;

    /* open display */
    if ( ! (dpy = XOpenDisplay(0)) ){
           fprintf(stderr, "Failed to open display\n");
           exit(1);
    }

    /* get framebuffer configs, any is usable (might want to add proper attribs) */
    if ( !(fbc = glXChooseFBConfig(dpy, DefaultScreen(dpy), visual_attribs, &fbcount) ) ){
           fprintf(stderr, "Failed to get FBConfig\n");
           exit(1);
    }

    /* get the required extensions */
    glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)glXGetProcAddressARB( (const GLubyte *) "glXCreateContextAttribsARB");
    glXMakeContextCurrentARB = (glXMakeContextCurrentARBProc)glXGetProcAddressARB( (const GLubyte *) "glXMakeContextCurrent");
    if ( !(glXCreateContextAttribsARB && glXMakeContextCurrentARB) ){
           fprintf(stderr, "missing support for GLX_ARB_create_context\n");
           XFree(fbc);
           exit(1);
    }

    /* create a context using glXCreateContextAttribsARB */
    if ( !( ctx = glXCreateContextAttribsARB(dpy, fbc[0], 0, True, context_attribs)) ){
           fprintf(stderr, "Failed to create opengl context\n");
           XFree(fbc);
           exit(1);
    }

    /* create temporary pbuffer */
    int pbuffer_attribs[] = {
           GLX_PBUFFER_WIDTH, WINDOW_WIDTH,
           GLX_PBUFFER_HEIGHT, WINDOW_HEIGHT,
           None
    };
    pbuf = glXCreatePbuffer(dpy, fbc[0], pbuffer_attribs);


    XFree(fbc);
    XSync(dpy, False);

    /* try to make it the current context */
    if ( !glXMakeContextCurrent(dpy, pbuf, pbuf, ctx) ){
           /* some drivers does not support context without default framebuffer, so fallback on
            * using the default window.
            */
           if ( !glXMakeContextCurrent(dpy, DefaultRootWindow(dpy), DefaultRootWindow(dpy), ctx) ){
                   fprintf(stderr, "failed to make current\n");
                   exit(1);
           }
    }

    /* try it out */
    printf("vendor: %s\n", (const char*)glGetString(GL_VENDOR));





    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        exit(EXIT_FAILURE);
    }

    // ======================== SET OPENGL OPTIONS ======================== //

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Disable color writes
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    // ====================== CREATE BUFFERS AND VBO ====================== //

    // creating a vertex array object (VBO)
    glGenVertexArrays(1, &vertex_array_ID_);
    glBindVertexArray(vertex_array_ID_);

    // create and fill vertex buffer with vertices_
    glGenBuffers(1, &vertex_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vector3f), &vertices_[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // create and fill index buffer with indices_
    glGenBuffers(1, &index_buffer_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_list_.size() * sizeof(uint), &indices_list_[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // create depth buffer and don't fill with anything
    glGenBuffers(1, &depth_buffer_);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, depth_buffer_);
    // the 0 means this buffer is uninitialized, since I only want to copy values back to the CPU that will be written by the GPU
    // TODO for performance: try GL_STREAM_READ instead of GL_DYNAMIC_READ
    glBufferData(GL_PIXEL_PACK_BUFFER, n_cols_ * n_rows_ * sizeof(GLfloat), NULL, GL_STREAM_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    // ======================= FRAMEBUFFER OBJECT ======================= //

    // create framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
    // We will add a texture which will only store the depth component.
    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    // The texture we're going to render the depth values to
    glGenTextures(1, &depth_texture_);
    glBindTexture(GL_TEXTURE_2D, depth_texture_);
    // Give an empty image to OpenGL ( the last "0" )
    glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT, WINDOW_WIDTH, WINDOW_HEIGHT, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_, 0);
    // tells OpenGL that we will not be using a color buffer
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    // Always check that our framebuffer is ok
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        cout << "Framebuffer incomplete!" << endl;
    }

    // ================= COMPILE SHADERS AND GET HANDLES ================= //

    // Create and compile our GLSL program from the shaders
    program_ID_ = LoadShaders(vertex_shader_path_, fragment_shader_path_);

    // Get a handle for our "MVP" uniform.
    matrix_ID_ = glGetUniformLocation(program_ID_, "MVP");

    // Use our shader
    glUseProgram(program_ID_);

    // ====================== PASS VALUES TO OpenGL ====================== //

    // 1rst attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
    glVertexAttribPointer(
        0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
        3,                  // size
        GL_FLOAT,           // type
        GL_FALSE,           // normalized?
        0,                  // stride
        (void*)0            // array buffer offset
    );

    // Index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_);
}




void ObjectRasterizer::GetMeasurement(const std::vector<float> state,
                                      const Eigen::Matrix3f camera_matrix,
                                      std::vector<int> &intersect_indices,
                                      std::vector<float> &depth) {
    VisualizeObject(state, camera_matrix);
    depth_transfer_start_ = omp_get_wtime();

    // ===================== READ VALUES FROM OpenGL ===================== //

    glBindBuffer(GL_PIXEL_PACK_BUFFER, depth_buffer_);
    glBindTexture(GL_TEXTURE_2D, depth_texture_);
//    GLfloat pixel_depth[int(WINDOW_WIDTH * WINDOW_HEIGHT)];
    glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

//    glReadPixels(0, 0, n_cols_, n_rows_, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    GLfloat *pixel_depth = (GLfloat*) glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);


    if (pixel_depth != (GLfloat*) NULL) {
//        cout << "Depth values of pixels: " << endl;

        depth.clear();
        intersect_indices.resize(0);
        vector<float> depth_image(n_rows_ * n_cols_, numeric_limits<float>::max());

        int offset = n_cols_ * (n_rows_ - 1);
        int index = 0;
        float z_b, z_n;
        for (int i = n_rows_ - 1; i >= 0; i--) {
            for (int j = 0; j < n_cols_; j++) {
                z_b = pixel_depth[offset + j];
                if (z_b != 1) {
                    z_n = 2.0 * z_b - 1.0;
                    index = ((n_rows_ - 1 - i) * n_cols_) + j;
                    intersect_indices.push_back(index);
                    depth_image[index] = 2.0 * KINECT_NEAR * KINECT_FAR / (KINECT_FAR + KINECT_NEAR - z_n * (KINECT_FAR - KINECT_NEAR));
//                    cout << "Claudi: prediction: " << depth_image[index] << endl;
                }
            }
//            cout << endl;
            offset -= n_cols_;
        }
        glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

        // fill the depths into the depth vector -------------------------------
        depth.resize(intersect_indices.size());
        for(int i = 0; i < int(intersect_indices.size()); i++) {
            depth[i] = depth_image[intersect_indices[i]];
        }
    } else {
        cout << "WARNING: Could not map Depth Buffer." << endl;
    }

    depth_transfer_stop_ = omp_get_wtime();
//    cout << "matrices: " << matrices_stop_ - matrices_start_ << ", rendering: " << rendering_stop_ - rendering_start_ << ", value transfer: " << depth_transfer_stop_ - depth_transfer_start_ << endl;

}



// Be sure to call SetResolution(width, height) before you call VisualizeObject(..), if you are not using the default windowsize
void ObjectRasterizer::VisualizeObject(const std::vector<float> state,
                                       const Eigen::Matrix3f camera_matrix)
{
    matrices_start_ = omp_get_wtime();

    // ==================== CONFIGURE IMAGE PARAMETERS ==================== //

    Eigen::Matrix3f camera_matrix_inverse = camera_matrix.inverse();

    Vector3f boundaries_min = camera_matrix_inverse * Vector3f(-0.5, -0.5, 1);
    Vector3f boundaries_max = camera_matrix_inverse * Vector3f(float(n_cols_)-0.5, float(n_rows_)-0.5, 1);

    float near = KINECT_NEAR;
    float far = KINECT_FAR;
    float left = near * boundaries_min(0);
    float right = near * boundaries_max(0);
    float top = -near * boundaries_min(1);
    float bottom = -near * boundaries_max(1);

    // ======================== CONFIGURE MATRICES ======================== //

    // =========================== MODEL MATRIX =========================== //

    // Model matrix equals the state of the object. Position and Quaternion just have to be expressed as a matrix.
    // note: state = (q.w,q.x,q.y.,q.z,v.x,v.y,v.z)
    model_matrix_ = Matrix4f::Identity();
    Transform<float, 3, Affine, ColMajor> model_matrix_transform;
    model_matrix_transform = Translation3f(state[4], state[5], state[6]);
    model_matrix_ = model_matrix_transform.matrix();

    Quaternionf qRotation = Quaternionf(state[0], state[1], state[2], state[3]);
    model_matrix_.topLeftCorner(3, 3) = qRotation.toRotationMatrix();

    // =========================== VIEW MATRIX =========================== //

    // OpenGL camera is rotated 180 degrees around the X-Axis compared to the Kinect camera
    view_matrix_ = Matrix4f::Identity();
    Transform<float, 3, Affine, ColMajor> view_matrix_transform;
    view_matrix_transform = AngleAxisf(M_PI, Vector3f::UnitX());
    view_matrix_ = view_matrix_transform.matrix();

    // ======================== PROJECTION MATRIX ======================== //

    // Projection Matrix takes into account our view frustum and projects the (3D)-scene onto a 2D image
    projection_matrix_ = GetProjectionMatrix(near, far, left, right, top, bottom);

    // =========================== MVP MATRIX ============================ //

    // Our ModelViewProjection : multiplication of our 3 matrices
    mvp_matrix_ = projection_matrix_ * view_matrix_ * model_matrix_;

    // Send our transformation to the currently bound shader, in the "MVP" uniform
    // For each model you render, since the MVP will be different (at least the M part)
    // mvp_matrix_.data() transforms Eigen Matrix into a datastream that GLSL can understand as glm::mat4
    glUniformMatrix4fv(matrix_ID_, 1, GL_FALSE, mvp_matrix_.data());

    matrices_stop_ = omp_get_wtime();

    DrawObject();
}


/* Draw the object stored in vertices_ and indices_ with the current matrix mvp_matrix_ */
void ObjectRasterizer::DrawObject() {
    rendering_start_ = omp_get_wtime();

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    // Clear the depth buffer
    glClear(GL_DEPTH_BUFFER_BIT);

    // ===================== RASTERIZE THE TRIANGLES ===================== //

    glDrawElements(
        GL_TRIANGLES,      // mode
        indices_list_.size(),    // count
        GL_UNSIGNED_INT,       // type
        (void*)0           // element array buffer offset
    );

    // one buffer is used for rendering to, the other buffer is used for displaying the current image
    // you have to swap them each time to keep it fluent
//    glXSwapBuffers();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    rendering_stop_ = omp_get_wtime();
}

void ObjectRasterizer::SetResolution(const int n_rows,
                                     const int n_cols) {
    n_rows_ = n_rows;
    n_cols_ = n_cols;

//    glfwSetWindowSize(n_colFs_, n_rows_);
    glViewport(0, 0, n_cols_, n_rows_);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, depth_buffer_);
    // the 0 means this buffer is uninitialized, since I only want to copy values back to the CPU that will be written by the GPU
    // TODO for performance: try GL_STREAM_READ instead of GL_DYNAMIC_READ
    glBufferData(GL_PIXEL_PACK_BUFFER, n_cols_ * n_rows_ * sizeof(GLfloat), NULL, GL_DYNAMIC_READ);

    // Clear the screen
    glClear(GL_DEPTH_BUFFER_BIT);
}


Matrix4f ObjectRasterizer::GetProjectionMatrix(float n, float f, float l, float r, float t, float b) {
    Matrix4f projection_matrix = Matrix4f::Zero();
    projection_matrix(0,0) = 2 * n / (r - l);
    projection_matrix(0,2) = (r + l) / (r - l);
    projection_matrix(1,1) = 2 * n / (t - b);
    projection_matrix(1,2) = (t + b) / (t - b);
    projection_matrix(2,2) = -(f + n) / (f - n);
    projection_matrix(2,3) = - 2 * f * n / (f - n);
    projection_matrix(3,2) = -1;

    return projection_matrix;
}

ObjectRasterizer::~ObjectRasterizer()
{
    cout << "clean up OpenGL.." << endl;

    glDisableVertexAttribArray(0);

    // Cleanup VBO
    glDeleteBuffers(1, &vertex_buffer_);
    glDeleteBuffers(1, &index_buffer_);
    glDeleteBuffers(1, &depth_buffer_);
    glDeleteVertexArrays(1, &vertex_array_ID_);
    glDeleteProgram(program_ID_);

    // Close OpenGL window and terminate GLFW
//    glfwTerminate();
}


