#include <cstdint>

#include "image.hpp"
#include "loader.hpp"
#include "rasterizer.hpp"

#include <iostream>                             //ADDED for std::cout (debugging)
#include "../thirdparty/glm/gtc/quaternion.hpp" //ADDED for toMat4 function --> not working

//ADDED helper function point_in_triangle. For use only in this file, not declared in rasterizer.hpp
static bool point_in_triangle(float x, float y, Triangle trig)
{
    glm::vec4 trig_p1 = trig.pos[0];             //triangle point A1 - extract points from triangle
    glm::vec4 trig_p2 = trig.pos[1];             //A2
    glm::vec4 trig_p3 = trig.pos[2];             //A3
    //calculate cross products Si. Formula--> (u1,u2)x(v1,v2) = (u1*v2) - (u2*v1)
    float S1 = (x - trig_p1.x)*(trig_p2.y - trig_p1.y) - (y - trig_p1.y)*(trig_p2.x - trig_p1.x);  //S1 = (X − A1)x(A2 − A1)
    float S2 = (x - trig_p2.x)*(trig_p3.y - trig_p2.y) - (y - trig_p2.y)*(trig_p3.x - trig_p2.x);  //S1 = (X − A1)x(A2 − A1)
    float S3 = (x - trig_p3.x)*(trig_p1.y - trig_p3.y) - (y - trig_p3.y)*(trig_p1.x - trig_p3.x);  //S1 = (X − A1)x(A2 − A1)
    //if S1, S2, S2 have the same sign (0 means point is on triangle boundary), then point is inside triangle:
    if( (S1<=0 && S2<=0 && S3<=0) || (S1>=0 && S2>=0 && S3>=0) ){
        return true;
    }
    else{return false;}
}


// TODO
void Rasterizer::DrawPixel(uint32_t x, uint32_t y, Triangle trig, AntiAliasConfig config, uint32_t spp, Image& image, Color color)
{
 
    if (config == AntiAliasConfig::NONE)             // if anti-aliasing is off
    {
        //If the pixel is inside the triangle trig:  
        if(point_in_triangle(float(x), float(y), trig) == true){ 
            image.Set(x, y, color);                 //write to image with given color
        }

    }
    else if (config == AntiAliasConfig::SSAA)       // if anti-aliasing is on
    {

        //generate sample points based on spp
        float count = 0;                           //total number of sub-pixels on triangle
        for(float i = 0; i < spp; i++){
            for(float j = 0; j < spp; j++){        //double for loop: for each sub pixel in the pixel, based on spp
                float sub_pixel_x = x + i/spp;
                float sub_pixel_y = y + j/spp;

                //if sample point is inside the traingle
                if(point_in_triangle(sub_pixel_x, sub_pixel_y, trig) == true){
                    count++;                       
                }
            }
        }
        Color avg_color = color*(count/(spp*spp));  //find avg color over cout and spp^2 (total number of samples)
        image.Set(x, y, avg_color);                 //write to image with avg color
    }

    // if the pixel is inside the triangle
    // image.Set(x, y, color);
    return;
}


// TODO
void Rasterizer::AddModel(MeshTransform transform, glm::mat4 rotation)
{
    /* model.push_back( model transformation constructed from translation, rotation and scale );*/
    glm::vec3 translation = transform.translation;
    glm::vec3 scale = transform.scale;

    glm::mat4 M_trans = glm::mat4(                          //build as a transpose since its glm 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        translation.x, translation.y, translation.z, 1
    );

    glm::mat4 M_scale = glm::mat4(                          //built as the transpose
        scale.x, 0, 0, 0,
        0, scale.y, 0, 0,
        0, 0, scale.z, 0,
        0, 0, 0,       1
    );

    glm::mat4 M_model = M_trans * rotation * M_scale;
    this->model.push_back(M_model);                         //push_back model

    return;
}



// TODO
void Rasterizer::SetView()
{
    const Camera& camera = this->loader.GetCamera();
    glm::vec3 cameraPos = camera.pos;
    glm::vec3 cameraLookAt = (camera.lookAt);

    // TODO change this line to the correct view matrix
    // this->view = glm::mat4(1.);

    // glm::vec3 cameraLookAt = glm::normalize(cameraLookAt);        //normalize
    glm::vec3 cameraUp = glm::normalize(camera.up);                  //get camera up vector, normalize

    glm::vec3 camerLookAt_vec = glm::normalize(cameraLookAt - cameraPos);   //get actual vector

    glm::mat4 T_view = glm::mat4(                           //build T_view (transposed for glm)
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        -cameraPos.x, -cameraPos.y, -cameraPos.z, 1
    );
    
    glm::vec3 X = glm::normalize(glm::cross(camerLookAt_vec, cameraUp));   //cross product of g_hat x t_hat (lookUp x lookAt) = X (from specs);

    glm::mat4 R_view = glm::mat4(                           //build R_view from R_view_inv, inverse = transpose. Built as 
        X.x, cameraUp.x, -camerLookAt_vec.x, 0,
        X.y, cameraUp.y, -camerLookAt_vec.y, 0,
        X.z, cameraUp.z, -camerLookAt_vec.z, 0,
        0,            0,               0, 1
    );                                                      //built as transpose (since glm), so typed out R_view_inv

    glm::mat4 M_view = R_view * T_view;
    this->view = M_view;

    return;
}


// TODO
void Rasterizer::SetProjection()
{
    const Camera& camera = this->loader.GetCamera();

    float nearClip = camera.nearClip;                       // near clipping distance, strictly positive
    float farClip = camera.farClip;                         // far clipping distance, strictly positive
    
    // float width = this->loader.GetWidth();
    // float height = this->loader.GetHeight();             //ignored
    
    // TODO change this line to the correct projection matrix
    // this->projection = glm::mat4(1.);

    float width = camera.width;                             //MODIFIED to be camera's properties, not loaders.
    float height = camera.height;
    glm::vec3 cameraPos = camera.pos;                       //get camera position

    //M_ortho = M_ortho_scale * M_ortho_trans
    glm::mat4 M_ortho_trans = glm::mat4(
        1, 0, 0, 0,                                         //frustom is already centered on z axis, no x/y trans
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, (nearClip+farClip)/2, 1
    );

    glm::mat4 M_ortho_scale = glm::mat4(
        2.0/width, 0, 0, 0,
        0, 2.0/height, 0, 0,
        0, 0, 2.0/(farClip-nearClip), 0,                    //f > n here.
        0, 0, 0, 1
    );
    
    glm::mat4 M_ortho = M_ortho_scale * M_ortho_trans;      //creat M_ortho
 
    glm::mat4 M_persp = glm::mat4(                          //create M_persp
      -nearClip, 0, 0, 0,
      0, -nearClip, 0, 0,                                   //negative nearClips for x and y, since n > 0.
      0, 0, (-nearClip-farClip), 1,
      0, 0, -nearClip*farClip, 0  
    );

    glm::mat4 M_proj = M_ortho * M_persp;                   //create M_proj 
    this->projection = M_proj;

    return;
}


// TODO
void Rasterizer::SetScreenSpace()
{
    float width = this->loader.GetWidth();
    float height = this->loader.GetHeight();

    // TODO change this line to the correct screenspace matrix
    // this->screenspace = glm::mat4(1.);

    glm::mat4 Mss_scale = glm::mat4(  //scale by width and height of canvas, preserve z value (=1)
        width/2, 0, 0, 0,
        0, height/2, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    );

    glm::mat4 Mss_trans = glm::mat4(  //need to transalte NDC origin to center of screen
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        width/2, height/2, 0, 1
    );

    this->screenspace = Mss_trans * Mss_scale;

    return;
}



// TODO
glm::vec3 Rasterizer::BarycentricCoordinate(glm::vec2 pos, Triangle trig)
{

    glm::vec3 bc = glm::vec3(0.0, 0.0, 0.0); //bc = Barycentric Coordinate. Initialized as all 0s

    //formula from solving system of equations--> (pos.x; pos.y; 0)= bc[0] ∗ v1.pos + bc[1] ∗ v2.pos + bc[2] ∗ v3.pos   
    bc[0] = ((pos.y - trig.pos[1].y)*(trig.pos[2].x - trig.pos[1].x) - (pos.x - trig.pos[1].x)*(trig.pos[2].y - trig.pos[1].y)) / ((trig.pos[0].y - trig.pos[1].y)*(trig.pos[2].x - trig.pos[1].x) - (trig.pos[0].x - trig.pos[1].x)*(trig.pos[2].y - trig.pos[1].y));
    bc[1] = ((pos.y - trig.pos[2].y)*(trig.pos[0].x - trig.pos[2].x) - (pos.x - trig.pos[2].x)*(trig.pos[0].y - trig.pos[2].y)) / ((trig.pos[1].y - trig.pos[2].y)*(trig.pos[0].x - trig.pos[2].x) - (trig.pos[1].x - trig.pos[2].x)*(trig.pos[0].y - trig.pos[2].y));
    bc[2] = 1.0 - bc[0] - bc[1];

    return bc;                                 // return glm::vec3();
}


// TODO
// float Rasterizer::zBufferDefault = float();
float Rasterizer::zBufferDefault = -1.0f;  //initialized to -1, limit of canonical plane


// TODO
void Rasterizer::UpdateDepthAtPixel(uint32_t x, uint32_t y, Triangle original, Triangle transformed, ImageGrey& ZBuffer)
{

    //interpolate depth (z) value of x,y using barycentric coordinates
    glm::vec3 barycentricCoords = Rasterizer::BarycentricCoordinate(glm::vec2(x, y), transformed);  //transformed or original?
    float interpolated_depth = (barycentricCoords[0]*transformed.pos[0].z + barycentricCoords[1]*transformed.pos[1].z + barycentricCoords[2]*transformed.pos[2].z);

    //update if depth > Zbuffer AND if point (x,y) is in triangle
    if( (interpolated_depth > ZBuffer.Get(x, y)) && (point_in_triangle(float(x), float(y), transformed))){
        ZBuffer.Set(x, y, interpolated_depth);
    }


    return;
}


// TODO
void Rasterizer::ShadeAtPixel(uint32_t x, uint32_t y, Triangle original, Triangle transformed, Image& image)
{

    const std::vector<Light>& lights = this->loader.GetLights();    //get Lights from Loader
    const Camera& camera = this->loader.GetCamera();                //get Camera

    glm::vec3 barycentricCoords = Rasterizer::BarycentricCoordinate(glm::vec2(x, y), transformed);  //transformed or original?
    float interpolated_depth = barycentricCoords[0]*transformed.pos[0].z + barycentricCoords[1]*transformed.pos[1].z + barycentricCoords[2]*transformed.pos[2].z;

    if((interpolated_depth == this->ZBuffer.Get(x, y)) && point_in_triangle(float(x), float(y), transformed) ){  //if interpolated depth value is exactly the value in Zbuffer:

        //compute position of depth in world space:
        float depth_world = barycentricCoords[0]*original.pos[0].z + barycentricCoords[1]*original.pos[1].z + barycentricCoords[2]*original.pos[2].z;

        //retrieve its normal using Bary coords:
        glm::vec3 interpolated_normal = (barycentricCoords[0]*original.normal[0] + barycentricCoords[1]*original.normal[1] + barycentricCoords[2]*original.normal[2]);
        interpolated_normal = glm::normalize(interpolated_normal);
        float e = this->loader.GetSpecularExponent();

        //object's interpolated position in world space
        glm::vec3 pos_world = barycentricCoords[0]*original.pos[0] + barycentricCoords[1]*original.pos[1] + barycentricCoords[2]*original.pos[2];

        //do shading:
        Color final = this->loader.GetAmbientColor();         //start with only ambient light

        for(size_t i = 0; i < lights.size(); i++){
            Color LightColor = lights[i].color;
            float LightIntensity = lights[i].intensity;
            float r = glm::length(lights[i].pos - pos_world);
            glm::vec3 l = glm::normalize(lights[i].pos - pos_world);

            Color L_d = (LightIntensity*std::max(0.0f, glm::dot(l, interpolated_normal) ) / (r*r)) * LightColor; //Diffuse

            glm::vec3 v = glm::normalize(camera.pos - pos_world);
            glm::vec3 h = (0.5f*(l + v));

            Color L_s = (LightIntensity*std::pow( std::max(0.0f, glm::dot(interpolated_normal, h)), e ) / (r*r)) * LightColor;    //Specular

            final = final + L_d + L_s;
        }

        image.Set(x, y, final);

    }

    return;
}
