#include "Scene.h"
#include "Config.h"
#include <iostream>
#include <filesystem>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"


Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
    if constexpr(DEBUG) {
        assert (ray.isNormalized());
    }
    if (bouncesLeft < 0) return {};

    // TODO...
    Intersection inter = getIntersection(ray);              //get intersection using helper function

    if(inter.happened){                                     //If ray intersects object:
    
        //8.1.1 - Indirect Radiance. First Ray, sampled randomly. Use Trace recursively, discardEmission if True.
        Vec3 Lo = Vec3();                                   //initialize direct radiance to 0. Stays 0 if discardEmission=True (8.1.1)
        
        if((discardEmission == false) || (bouncesLeft==0)){ 
            Lo = Lo + inter.object->ke;                     //return value. Term outside the integral/sampling
        }

        //first 'sampled' ray
        Ray next_ray = Ray(inter.pos, Random::cosWeightedHemisphere(inter.getNormal()));     //get sampled ray direction. importane sampling 
        Intersection inter_sampled = getIntersection(next_ray);

        //terms to calculate indirect lighting
        Vec3 fr = inter.calcBRDF(-next_ray.dir, -ray.dir);                                       //BRDF value
        float cosine_term = next_ray.dir.dot(inter.getNormal());                                 
        
        if(inter_sampled.happened){
            Vec3 Li = Vec3();                                                //Li is 0 vector if no bounces left. Base case
            float pdf = cosine_term/(PI);                                    //=1.0/(2*PI) for global ill
            if(bouncesLeft > 0){
                Li = trace(next_ray, bouncesLeft-1, true);                   //recursive call. bool set to true to discardEmission (8.1.1)
            }
            Lo = Lo + Li * fr * (cosine_term / pdf);                      //Li = emission of intersection point if intersects
        }

        //8.1.2 - Direct Radiance
        Intersection lightSample = sampleLight();
        Vec3 lightDir = lightSample.pos - inter.pos;
        float distanceToLight = lightDir.getLength();
        lightDir.normalize();
        Ray rayToLight(inter.pos, lightDir);        //set up shadow ray
        
        Intersection light_inter = getIntersection(rayToLight);     //check intersections and obstacles to light

        if(light_inter.happened && light_inter.object->hasEmission){       
            float pdfLightSample = 1.0 / lightArea;
            Vec3 light_emission = lightSample.object->ke;
            Vec3 fr_light = inter.calcBRDF(-rayToLight.dir, -ray.dir);
            float cos_term1 = rayToLight.dir.dot(inter.getNormal());
            float d_squared = distanceToLight * distanceToLight;
            float cos_term2 = rayToLight.dir.dot(-light_inter.getNormal()) / d_squared;
            Lo = Lo + light_emission*fr_light*(cos_term1*cos_term2 / pdfLightSample);
        }
        return Lo;
    }

    return {};                                      //deduces return type, default constructs object (zero vector; Vec3())
}


tinyobj::ObjReader Scene::reader {};

void Scene::addObjects(std::string_view modelPath, std::string_view searchPath) {
    tinyobj::ObjReaderConfig config;
    config.mtl_search_path = searchPath;
    if (!reader.ParseFromFile(std::string(modelPath), config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
            std::filesystem::path relative(modelPath);
            std::cerr << "Reading file " << std::filesystem::absolute(relative) << " error. File may be malformed or not exist.\n";
        }
        exit(1);
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        auto* object = new Object();
        object->name = shapes[s].name;
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            std::vector<Vec3> positions;
            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
                tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
                tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

                positions.push_back({vx, vy, vz});
            } // per-vertex
            index_offset += fv;
            Mesh mesh {positions[0], positions[1], positions[2]};
            object->area += mesh.area;
            object->meshes.push_back(std::move(mesh));
        } // per-face
        object->constructBoundingBox();
        // we assume each object uses only a single material for all meshes
        auto materialId = shapes[s].mesh.material_ids[0];
        auto& material = materials[materialId];
        object->kd = Vec3 {
            material.diffuse[0],
            material.diffuse[1],
            material.diffuse[2],
        };
        if (material.emission[0] + 
            material.emission[1] + 
            material.emission[2] > 0) { // is light
            object->ke = Vec3 {
                material.emission[0], 
                material.emission[1],
                material.emission[2]
            };
            object->hasEmission = true;
            lights.push_back(object);
            lightArea += object->area;
        }
        objects.push_back(object);
    } // per-shape
}

void Scene::constructBVH() {
    assert (!objects.empty());
    bvh.root = BVH::build(objects);
}

Intersection Scene::getIntersection(const Ray &ray) {
    assert (bvh.root);
    return bvh.root->intersect(ray);
}

Intersection Scene::sampleLight() const {
    assert (lights.size() == 1 && "Currently only support a single light object");
    assert (lightArea > 0.0f);
    Intersection inter;
    return lights[0]->sample();
}

Scene::~Scene() {
    for (auto obj : objects) {
        delete obj;
    }
}
