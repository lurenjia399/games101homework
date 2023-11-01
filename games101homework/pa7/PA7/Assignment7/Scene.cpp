//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir = Vector3f(0.f); // 直接光照的颜色

    Intersection point_Intersection = intersect(ray);
    if (!point_Intersection.happened) return L_dir;

    if (point_Intersection.m->hasEmission()) return point_Intersection.m->getEmission();

    Intersection light_Intersection;
    float light_pdf;
    sampleLight(light_Intersection, light_pdf);

    Vector3f p_to_light_dir = (light_Intersection.coords - point_Intersection.coords).normalized();
    Ray p_to_light(point_Intersection.coords, p_to_light_dir, (light_Intersection.coords - point_Intersection.coords).x / p_to_light_dir.x);
    // 如果p 到 light 没有object 阻挡
    Intersection block_object_Intersection = intersect(p_to_light);
    if ((point_Intersection.coords - light_Intersection.coords).norm() - block_object_Intersection.distance < 0.001)
    {
        L_dir = light_Intersection.emit
            * point_Intersection.m->eval(Vector3f(), p_to_light_dir, point_Intersection.normal)
            * dotProduct(p_to_light_dir, point_Intersection.normal)
            * dotProduct(-p_to_light_dir, light_Intersection.normal)
            / std::pow((light_Intersection.coords - point_Intersection.coords).norm(), 2)
            / light_pdf;
    }


    Vector3f L_indir = Vector3f(0.f); // 间接光照的颜色
    if (get_random_float() < RussianRoulette)
    {
        Vector3f p_to_object_dir = point_Intersection.m->sample(-ray.direction, point_Intersection.normal).normalized();
        Ray p_to_object = Ray(point_Intersection.coords, p_to_object_dir); // p点 发射出的任意方向射线
        Intersection object_Intersection = intersect(p_to_object);
        if (object_Intersection.happened && !object_Intersection.obj->hasEmit())
        {   // 打中object
            L_indir = castRay(p_to_object, 0)
                * point_Intersection.m->eval(Vector3f(), p_to_object_dir, point_Intersection.normal)
                * dotProduct(p_to_object_dir, point_Intersection.normal)
                / point_Intersection.m->pdf(Vector3f(), p_to_object_dir, point_Intersection.normal)
                / RussianRoulette;
        }
    }


    return L_dir + L_indir;
}