//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Intersection.hpp"
#include "Material.hpp"


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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
/* shade (p, wo)
    sampleLight ( inter , pdf_light )
    Get x, ws , NN , emit from inter
    Shoot a ray from p to x
    If the ray is not blocked in the middle
        L_dir = emit * eval (wo , ws , N) * dot (ws , N) * dot (ws ,NN) / |x-p |^2 / pdf_light

    L_indir = 0.0
    Test Russian Roulette with probability RussianRoulette
    wi = sample (wo , N)
    Trace a ray r(p, wi)
    If ray r hit a non - emitting object at q
        L_indir = shade (q, wi) * eval (wo , wi , N) * dot (wi , N)/ pdf (wo , wi , N) / RussianRoulette
    Return L_dir + L_indir
 */
    // Contribute from the light source
    Vector3f dir = {0.0f,0.0f,0.0f};
    Vector3f indir = {0.0f,0.0f,0.0f};
    // 超过最大次数，跳出
    if(depth > maxDepth)return dir;
    Intersection inter = Scene::intersect(ray);
    // 1. 没打到任何物体，黑色
    if(!inter.happened)return dir;
    // 2. 打到光源，返回材质发光项
    if(inter.m->hasEmission()){
        if(depth==0){
            return inter.m->m_emission;
        }else{
            return dir; // 光源反射的其他光不要
        }
    }
    
    // 3. 打到物体
    // 采样光源
    Intersection lightpos;
    float pdf_light=0;
    sampleLight(lightpos, pdf_light);

    // 3.1 直接光照
    // 物体参数
    Vector3f& p = inter.coords;
    Vector3f N = inter.normal.normalized();
    Vector3f wo = ray.direction; // 物体到场景方向
    
    // 光源参数
    Vector3f& xx= lightpos.coords;
    Vector3f NN= lightpos.normal.normalized();
    Vector3f ws= (p-xx).normalized();// 光源指向物体
    float dis = (p-xx).norm();
    float dis2= dotProduct((p-xx), (p-xx));// 用的是第二范式
    Ray light_to_obj(xx,ws);
    Intersection light_to_scene = Scene::intersect(light_to_obj);
    // 对light_to_obj进行去交，看从obj到物体的交点距离是否小于到光源，是则表示被遮挡
    if(light_to_scene.happened && (light_to_scene.distance - dis)>-100*EPSILON){
        Vector3f& emit = lightpos.emit;
        Vector3f f_r = inter.m->eval(wo,-ws,N);
        float cos_theta = dotProduct(N, -ws);
        float cos_theta_p = dotProduct(NN,ws);
        Vector3f& xp = lightpos.coords;
        dir = emit * f_r * cos_theta * cos_theta_p / dis2 / pdf_light;
    }

    // 间接光照
    if(get_random_float()<RussianRoulette){
        auto wi = inter.m->sample(wo, N).normalized();
        Ray r(p,wi);
        Intersection r_inter = Scene::intersect(r);
        if(r_inter.happened && !r_inter.m->hasEmission()){
            Vector3f f_r = inter.m->eval(wo, wi, N);
            float cos_theta = dotProduct(wi, N);
            float pdf = r_inter.m->pdf(wo, wi, N);
            indir = castRay(r, depth+1) * f_r * cos_theta / pdf / RussianRoulette;
        }
    }

    return dir+indir;
}