//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}
// 利用bvh得到光线与最近物体的求交
Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}
// 对场景中的光源按面经进行平均随机采样
// 参数需要填入光线与光源的交点pos和pdf
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    // 对在场景的所有光源上按面积均匀sample一个点，并计算该sample的概率密度也就是1/A(因为均匀)
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
    // 计算直接光照部分
    Vector3f L_dir (0, 0, 0);
    // 从像素射出光线作为着色点的入射光线，获得渲染方程中入射光线与着色点相交的各项参数
    Intersection inter_obj = intersect(ray);
    if (!inter_obj.happened) // 需先判断从像素采样的该光纤是否有射到物体
        return L_dir;
    if (inter_obj.m->hasEmission()) // 如果追踪到光源则直接返回光源，保证光源是亮的
        return inter_obj.m->getEmission();
    Vector3f p = inter_obj.coords;
    Vector3f N = inter_obj.normal.normalized(); // 法线需要计算cos，所以需要单位化
    Vector3f wo = ray.direction;   // 由于框架原因，这里的wo为像素到着色点的方向，与着色点向外的方向相反
    // 计算反射光线与光源相交，为提高采样效率所以从光源出发采样
    Intersection inter_light;
    float pdf_light;
    sampleLight(inter_light, pdf_light); // 对光源按面积采样,直接填入默认参数，得到着色点与光源的交点
    Vector3f x = inter_light.coords;
    Vector3f ws = (x - p).normalized(); // 从物体指向光源

    // 从着色点射出光线，与着色点反射的光线比较，判断着色点反射的光线是否能射到光源
    Ray objTolight(p, ws);
    float d1 = (x - p).norm(); // 着色点与光源距离
    float d2 = intersect(objTolight).distance; // 着色点与相交物体距离
    // 计算L_dir直接光照
    if (d2 - d1 > -0.001) { // 做差判断浮点数是否相等
        Vector3f eval = inter_obj.m->eval(wo, ws, N); // 注意参数的方向，需要入射方向、出射方向
        Vector3f emit = inter_light.emit;   // 光源投射到着色点的光线的radiance
        Vector3f NN = inter_light.normal.normalized();
        float cos_theta_obj = dotProduct(N, ws);
        float cos_theta_light = dotProduct(NN, -ws); // 这里要取负号，因为ws是从着色点指向光源，与NN夹角大于90度为负
        L_dir = emit * eval * cos_theta_obj * cos_theta_light / std::pow(d1, 2) / pdf_light;
    }

    // 递归计算间接光照部分
    Vector3f L_indir (0, 0, 0);
    float P_RR = get_random_float(); // 采用俄罗斯轮盘赌，避免无穷递归，最终数学期望一样
    if (P_RR < RussianRoulette) {
        Vector3f wi = inter_obj.m->sample(wo, N).normalized(); // 对着色点随机采样出射光线
        Ray ray_objToobj(p, wi); // 根据采样获得从着色点射向其他物体的光线
        Intersection inter = intersect(ray_objToobj);
        if (inter.happened && !inter.m->hasEmission()) { // 如果有交点且交点材质不发光
            Vector3f eval = inter_obj.m->eval(wo, wi, N); // 计算着色点的eval,而不是反射光线与其他物体交点的
            float pdf = inter_obj.m->pdf(wo, wi, N); // 计算着色点采样的pdf
            float cos_theta = dotProduct(wi, N);
            L_indir = castRay(ray_objToobj, depth + 1) * eval * cos_theta / pdf / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}

/*
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir;
    Vector3f L_indir;
 
    // 从像素发出的光线与物体的交点
    Intersection obj_inter = intersect(ray);
    if(!obj_inter.happened)
        return L_dir;
 
    // 打到光源
    if(obj_inter.m->hasEmission())
        return obj_inter.m->getEmission();
 
    // 打到物体
    Vector3f p = obj_inter.coords;
    Material* m = obj_inter.m;
    Vector3f N = obj_inter.normal.normalized();
    Vector3f wo = ray.direction; // 由于框架原因，这里的wo为像素到着色点的方向，与着色点向外的方向相反
    
    // 有交点，对光源采样
    float pdf_L = 1.0; //可以不初始化
    Intersection light_inter ;
    sampleLight(light_inter,pdf_L);    // 得到光源位置和对光源采样的pdf
    
    Vector3f x = light_inter.coords;
    Vector3f ws = (x - p).normalized(); //物体到光源
    Vector3f NN = light_inter.normal.normalized();  
    Vector3f emit = light_inter.emit;
    float d = (x-p).norm();
    
    // 再次从光源发出一条光线，判断是否能打到该物体，即中间是否有阻挡
    Ray Obj2Light(p,ws);
    float d2 = intersect(Obj2Light).distance;
    // 是否阻挡，利用距离判断，需注意浮点数的处理
    if(d2-d > -0.001){
        Vector3f eval = m->eval(wo,ws,N); // wo不会用到
        float cos_theta = dotProduct(N,ws);
        float cos_theta_x = dotProduct(NN,-ws);//ws从物体指向光源，与NN的夹角大于180
        L_dir = emit * eval * cos_theta * cos_theta_x / std::pow(d,2) / pdf_L;
    }
    
    // L_indir
    float P_RR = get_random_float();
    if(P_RR < RussianRoulette){
        Vector3f wi = m->sample(wo,N).normalized();
        Ray r(p,wi);
        Intersection inter = intersect(r);
        // 判断打到的物体是否会发光取决于m
        if(inter.happened && !inter.m->hasEmission()){
            Vector3f eval = m->eval(wo,wi,N);
            float pdf_O = m->pdf(wo,wi,N);
            float cos_theta = dotProduct(wi,N);
            L_indir = castRay(r, depth+1) * eval * cos_theta/ pdf_O / RussianRoulette;
        }
    }
    //4->16min
    return L_dir + L_indir;
}*/