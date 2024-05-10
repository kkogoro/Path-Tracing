#include "Labs/3-Rendering/tasks.h"
#include <iostream>
#include <algorithm>
#include <random>
#define M_PI 3.141592653589793f
namespace VCX::Labs::Rendering {

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        // your code here
        auto E1 = p2 - p1;
        auto E2 = p3 - p1;
        auto S  = ray.Origin - p1;
        auto S1 = cross(ray.Direction, E2);
        auto S2 = cross(S, E1);
        auto div = dot(S1, E1);
        auto t = dot(S2, E2) / div;
        auto b1 = dot(S1, S) / div;
        auto b2 = dot(S2, ray.Direction) / div;
        if (0.0 <= b1 && 0.0 <= b2 && b1 + b2 <= 1.0) {
            output.t = t;
            output.u = b1;
            output.v = b2;
            return true;
        }
        return false;
    }

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);

        for (int depth = 0; depth < maxDepth; depth++) {
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = rayHit.IntersectNormal;
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;

            glm::vec3 result(0.0f);
            /******************* 2. Whitted-style ray tracing *****************/
            // your code here
            result = intersector.InternalScene->AmbientIntensity * kd;
            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation;
                /******************* 3. Shadow ray *****************/
                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    attenuation = 1.0f / glm::dot(l, l);
                    if (enableShadow) {
                        // your code here
                        auto hit = intersector.IntersectShadowRay(Ray(pos, l), true);
                        //while (hit.IntersectState && hit.IntersectAlbedo.w < 0.2) hit = intersector.IntersectRay(Ray(hit.IntersectPosition, normalize(l)));
                        if (hit.IntersectState && glm::length(hit.IntersectPosition - pos) < glm::length(l)) continue;
                    }
                } else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    attenuation = 1.0f;
                    if (enableShadow) {
                        // your code here
                        auto hit = intersector.IntersectShadowRay(Ray(pos, glm::normalize(l)), false);
                        //while (hit.IntersectState && hit.IntersectAlbedo.w < 0.2) hit = intersector.IntersectRay(Ray(hit.IntersectPosition, normalize(l)));
                        if (hit.IntersectState) continue;
                    }
                }

                /******************* 2. Whitted-style ray tracing *****************/
                // your code here
                glm::vec3 lightDir = glm::normalize(l);
                glm::vec3 viewDir  = glm::normalize(-ray.Direction);
                glm::vec3 lightIntensity = light.Intensity * attenuation;
                glm::vec3 normal = glm::normalize(n);
                result += kd * lightIntensity * std::max(0.0f,  glm::dot(normal, lightDir)) + 
                            ks * lightIntensity * 
                            pow(std::max(0.0f, glm::dot(normal, glm::normalize((lightDir + viewDir) / 2.0f)) ), shininess);
                           

            }

            if (alpha < 0.9) {
                // refraction
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;

                // generate new ray
                ray = Ray(pos, ray.Direction);
            } else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;

                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos, glm::normalize(out_dir));
            }
        }
        return color;
    }
    inline float get_random_float() {
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_real_distribution<float> dist(0.f, 1.f);
        return dist(rng);
    }
    glm::vec3 SampleLight(const Engine::Light & light, float &pdf) {
        glm::vec3 ret(0.0f);
        if (light.Type == Engine::LightType::Spot) {
            float theta = 2.0 * M_PI * get_random_float(), phi = M_PI * get_random_float();
            glm::vec3 dir(std::cos(phi), std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta));
            ret = light.Position + light.Length * dir;
            pdf = 1.0f / (4 * M_PI * light.Length * light.Length); //S      
        }
        else { //Triangle Area Light
            float x = std::sqrt(get_random_float()), y = get_random_float();
            ret = light.p[0] * (1.0f - x) + light.p[1] * (x * (1.0f - y)) + light.p[2] * (x * y);
            pdf = 1.0f / (glm::length(glm::cross(light.p[0] - light.p[1], light.p[2] - light.p[1])) / 2);
        }
        return ret;
    }
    glm::vec3 f_r(const glm::vec3 &wo, const glm::vec3 &wi, const VCX::Labs::Rendering::RayHit &hit) {
        const glm::vec3 n = glm::normalize( hit.IntersectNormal );
        const glm::vec3 kd = hit.IntersectAlbedo;
        if (glm::dot(n, wo) > 0.0f) 
            return kd / M_PI;                    //Kd / M_PI;
        return glm::vec3(0.0f);
    }
    float get_pdf(const glm::vec3 &wo, const glm::vec3 &wi, const VCX::Labs::Rendering::RayHit &hit) {
        const glm::vec3 n = glm::normalize( hit.IntersectNormal );
        if (glm::dot(n, wo) > 0.0f)
            return 0.5f / M_PI;
        return 0.0f;
    }
    glm::vec3 SampleRay(const glm::vec3 &wi, const glm::vec3 &N) {
        float x_1 = get_random_float(), x_2 = get_random_float();
        float z = std::fabs(1.0f - 2.0f * x_1);
        float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
        glm::vec3 localRay(r*std::cos(phi), r*std::sin(phi), z);
                glm::vec3 B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = glm::vec3(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = glm::vec3(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = glm::cross(C, N);
        return localRay.x * B + localRay.y * C + localRay.z * N;
    }

    const float RussianRoulette = 0.8;

    glm::vec3 PathTrace(const RayIntersector & intersector, const VCX::Labs::Rendering::RayHit &hit_p, const Ray &ray) {
        //若击中光源则返回光源辐射IntersectEmission
        if (hit_p.IntersectEmission.r > 0.0f) return hit_p.IntersectEmission; 
        glm::vec3 l_directly(0.0f);
        const glm::vec3 n = glm::normalize( hit_p.IntersectNormal );
        glm::vec3 p = hit_p.IntersectPosition;

        float area_sum = 0;
        for (const Engine::Light & light : intersector.InternalScene->Lights) {
            area_sum += (glm::length(glm::cross(light.p[0] - light.p[1], light.p[2] - light.p[1])) / 2);
        }
        float limit = get_random_float() * area_sum;
        area_sum = 0;
        for (const Engine::Light & light : intersector.InternalScene->Lights) {
            if (light.Type != Engine::LightType::Area) continue;
            area_sum += (glm::length(glm::cross(light.p[0] - light.p[1], light.p[2] - light.p[1])) / 2);
            if (limit <= area_sum) {
                float pdf = 0.0f;
                glm::vec3 q =  SampleLight(light, pdf);
                glm::vec3 wi = q - p; //to be normalized
                auto hit = intersector.IntersectRay(Ray(p, wi));
                if (hit.IntersectState && glm::length(wi) - glm::length(hit.IntersectPosition - p) > 1e-3) continue;
                wi = glm::normalize(q - p);
                l_directly += light.Intensity / glm::dot(q - p, q - p) / pdf * f_r(wi, -ray.Direction, hit_p)
                                * std::max(0.0f, glm::dot(n, wi)) * std::max(0.0f, glm::dot(-wi, light.Direction))
                                ;
                break;
            }
        }
        if (get_random_float() > RussianRoulette) return l_directly;
        glm::vec3 l_indirectly(0.0f);
        glm::vec3 wi = glm::normalize( SampleRay(-ray.Direction, n) );
        Ray r(p, wi);
        auto hit = intersector.IntersectRay(r);
        if (hit.IntersectState && hit.IntersectEmission.r <= 0.0f) {
            l_indirectly = 5.0f * f_r(wi, -ray.Direction, hit_p) * PathTrace(intersector, hit, r) * std::max(0.0f, glm::dot(n, wi)) / RussianRoulette / get_pdf( wi, ray.Direction, hit_p);
        }
        return l_directly + l_indirectly;
    }
} // namespace VCX::Labs::Rendering