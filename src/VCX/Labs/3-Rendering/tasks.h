#pragma once

#include <numeric>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Engine/Scene.h"
#include "Labs/3-Rendering/Ray.h"

namespace VCX::Labs::Rendering {
 
    constexpr float EPS1 = 1e-2f; // distance to prevent self-intersection
    constexpr float EPS2 = 1e-8f; // angle for parallel judgement
    constexpr float EPS3 = 1e-4f; // relative distance to enlarge kdtree

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord);

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord);

    struct Intersection {
        float t, u, v; // ray parameter t, barycentric coordinates (u, v)
    };

    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3);
    
    struct RayHit {
        bool              IntersectState;
        Engine::BlendMode IntersectMode;
        glm::vec3         IntersectPosition;
        glm::vec3         IntersectNormal;
        glm::vec4         IntersectAlbedo;   // [Albedo   (vec3), Alpha     (float)]
        glm::vec4         IntersectMetaSpec; // [Specular (vec3), Shininess (float)]
        glm::vec3         IntersectEmission;
    };

    struct FastRayIntersector {
        Engine::Scene const * InternalScene = nullptr;

        float get_alpha(const Intersection &its, const int &modelIdx, const int &meshIdx) const {
            const float &tmin = its.t;
            const float &umin = its.u;
            const float &vmin = its.v;
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            return GetAlbedo(material, uvCoord).w;
        }
        struct BVH {
            struct My_Triangle {
                glm::vec3 p[3];
                glm::vec3 g; //重心
                int modelIdx, meshIdx;
                My_Triangle(glm::vec3 const &p1, glm::vec3 const &p2, glm::vec3 const &p3, const int &i, const int &j) {
                    p[0] = p1;
                    p[1] = p2;
                    p[2] = p3;
                    g = (p1 + p2 + p3) / 3.0f;
                    modelIdx = i, meshIdx = j;
                }
            };
            std::vector<My_Triangle> all_triangle; 
            void push_triangle(glm::vec3 const &p1, glm::vec3 const &p2, glm::vec3 const &p3, const int &i, const int &j) {
                all_triangle.push_back(My_Triangle(p1, p2, p3, i, j));
                return;
            }
            struct Node {
                int l, r;
                glm::vec3 box1, box2;
                std::vector<My_Triangle> triangle;
                Node() {}
            };
            Node *node;
            int root;
            int cntNode;
            BVH() {}
            ~BVH() {
                delete[] node;
            }
            void init(bool SAH_on) {
                node = new Node[all_triangle.size()];
                cntNode = 0;
                root = ++cntNode;
                if (SAH_on) {
                    std::cout << "Using SAH" << std::endl;
                    SAH_build(all_triangle, root);
                }
                else {
                    std::cout << "Using BVH" << std::endl;
                    build(all_triangle, root);
                }
                return;
            }
            bool in_box(int now, Ray const &ray) const { //div 0?
                float t_enter = -INFINITY;
                float t_exit = INFINITY;
                for (int i = 0; i < 3; ++i) {
                    if (std::abs(ray.Direction[i]) < EPS2) {
                        if (ray.Origin[i] < node[now].box1[i] || ray.Origin[i] > node[now].box2[i]) return false;
                        continue;
                    }
                    float dis_box1 = (node[now].box1[i] - ray.Origin[i]) / ray.Direction[i];
                    float dis_box2 = (node[now].box2[i] - ray.Origin[i]) / ray.Direction[i];
                    t_enter = std::max(t_enter, std::min(dis_box1, dis_box2));
                    t_exit = std::min(t_exit, std::max(dis_box1, dis_box2));
                }
                return t_enter <= t_exit && t_exit >= 0; //仅1平面特殊情况！
            }
            float get_t_enter(int now, Ray const &ray) const {
                float t_enter = -INFINITY;
                for (int i = 0; i < 3; ++i) {
                    if (std::abs(ray.Direction[i]) < EPS2) continue;
                    float dis_box1 = (node[now].box1[i] - ray.Origin[i]) / ray.Direction[i];
                    float dis_box2 = (node[now].box2[i] - ray.Origin[i]) / ray.Direction[i];
                    t_enter = std::max(t_enter, std::min(dis_box1, dis_box2));
                }
                return t_enter;
            }
            bool query(int now, Intersection & output, std::pair<int, int> &Idx, Ray const & ray) const {
                bool flag = false;
                if (!in_box(now, ray)) return false;
                float t_estimate = get_t_enter(now, ray);
                if (t_estimate > output.t) return false; //剪枝2

                if (!node[now].l) { //leaf
                    for (auto it : node[now].triangle) {
                        Intersection its;
                        if (!IntersectTriangle(its, ray, it.p[0], it.p[1], it.p[2])) continue;
                        if (its.t < EPS1) continue;
                        flag = true;
                        if (its.t < output.t) {
                            Idx = std::make_pair(it.modelIdx, it.meshIdx);
                            output = its;
                        }
                    }
                    return flag;
                }
                flag |= query(node[now].l, output, Idx, ray);
                flag |= query(node[now].r, output, Idx, ray);
                return flag;
            }
            bool shadow_query(bool &changed, int now, Intersection & output, std::pair<int, int> &Idx, Ray const & ray, const VCX::Labs::Rendering::FastRayIntersector * f) const {
                if (changed) return false;
                bool flag = false;
                if (!in_box(now, ray)) return false;
                float t_estimate = get_t_enter(now, ray);
                if (t_estimate > output.t) return false; //剪枝2

                if (!node[now].l) { //leaf
                    for (auto it : node[now].triangle) {
                        Intersection its;
                        if (!IntersectTriangle(its, ray, it.p[0], it.p[1], it.p[2])) continue;
                        if (its.t < EPS1) continue;
                        float alpha = f->get_alpha(its, it.modelIdx, it.meshIdx);
                        if (alpha < 0.2) continue; //透明剪枝
                        flag = true;
                        if (its.t < output.t) {
                            changed = true;
                            Idx = std::make_pair(it.modelIdx, it.meshIdx);
                            output = its;
                            return true;
                        }
                    }
                    return flag;
                }
                flag |= shadow_query(changed, node[now].l, output, Idx, ray, f);
                flag |= shadow_query(changed, node[now].r, output, Idx, ray, f);
                return flag;
            }
            void SAH_build(std::vector<My_Triangle> &now_triangle, int now) {
                node[now].l = node[now].r = 0;
                node[now].box1 = glm::vec3(INFINITY);
                node[now].box2 = glm::vec3(-INFINITY);
                node[now].l = node[now].r = 0;
                for (auto it : now_triangle) {
                    for (int i = 0; i < 3; ++i) {
                        node[now].box1 = glm::min(node[now].box1, it.p[i]);
                        node[now].box2 = glm::max(node[now].box2, it.p[i]);
                    }
                }
                int tot = now_triangle.size();
                if (tot < 5) {
                    node[now].triangle = now_triangle;
                    return;
                }
                
                float max_delta = 0.0f;
                int flag = 0;
                int mid = tot / 2;
                for (int i = 0; i < 3; ++i) {
                    if (node[now].box2[i] - node[now].box1[i] > max_delta) {
                        max_delta = node[now].box2[i] - node[now].box1[i];
                        flag = i;
                    }
                }
                if (tot <= 8) { //小数据用均分
                    std::nth_element(now_triangle.begin(), now_triangle.begin() + mid, now_triangle.end(), [flag](const auto&a, const auto&b){return a.g[flag] < b.g[flag];});
                    node[now].l = ++cntNode;
                    node[now].r = ++cntNode;
                    std::vector<My_Triangle> l_triangle;
                    std::vector<My_Triangle> r_triangle;
                    for (auto it = now_triangle.begin(); it != now_triangle.begin() + mid; ++it) l_triangle.push_back(*it);
                    for (auto it = now_triangle.begin() + mid; it != now_triangle.end(); ++it) r_triangle.push_back(*it);
                    SAH_build(l_triangle, node[now].l);
                    SAH_build(r_triangle, node[now].r);
                    return;
                }
                
                std::sort(now_triangle.begin(), now_triangle.end(), [flag](const auto&a, const auto&b){return a.g[flag] < b.g[flag];});
                mid = 0;
                float min_cost = (float)tot;
                float SA = 2.0f * (node[now].box2.x - node[now].box1.x) * (node[now].box2.y - node[now].box1.y) + 2.0f * (node[now].box2.x - node[now].box1.x) * (node[now].box2.z - node[now].box1.z) + 2.0f * (node[now].box2.y - node[now].box1.y) * (node[now].box2.z - node[now].box1.z);            
                {
                    std::vector<glm::vec3> tmp_box1_from_front(tot);
                    std::vector<glm::vec3> tmp_box2_from_front(tot);
                    std::vector<glm::vec3> tmp_box1_from_back(tot + 1);
                    std::vector<glm::vec3> tmp_box2_from_back(tot + 1);
                    tmp_box1_from_front[0]  = tmp_box1_from_back[tot] = glm::vec3(INFINITY);
                    tmp_box2_from_front[0]  = tmp_box2_from_back[tot] = glm::vec3(-INFINITY);
                    for (int i = 0; i < tot; ++i) {
                        if (i > 0) {
                            tmp_box1_from_front[i] = tmp_box1_from_front[i - 1];
                            tmp_box2_from_front[i] = tmp_box2_from_front[i - 1];
                        }
                        tmp_box1_from_back[tot - i - 1] = tmp_box1_from_back[tot - i];
                        tmp_box2_from_back[tot - i - 1] = tmp_box2_from_back[tot - i];
                        for (int j = 0; j < 3; ++j) {
                            tmp_box1_from_front[i] = glm::min(tmp_box1_from_front[i], now_triangle[i].p[j]);
                            tmp_box2_from_front[i] = glm::max(tmp_box2_from_front[i], now_triangle[i].p[j]);
                            tmp_box1_from_back[tot - i - 1] = glm::min(tmp_box1_from_back[tot - i - 1], now_triangle[tot - i - 1].p[j]);
                            tmp_box2_from_back[tot - i - 1] = glm::max(tmp_box2_from_back[tot - i - 1], now_triangle[tot - i - 1].p[j]);
                        }
                    }
                    for (int i = 1; i < tot - 1; ++i) {    
                        int cnt1 = i + 1;
                        int cnt2 = tot - i - 1;
                        float S1 = 2.0f * (tmp_box2_from_front[i].x - tmp_box1_from_front[i].x) * (tmp_box2_from_front[i].y - tmp_box1_from_front[i].y) + 2.0f * (tmp_box2_from_front[i].x - tmp_box1_from_front[i].x) * (tmp_box2_from_front[i].z - tmp_box1_from_front[i].z) + 2.0f * (tmp_box2_from_front[i].y - tmp_box1_from_front[i].y) * (tmp_box2_from_front[i].z - tmp_box1_from_front[i].z);
                        float S2 = 2.0f * (tmp_box2_from_back[i + 1].x - tmp_box1_from_back[i + 1].x) * (tmp_box2_from_back[i + 1].y - tmp_box1_from_back[i + 1].y) + 2.0f * (tmp_box2_from_back[i + 1].x - tmp_box1_from_back[i + 1].x) * (tmp_box2_from_back[i + 1].z - tmp_box1_from_back[i + 1].z) + 2.0f * (tmp_box2_from_back[i + 1].y - tmp_box1_from_back[i + 1].y) * (tmp_box2_from_back[i + 1].z - tmp_box1_from_back[i + 1].z);
                        float cost = 0.2f + (S1 / SA) * cnt1 + (S2 / SA) * cnt2;
                        if (cost < min_cost) {
                            min_cost = cost;
                            mid = i;
                        }
                    }   
                }
                if (mid == 0) {
                    node[now].triangle = now_triangle;
                    return;
                }
                node[now].l = ++cntNode;
                node[now].r = ++cntNode;
                std::vector<My_Triangle> l_triangle;
                std::vector<My_Triangle> r_triangle;
                for (auto it = now_triangle.begin(); it != now_triangle.begin() + mid + 1; ++it) l_triangle.push_back(*it);
                for (auto it = now_triangle.begin() + mid + 1; it != now_triangle.end(); ++it) r_triangle.push_back(*it);
                SAH_build(l_triangle, node[now].l);
                SAH_build(r_triangle, node[now].r);
                return;
            }
            void build(std::vector<My_Triangle> &now_triangle, int now) {
                node[now].l = node[now].r = 0;
                node[now].box1 = glm::vec3(INFINITY);
                node[now].box2 = glm::vec3(-INFINITY);
                node[now].l = node[now].r = 0;
                for (auto it : now_triangle) {
                    for (int i = 0; i < 3; ++i) {
                        node[now].box1 = glm::min(node[now].box1, it.p[i]);
                        node[now].box2 = glm::max(node[now].box2, it.p[i]);
                    }
                }
                int tot = now_triangle.size();
                if (tot < 5) {
                    node[now].triangle = now_triangle;
                    return;
                }
                int mid = tot / 2;
                float max_delta = 0.0f;
                int flag = 0;
                for (int i = 0; i < 3; ++i) {
                    if (node[now].box2[i] - node[now].box1[i] > max_delta) {
                        max_delta = node[now].box2[i] - node[now].box1[i];
                        flag = i;
                    }
                }
                if (flag == 0) std::nth_element(now_triangle.begin(), now_triangle.begin() + mid, now_triangle.end(), [](const auto&a, const auto&b){return a.g.x < b.g.x;});
                else if (flag == 1) std::nth_element(now_triangle.begin(), now_triangle.begin() + mid, now_triangle.end(), [](const auto&a, const auto&b){return a.g.y < b.g.y;});
                else if (flag == 2) std::nth_element(now_triangle.begin(), now_triangle.begin() + mid, now_triangle.end(), [](const auto&a, const auto&b){return a.g.z < b.g.z;});
                node[now].l = ++cntNode;
                node[now].r = ++cntNode;
                std::vector<My_Triangle> l_triangle;
                std::vector<My_Triangle> r_triangle;
                for (auto it = now_triangle.begin(); it != now_triangle.begin() + mid; ++it) l_triangle.push_back(*it);
                for (auto it = now_triangle.begin() + mid; it != now_triangle.end(); ++it) r_triangle.push_back(*it);
                build(l_triangle, node[now].l);
                build(r_triangle, node[now].r);
                return;
            }
        };

        FastRayIntersector() = default;
        BVH *bvh = NULL;
        void InitScene(Engine::Scene const * scene, bool SAH_on) {
            InternalScene = scene;
            if (bvh) delete bvh;
            bvh = new BVH;
            Intersection its;
            int          maxmodel = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                for (int j = 0; j < maxidx; j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    glm::vec3 const &     p1   = model.Mesh.Positions[face[0]];
                    glm::vec3 const &     p2   = model.Mesh.Positions[face[1]];
                    glm::vec3 const &     p3   = model.Mesh.Positions[face[2]];
                    bvh->push_triangle(p1, p2, p3, i, j);
                }
            }
            bvh->init(SAH_on);
            return;
        }
        RayHit IntersectRay(Ray const & ray) const {
            RayHit result;
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }
            int          modelIdx, meshIdx;
            Intersection its;
            std::pair<int, int> Idx;
            float        tmin     = 1e7, umin, vmin;
            int          maxmodel = InternalScene->Models.size();
            its.t = INFINITY;
            if (!bvh->query(bvh->root, its, Idx, ray)) {
                result.IntersectState = false;
                return result;
            }
            modelIdx = Idx.first;
            meshIdx  = Idx.second;
            tmin = its.t;
            umin = its.u;
            vmin = its.v;
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);
            result.IntersectEmission        = material.Emission;
            return result;
        }
        //if the light is point, then ray.deriction should not be normalized, and is_point should be true
        RayHit IntersectShadowRay(Ray const & ray, bool is_point) const {
            RayHit result;
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }
            int          modelIdx, meshIdx;
            Intersection its;
            std::pair<int, int> Idx;
            float        tmin     = 1e7, umin, vmin;
            int          maxmodel = InternalScene->Models.size();

            auto tmp_p = ray.Origin + ray.Direction;
            if (is_point) {
                its.t = 1.0f;
            }
            else its.t = INFINITY;
            
            bool changed = false;
            bvh->shadow_query(changed, bvh->root, its, Idx, ray, this);
            if (!changed) {
                result.IntersectState = false;
                return result;
            }
            modelIdx = Idx.first;
            meshIdx  = Idx.second;
            tmin = its.t;
            umin = its.u;
            vmin = its.v;
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);
            result.IntersectEmission        = material.Emission;
            return result;
        }
    };
    /* Optional: write your own accelerated intersector here */
    
    using RayIntersector = FastRayIntersector;

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow);
    glm::vec3 PathTrace(const RayIntersector & intersector, const VCX::Labs::Rendering::RayHit &hit_p, const Ray &ray);
} // namespace VCX::Labs::Rendering
