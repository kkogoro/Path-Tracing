#include "Labs/3-Rendering/CasePathTracing.h"
#include <vector>
#include <thread>
#include <iostream>
#include <condition_variable>
namespace VCX::Labs::Rendering {

    CasePathTracing::CasePathTracing(std::initializer_list<Assets::ExampleScene> && scenes):
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(4),
        _texture({ .MinFilter = Engine::GL::FilterMode::Linear, .MagFilter = Engine::GL::FilterMode::Nearest }) {
        _cameraManager.AutoRotate = false;
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1, 1, 1));
    }

    CasePathTracing::~CasePathTracing() {
        _stopFlag = true;
        if (_task.joinable()) {
            _task.join();
        }
    }

    void CasePathTracing::OnSetupPropsUI() {
        if (ImGui::BeginCombo("Scene", GetSceneName(_sceneIdx))) {
            for (std::size_t i = 0; i < _scenes.size(); ++i) {
                bool selected = i == _sceneIdx;
                if (ImGui::Selectable(GetSceneName(i), selected)) {
                    if (! selected) {
                        _sceneIdx   = i;
                        _sceneDirty = true;
                        _treeDirty = true;
                        _resetDirty = true;
                    }
                }
            }
            ImGui::EndCombo();
        }
        if (ImGui::Button("Reset Scene")) _resetDirty = true;
        ImGui::SameLine();
        if (_task.joinable()) {
            if (ImGui::Button("Stop Rendering")) {
                _stopFlag = true;
                if (_task.joinable()) {
                    _task.join();
                }
            }
        } else if (ImGui::Button("Start Rendering")) _stopFlag = false;
        ImGui::ProgressBar(float(_pixelIndex) / (_buffer.GetSizeX() * _buffer.GetSizeY()));
        Common::ImGuiHelper::SaveImage(_texture, GetBufferSize(), true);
        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Accelerator", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool tmp_flag = ImGui::Checkbox("Use SAH(Default : BVH)", &_SAH_on);
            _resetDirty |= tmp_flag;
            _treeDirty |= tmp_flag;
        }
        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            _resetDirty |= ImGui::SliderInt("Sample Rate", &_superSampleRate, 1, 20);
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Control")) {
            ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CasePathTracing::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_resetDirty) {
            _stopFlag = true;
            if (_task.joinable()) _task.join();
            _pixelIndex = 0;
            _resizable  = true;
            _resetDirty = false;
        }
        if (_sceneDirty) {
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            _cameraManager.Save(_sceneObject.Camera);
            _sceneDirty = false;
        }
        if (_resizable) {
            _frame.Resize(desiredSize);
            _cameraManager.Update(_sceneObject.Camera);
            _program.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
            _program.GetUniforms().SetByName("u_View"      , _sceneObject.Camera.GetViewMatrix());
            
            gl_using(_frame);

            glEnable(GL_DEPTH_TEST);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            for (auto const & model : _sceneObject.OpaqueModels)
                model.Mesh.Draw({ _program.Use() });
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_DEPTH_TEST);
        }
        if (! _stopFlag && ! _task.joinable()) {
            if (_pixelIndex == 0) {
                _resizable = false;
                _buffer    = _frame.GetColorAttachment().Download<Engine::Formats::RGB8>();
            }
            _task = std::thread([&]() {
                auto const width  = _buffer.GetSizeX();
                auto const height = _buffer.GetSizeY();
                if (_pixelIndex == 0 && _treeDirty) {
                    Engine::Scene const & scene = GetScene(_sceneIdx);
                    _intersector.InitScene(&scene, _SAH_on);
                    _treeDirty = false;
                }
                // Render into tex.

                unsigned const concurrent_count = std::thread::hardware_concurrency();
                //unsigned const concurrent_count = 2;
                std::cout << "hardware_concurrency: " << concurrent_count << std::endl;
                unsigned  cnt1 = 0;

                float        step = 1.0f / _superSampleRate;
                auto const & camera    = _sceneObject.Camera;
                float const  aspect    = width * 1.f / height;
                float const  fovFactor = std::tan(glm::radians(camera.Fovy) / 2);

                std::mutex mtx;
                std::condition_variable condVar;
                std::mutex consume;

                while (_pixelIndex < std::size_t(width) * height) {
                    int i = _pixelIndex % width;
                    int j = _pixelIndex / width;
                    int my_Index = _pixelIndex;

                    std::unique_lock<std::mutex> l(consume);
                    condVar.wait(l, [&]{return cnt1 < concurrent_count;});

                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        ++cnt1;
                        //std::cout << cnt1 << std::endl;
                    }

                    std::thread render_pixel([&,i, j, my_Index]() {
                        auto sum = glm::vec3(0.0f);
                        for (int dy = 0; dy < _superSampleRate; ++dy) {
                            for (int dx = 0; dx < _superSampleRate; ++dx) {
                                float        di = step * (0.5f + dx), dj = step * (0.5f + dy);
                                glm::vec3    lookDir   = glm::normalize(camera.Target - camera.Eye);
                                glm::vec3    rightDir  = glm::normalize(glm::cross(lookDir, camera.Up));
                                glm::vec3    upDir     = glm::normalize(glm::cross(rightDir, lookDir));     
                                lookDir += fovFactor * (2.0f * (j + dj) / height - 1.0f) * upDir;
                                lookDir += fovFactor * aspect * (2.0f * (i + di) / width - 1.0f) * rightDir;
                                Ray       initialRay(camera.Eye, glm::normalize(lookDir));
                                
                                auto rayHit = _intersector.IntersectRay(initialRay);
                                if (! rayHit.IntersectState) continue;

                                glm::vec3 res = PathTrace(_intersector, rayHit, initialRay);
                                sum += glm::pow(res, glm::vec3(1.0 / 2.2)) / glm::vec3(_superSampleRate * _superSampleRate);
                            }
                        }
                        {
                            std::lock_guard<std::mutex> lock(mtx);
                            _buffer.At(i, j) = sum;
                            --cnt1;
                        }
                        condVar.notify_all();
                    });
                    render_pixel.detach();
                    ++_pixelIndex;

                    if (_stopFlag) {
                        condVar.wait(l, [&]{return cnt1 == 0;});
                        return;
                    }
                }
                
                std::unique_lock<std::mutex> l(consume);
                condVar.wait(l, [&]{return cnt1 == 0;});
            });
        }
        if (! _resizable) {
            if (!_stopFlag) _texture.Update(_buffer);
            if (_task.joinable() && _pixelIndex == _buffer.GetSizeX() * _buffer.GetSizeY()) {
                _stopFlag = true;
                _task.join();
            }
        }
        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _resizable ? _frame.GetColorAttachment() : _texture,
            .ImageSize = _resizable ? desiredSize : GetBufferSize(),
        };
    }

    void CasePathTracing::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImVec2 const delta   = ImGui::GetIO().MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        if (! hovered) return;
        if (_resizable) {
            _cameraManager.ProcessInput(_sceneObject.Camera, pos);
        } else {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.x != 0.f)
                ImGui::SetScrollX(window, window->Scroll.x - delta.x);
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.y != 0.f)
                ImGui::SetScrollY(window, window->Scroll.y - delta.y);
        }
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered())
            Common::ImGuiHelper::ZoomTooltip(_resizable ? _frame.GetColorAttachment() : _texture, GetBufferSize(), pos, true);
    }

} // namespace VCX::Labs::Rendering
