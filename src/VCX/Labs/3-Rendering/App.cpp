#include "Assets/bundled.h"
#include "Labs/3-Rendering/App.h"

namespace VCX::Labs::Rendering {
    using namespace Assets;

    App::App() :
        _ui(Labs::Common::UIOptions { }),
        _caseRayTracing({ ExampleScene::Floor, ExampleScene::CornellBox, ExampleScene::WhiteOak, ExampleScene::SportsCar, ExampleScene::BreakfastRoom, ExampleScene::Sponza }),
        _casePathTracing({ ExampleScene::Box_ver_path}) {
}

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
