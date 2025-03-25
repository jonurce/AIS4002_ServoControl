#include <threepp/threepp.hpp>

#include <threepp/helpers/CameraHelper.hpp>
#include <threepp/loaders/CubeTextureLoader.hpp>
#include <threepp/renderers/GLRenderTarget.hpp>

#include <opencv2/opencv.hpp>

#include <cmath>


using namespace threepp;

class PanTiltMechanism : public Group {
public:
    explicit PanTiltMechanism(WindowSize camSize): camera_(60, camSize.aspect(), 0.01, 100) {
        camera_.position.x = 0.03;
        camera_.rotateY(math::degToRad(-90));

        static OBJLoader loader;
        const auto bottom = loader.load("data/pantilt/bottom.obj");
        const auto upper = loader.load("data/pantilt/upper.obj");
        const auto servo = loader.load("data/pantilt/servo.obj");

        bottom->position.x = 0.66;
        bottom->position.y = 0.75;
        bottom->scale *= 10;
        upper->name = "upper";

        auto material = MeshStandardMaterial::create();
        material->color = Color::orange;
        auto baseGeometry = BoxGeometry::create(0.6, 1, 0.6);
        baseGeometry->applyMatrix4(Matrix4().setPosition(0, 0.5, 0));
        auto base = Mesh::create(baseGeometry, material);

        add(base);
        base->add(bottom);
        bottom->add(servo);
        servo->add(upper);
        upper->add(camera_);
    }

    float getTiltAngle() {
        return getObjectByName("upper")->rotation.z;
    }

    float getPanAngle() const {
        return children.front()->rotation.y;
    }

    void setTiltSpeed(float rad) {
        tiltSpeed_ = rad;
    }

    void setPanSpeed(float rad) {
        panSpeed_ = rad;
    }

    void update(float delta) {
        //tilt
        getObjectByName("upper")->rotation.z += std::clamp(tiltSpeed_, -maxSpeed_, maxSpeed_) * delta;
        //pan
        children.front()->rotation.y += std::clamp(panSpeed_, -maxSpeed_, maxSpeed_) * delta;
    }

    PerspectiveCamera &getCamera() {
        return camera_;
    }

private:
    float panSpeed_{0};
    float tiltSpeed_{0};

    float maxSpeed_{0.5};
    PerspectiveCamera camera_;
};

std::shared_ptr<Object3D> loadHuman() {
    static OBJLoader loader;
    const auto obj = loader.load("data/female02/female02.obj");
    obj->scale /= 10;
    return obj;
}

void setBackground(Scene &scene) {
    std::filesystem::path path("data/Bridge2");
    std::array urls{
        // clang-format off
        path / "posx.jpg", path / "negx.jpg",
        path / "posy.jpg", path / "negy.jpg",
        path / "posz.jpg", path / "negz.jpg"
        // clang-format on
    };

    CubeTextureLoader cubeTextureLoader{};
    const auto reflectionCube = cubeTextureLoader.load(urls);
    scene.background = reflectionCube;
}

int main() {
    Canvas canvas("Servo control", {{"resizable", false}});
    GLRenderer renderer(canvas.size());
    renderer.autoClear = false;

    Scene scene;
    setBackground(scene);

    const auto light = HemisphereLight::create();
    scene.add(light);

    PerspectiveCamera camera(60, canvas.aspect(), 0.1, 1000.);
    camera.position.z = -5;

    const auto grid = GridHelper::create();
    scene.add(grid);

    const auto human = loadHuman();
    human->rotateY(math::degToRad(180));
    human->position.z = 50;
    scene.add(human);

    std::pair virtualCameraSize = {640, 640};
    PanTiltMechanism panTilt(virtualCameraSize);
    scene.add(panTilt);

    auto cameraHelper = CameraHelper::create(panTilt.getCamera());
    scene.add(cameraHelper);

    OrbitControls controls(camera, canvas);

    const std::string openCVWindowName = "OpenCV PanTilt";
    cv::namedWindow(openCVWindowName, cv::WINDOW_AUTOSIZE);

    Clock clock;
    long long tick{};
    cv::Mat image;
    canvas.animate([&] {
        const bool renderVirtual = tick % 2 == 0;

        const auto dt = clock.getDelta();
        panTilt.update(dt);

        constexpr float amplitude = 90;
        constexpr float frequency = 0.05;
        const auto speed = math::TWO_PI * frequency * amplitude *
                           std::cos(math::TWO_PI * frequency * clock.elapsedTime);
        panTilt.setPanSpeed(math::degToRad(speed));

        if (renderVirtual) {
            renderer.clear();
            cameraHelper->visible = false;
            renderer.setSize(virtualCameraSize);
            renderer.render(scene, panTilt.getCamera());
            cameraHelper->visible = true;

            image = cv::Mat(virtualCameraSize.first, virtualCameraSize.second, CV_8UC3);
            renderer.readPixels({0, 0}, virtualCameraSize, Format::BGR, image.data);
        }

        renderer.clear();
        renderer.setSize(canvas.size());
        renderer.render(scene, camera);

        if (renderVirtual) {
            // OpenGL stores pixels bottom-to-top, OpenCV is top-to-bottom, so flip
            cv::flip(image, image, 0);

            cv::imshow(openCVWindowName, image);
        }

        ++tick;
    });

    return 0;
}
