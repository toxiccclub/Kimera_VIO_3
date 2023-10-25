#include "CaptureService.h"

#include "DeviceIMU.h"

#include "log.h"

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include<iostream>
#include <chrono>
#include <thread>

using namespace std::literals::chrono_literals;

using namespace VIO::capture;

bool checkPath(const std::string &rawPath, boost::filesystem::path &path) {
    boost::filesystem::path oPath(rawPath);

    if(!boost::filesystem::exists(oPath)) {
        try{
            boost::filesystem::create_directories(oPath);
        }
        catch(boost::filesystem::filesystem_error &e)
        {
            TRACE_N_CONSOLE(ERROR, "Cannot crate output directory: %s", e.what());
            return false;
        }
    }
    else {
        if(!boost::filesystem::is_directory(oPath)) {
            TRACE_N_CONSOLE(ERROR, "Output path  %s is not honest directory.", boost::filesystem::canonical(oPath).c_str());
            return false;
        }
    }

    path = boost::filesystem::canonical(oPath);
    return true;
}

int main_() {
    DeviceIMU cam("/dev/ttyACM0", 115200);
    cam.open();
    FrameCallback cb = [](const void *, const IMV_FrameInfo *)->void {
            return;
    };
    while(true) {
        cam.nextFrame(cb,1000);
    }

}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);

    boost::program_options::options_description options("Options");
    options.add_options()
            ("help,h", "Produce this help")
            ("left,m", boost::program_options::value<std::string>()->default_value("10.0.0.2"), "Left camera IP address")
            ("right,s", boost::program_options::value<std::string>()->default_value("10.0.0.1"), "Right camera IP address")
            ("path,p", boost::program_options::value<std::string>()->default_value("."), "Path to storage")
            ("imu-port", boost::program_options::value<std::string>()->default_value("/dev/ttyACM0"), "IMU sensor port")
            ("height", boost::program_options::value<std::string>()->default_value("1024"), "Image height")
            ("width", boost::program_options::value<std::string>()->default_value("1280"), "Image width")
            ("image-bpp", boost::program_options::value<std::string>()->default_value("8"), "Image bpp");


    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, options), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << options << "\n";
        return 0;
    }

    std::string master = vm["left"].as<std::string>();
    std::string slave = vm["right"].as<std::string>();

    boost::filesystem::path path;
    if(checkPath(vm["path"].as<std::string>(), path)) {
        path /= std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    }
    else {
        return 1;
    }

    if(!checkPath(path.string(), path)) {
        return 1;
    }

    size_t height;
    size_t width;
    size_t bpp;
    try {
        height = 1024;//boost::lexical_cast<size_t>("1024");//(vm["height"].as<std::string>());
        width = 1280;//boost::lexical_cast<size_t>("1280");//(vm["width"].as<std::string>());
        bpp = 8;//boost::lexical_cast<size_t>("8");//(vm["image-bpp"].as<std::string>());
    }
    catch (...) {
        TRACE(ERROR, "Non-numeric feature for image format");
        return 1;
    }
    CaptureSettings settings{
    {
        {
            master,
            true,
            width,
            height,
            bpp,
            ""
        },
        {
            slave,
            false,
            width,
            height,
            bpp,
            ""
        }
    },
    true,
    true};

    auto failureEvent = neosmart::CreateEvent(true,false);

    auto capturer = new CameraCaptureService (settings,
        path.native()+boost::filesystem::path::separator,
        vm["imu-port"].as<std::string>(), 115200,
        &failureEvent);

    capturer->start();

    std::this_thread::sleep_for(500000000ms);

    delete capturer;

    return 0;
}
