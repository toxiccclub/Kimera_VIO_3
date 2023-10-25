#include <glog/logging.h>

#include "FrameHeader.h"


#include <IMVDefines.h>

#include <boost/algorithm/string.hpp>

#include <boost/filesystem.hpp>

#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <boost/program_options.hpp>

#include <fmt/core.h>

//# define MAGICKCORE_HDRI_ENABLE 0
//# define MAGICKCORE_QUANTUM_DEPTH 16

//#include <Magick++.h>

#include "MagickppWrapper.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

void help(const char *argv, boost::program_options::options_description &options) {
    std::cout << "Decode and extract image and metadata"<<std::endl;
    std::cout << "Usage:\n"
    << "["<<argv<<"] [options --] <filename>\n"
    <<options<<std::endl;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);

    boost::program_options::options_description options("Options");
    options.add_options()
            ("help,h", "Produce this help")
            ("print-metainfo,m", boost::program_options::value<bool>()->default_value(false), "Print contents of metadata")
            ("format,f",boost::program_options::value<std::string>()->default_value("image"), "Data format: image or imu")
            ("output-path,o", boost::program_options::value<std::string>()->default_value("."), "Output path");

    boost::program_options::options_description hidden;
    hidden.add_options()
        ("positional", boost::program_options::value<std::vector<std::string> >())
        ;

    boost::program_options::options_description all_options;
    all_options.add(options).add(hidden);

    boost::program_options::positional_options_description p;
    p.add("positional", 1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).
              options(all_options).positional(p).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        help(argv[0], options);
        return 0;
    }
    auto mode = boost::algorithm::to_lower_copy(vm["format"].as<std::string>());

    bool processImage = true;

    if(mode == "image" || mode == "imu") {
        processImage = (mode == "image");
    }
    else {
        std::cout << "Unrecognised data format identity: "<<
        vm["format"].as<std::string>() << ". Use 'image' or 'imu'" << std::endl;
        return 1;
    }

    boost::filesystem::path fileName;

    if (vm.count("positional")) {
        const std::vector<std::string> &v = vm["positional"].as<std::vector<std::string> >();
        fileName = boost::filesystem::canonical(v.front());
    }
    else {
        std::cout << "No image specified: argument missing" << std::endl;
        return 1;
    }

    if(!boost::filesystem::exists(fileName) || !boost::filesystem::is_regular_file(fileName)) {
        std::cout << "No image specified: not a file" << std::endl;
        return 1;
    }

    auto fileSize = boost::filesystem::file_size(fileName);

    if(fileSize <= sizeof(VIO::capture::FrameHeader) + sizeof(IMV_FrameInfo)) {
        std::cout << "No image specified: file is too small (no space for expected metadata)" << std::endl;
        return 1;
    }

    boost::interprocess::file_mapping rawImage(fileName.c_str(), boost::interprocess::read_only);

    boost::interprocess::mapped_region vioHeaderHandle(rawImage, boost::interprocess::read_only, 0,
        sizeof(VIO::capture::FrameHeader));
    boost::interprocess::mapped_region imvHeaderHandle(rawImage, boost::interprocess::read_only, sizeof(VIO::capture::FrameHeader),
        sizeof(IMV_FrameInfo));

    VIO::capture::FrameHeader *vioHeader = static_cast<VIO::capture::FrameHeader *>(vioHeaderHandle.get_address());

    IMV_FrameInfo *imvHeader = static_cast<IMV_FrameInfo *>(imvHeaderHandle.get_address());

    if(fileSize != sizeof(VIO::capture::FrameHeader) + sizeof(IMV_FrameInfo) + imvHeader->size ||
        processImage && imvHeader->size != imvHeader->height * imvHeader->width) {
        std::cout << "File size dose not match or header is corrupted" << std::endl;
        return 1;
    }

    boost::interprocess::mapped_region pixmap(rawImage, boost::interprocess::read_only,
        sizeof(VIO::capture::FrameHeader) + sizeof(IMV_FrameInfo), imvHeader->size);

    boost::filesystem::path oPath =  vm["output-path"].as<std::string>();

    if(!boost::filesystem::exists(oPath)) {
        try{
            boost::filesystem::create_directories(oPath);
        }
        catch(boost::filesystem::filesystem_error &e)
        {
            std::cout<<"Cannot crate output directory: " << e.what() << std::endl;
            return 1;
        }
    }
    else {
        if(!boost::filesystem::is_directory(oPath)) {
            std::cout<<"Output path "<<boost::filesystem::canonical(oPath) << " is not honest directory.";
            return 1;
        }
    }

    oPath = boost::filesystem::canonical(oPath);

    if(processImage) {
        oPath.append(fmt::format("{}.png", vioHeader->timestampUTC));
        MagickppWrapper pic(imvHeader->width, imvHeader->height, pixmap.get_address());
        //Magick::Image pic(imvHeader->width, imvHeader->height, "I", Magick::StorageType::CharPixel, pixmap.get_address());
        pic.write(oPath.string());
    }
    else {
        oPath.append(fmt::format("{}.imu", vioHeader->timestampUTC));
        auto file = std::fstream(oPath.string(), std::ios::out | std::ios::binary);
        file.write(static_cast<char *>(pixmap.get_address()), imvHeader->size);
    }



    return 0;
}