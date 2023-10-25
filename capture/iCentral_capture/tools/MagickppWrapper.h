#ifndef ODOMETRY_MAGICKPPWRAPPER_H
#define ODOMETRY_MAGICKPPWRAPPER_H

#include <string>
namespace Magick{
    class Image;
}
class MagickppWrapper {
public:
    explicit MagickppWrapper(const size_t width, const size_t height, const void *pixels);
    virtual ~MagickppWrapper();
    void write(const std::string &objective);
private:
    Magick::Image *_pic;
};

#endif