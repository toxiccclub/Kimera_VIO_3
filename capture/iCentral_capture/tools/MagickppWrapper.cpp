#include "MagickppWrapper.h"

# define MAGICKCORE_HDRI_ENABLE 0
# define MAGICKCORE_QUANTUM_DEPTH 16

#include <Magick++.h>

MagickppWrapper::MagickppWrapper(const size_t width,
                                 const size_t height,
                                 const void* pixels):
                                 _pic(new Magick::Image(width, height, "I", Magick::StorageType::CharPixel, pixels))
                                  {}

MagickppWrapper::~MagickppWrapper() {
    delete _pic;
}

void MagickppWrapper::write(const std::string& objective) {
    _pic->write(objective);
}
