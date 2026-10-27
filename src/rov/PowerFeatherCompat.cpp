// Workaround for PowerFeather-SDK 2.1.x under the Arduino framework.
//
// MasterI2C declares virtual start/end/write/read, but its .cpp only defines
// them for ESP-IDF builds (#ifndef ARDUINO). The Arduino build uses the
// ArduinoMasterI2C subclass, which overrides all four, yet the base class
// vtable still has to exist, so linking fails with
// "undefined reference to vtable for PowerFeather::MasterI2C".
// These base versions are never called.

#include <Utils/MasterI2C.h>

namespace PowerFeather {

bool MasterI2C::start() {
    return false;
}
bool MasterI2C::end() {
    return false;
}
bool MasterI2C::write(uint8_t, uint8_t, const uint8_t*, size_t) {
    return false;
}
bool MasterI2C::read(uint8_t, uint8_t, uint8_t*, size_t) {
    return false;
}

}  // namespace PowerFeather
