#pragma once

#include <stdexcept>

namespace rko_lio::core {

class InputError : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

class RegistrationError : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

} // namespace rko_lio::core
