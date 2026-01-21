#include <string>

#include "ares_embedded/ares_embedded.hpp"

auto main() -> int
{
  auto const exported = exported_class {};

  return std::string("ares_embedded") == exported.name() ? 0 : 1;
}
