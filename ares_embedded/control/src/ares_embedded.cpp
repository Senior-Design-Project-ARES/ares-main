#include <string>

#include "ares_embedded/ares_embedded.hpp"

exported_class::exported_class()
    : m_name {"ares_embedded"}
{
}

auto exported_class::name() const -> char const*
{
  return m_name.c_str();
}
