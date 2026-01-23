#include "ares_embedded.hpp"

exported_class::exported_class()
{
}

auto exported_class::name() const -> char const*
{
  return m_name;
}
