#include <common_lib.h>

namespace CommonLib {

  common_lib::common_lib(const std::string& pkg_mode_) {
    PKG_VERSION(pkg_mode_);
  }

  common_lib::~common_lib() {}

  void common_lib::PKG_VERSION(const std::string& pkg_mode_) {
    std::cout << "\033[34m" << std::string(60, '*')                       << std::endl;
    std::cout << "Author          : Yixin Fang"                        << std::endl;
    std::cout << "Slam mode       : " << pkg_mode_                        << std::endl;
    std::cout << "Package version : v0.0.0"                               << std::endl;
    std::cout << "Package link    : https://github.com/Yixin-F/LiLoc" << std::endl;
    std::cout << std::string(60, '*') << "\033[0m"                        << std::endl;
  }

} // namespace CommonLib