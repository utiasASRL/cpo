#pragma once

#include <cstdlib>
#include <stdexcept>
#include <string>

/** \brief Utility function to expand ~ to /home/username */
inline std::string expandUser(std::string path) {
  if (path.empty() || path[0] != '~') return path;

  if (path.size() != 1 && path[1] != '/')
    throw std::invalid_argument("Invalid syntax.");

  auto home = getenv("HOME");
  if (home || ((home = getenv("USERPROFILE")))) {
    path.replace(0, 1, home);
  } else {
    auto hdrive = getenv("HOMEDRIVE");
    auto hpath = getenv("HOMEPATH");
    if (!hdrive || !hpath) throw std::invalid_argument("Invalid syntax.");
    path.replace(0, 1, std::string(hdrive) + hpath);
  }
  return path;
}
