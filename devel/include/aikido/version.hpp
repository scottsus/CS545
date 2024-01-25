/* version.hpp. Generated by CMake for aikido. */
#ifndef AIKIDO_VERSION_HPP_
#define AIKIDO_VERSION_HPP_

/* Version number */
#define AIKIDO_MAJOR_VERSION 0
#define AIKIDO_MINOR_VERSION 3
#define AIKIDO_PATCH_VERSION 0

#define AIKIDO_VERSION "0.3.0"

#define AIKIDO_VERSION_AT_LEAST(x,y,z) \
  (AIKIDO_MAJOR_VERSION > x || (AIKIDO_MAJOR_VERSION >= x && \
  (AIKIDO_MINOR_VERSION > y || (AIKIDO_MINOR_VERSION >= y && \
  AIKIDO_PATCH_VERSION >= z))))

#define AIKIDO_MAJOR_MINOR_VERSION_AT_LEAST(x,y) \
  (AIKIDO_MAJOR_VERSION > x || (AIKIDO_MAJOR_VERSION >= x && \
  (AIKIDO_MINOR_VERSION > y || (AIKIDO_MINOR_VERSION >= y))))

#define AIKIDO_VERSION_AT_MOST(x,y,z) \
  (AIKIDO_MAJOR_VERSION < x || (AIKIDO_MAJOR_VERSION <= x && \
  (AIKIDO_MINOR_VERSION < y || (AIKIDO_MINOR_VERSION <= y && \
  AIKIDO_PATCH_VERSION <= z))))

#define AIKIDO_MAJOR_MINOR_VERSION_AT_MOST(x,y) \
  (AIKIDO_MAJOR_VERSION < x || (AIKIDO_MAJOR_VERSION <= x && \
  (AIKIDO_MINOR_VERSION < y || (AIKIDO_MINOR_VERSION <= y))))

#endif // #ifndef AIKIDO_VERSION_HPP_
