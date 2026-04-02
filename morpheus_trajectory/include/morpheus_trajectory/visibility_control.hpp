#ifndef MORPHEUS_TRAJECTORY__VISIBILITY_CONTROL_H_
#define MORPHEUS_TRAJECTORY__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MORPHEUS_TRAJECTORY_EXPORT __attribute__ ((dllexport))
    #define MORPHEUS_TRAJECTORY_IMPORT __attribute__ ((dllimport))
  #else
    #define MORPHEUS_TRAJECTORY_EXPORT __declspec(dllexport)
    #define MORPHEUS_TRAJECTORY_IMPORT __declspec(dllimport)
  #endif
  #ifdef MORPHEUS_TRAJECTORY_BUILDING_DLL
    #define MORPHEUS_TRAJECTORY_PUBLIC MORPHEUS_TRAJECTORY_EXPORT
  #else
    #define MORPHEUS_TRAJECTORY_PUBLIC MORPHEUS_TRAJECTORY_IMPORT
  #endif
  #define MORPHEUS_TRAJECTORY_PUBLIC_TYPE MORPHEUS_TRAJECTORY_PUBLIC
  #define MORPHEUS_TRAJECTORY_LOCAL
#else
  #define MORPHEUS_TRAJECTORY_EXPORT __attribute__ ((visibility("default")))
  #define MORPHEUS_TRAJECTORY_IMPORT
  #if __GNUC__ >= 4
    #define MORPHEUS_TRAJECTORY_PUBLIC __attribute__ ((visibility("default")))
    #define MORPHEUS_TRAJECTORY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MORPHEUS_TRAJECTORY_PUBLIC
    #define MORPHEUS_TRAJECTORY_LOCAL
  #endif
  #define MORPHEUS_TRAJECTORY_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MORPHEUS_TRAJECTORY__VISIBILITY_CONTROL_H_