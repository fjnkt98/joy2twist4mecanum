#ifndef JOY2TWIST4MECANUM__VISIBILITY_CONTROL_H_
#define JOY2TWIST4MECANUM__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JOY2TWIST4MECANUM_EXPORT __attribute__ ((dllexport))
    #define JOY2TWIST4MECANUM_IMPORT __attribute__ ((dllimport))
  #else
    #define JOY2TWIST4MECANUM_EXPORT __declspec(dllexport)
    #define JOY2TWIST4MECANUM_IMPORT __declspec(dllimport)
  #endif
  #ifdef JOY2TWIST4MECANUM_BUILDING_DLL
    #define JOY2TWIST4MECANUM_PUBLIC JOY2TWIST4MECANUM_EXPORT
  #else
    #define JOY2TWIST4MECANUM_PUBLIC JOY2TWIST4MECANUM_IMPORT
  #endif
  #define JOY2TWIST4MECANUM_PUBLIC_TYPE JOY2TWIST4MECANUM_PUBLIC
  #define JOY2TWIST4MECANUM_LOCAL
#else
  #define JOY2TWIST4MECANUM_EXPORT __attribute__ ((visibility("default")))
  #define JOY2TWIST4MECANUM_IMPORT
  #if __GNUC__ >= 4
    #define JOY2TWIST4MECANUM_PUBLIC __attribute__ ((visibility("default")))
    #define JOY2TWIST4MECANUM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JOY2TWIST4MECANUM_PUBLIC
    #define JOY2TWIST4MECANUM_LOCAL
  #endif
  #define JOY2TWIST4MECANUM_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // JOY2TWIST4MECANUM__VISIBILITY_CONTROL_H_
