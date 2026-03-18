// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef LOTTI_CONTROL__VISIBILITY_CONTROL_H_
#define LOTTI_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFFDRIVE_LOTTI_EXPORT __attribute__((dllexport))
#define DIFFDRIVE_LOTTI_IMPORT __attribute__((dllimport))
#else
#define DIFFDRIVE_LOTTI_EXPORT __declspec(dllexport)
#define DIFFDRIVE_LOTTI_IMPORT __declspec(dllimport)
#endif
#ifdef DIFFDRIVE_LOTTI_BUILDING_DLL
#define DIFFDRIVE_LOTTI_PUBLIC DIFFDRIVE_LOTTI_EXPORT
#else
#define DIFFDRIVE_LOTTI_PUBLIC DIFFDRIVE_LOTTI_IMPORT
#endif
#define DIFFDRIVE_LOTTI_PUBLIC_TYPE DIFFDRIVE_LOTTI_PUBLIC
#define DIFFDRIVE_LOTTI_LOCAL
#else
#define DIFFDRIVE_LOTTI_EXPORT __attribute__((visibility("default")))
#define DIFFDRIVE_LOTTI_IMPORT
#if __GNUC__ >= 4
#define DIFFDRIVE_LOTTI_PUBLIC __attribute__((visibility("default")))
#define DIFFDRIVE_LOTTI_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFFDRIVE_LOTTI_PUBLIC
#define DIFFDRIVE_LOTTI_LOCAL
#endif
#define DIFFDRIVE_LOTTI_PUBLIC_TYPE
#endif

#endif  // LOTTI_CONTROL__VISIBILITY_CONTROL_H_