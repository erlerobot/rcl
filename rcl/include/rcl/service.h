// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCL__SERVICE_H_
#define RCL__SERVICE_H_

#if __cplusplus
extern "C"
{
#endif

#include "rosidl_generator_c/message_type_support.h"

#include "rcl/macros.h"
#include "rcl/node.h"
#include "rcl/visibility_control.h"

/// Internal rcl implementation struct.
struct rcl_service_impl_t;

/// Handle for a rcl service.
typedef struct rcl_service_t
{
  struct rcl_service_impl_t * impl;
} rcl_service_t;

/// Options available for a rcl service.
typedef struct rcl_service_options_t
{
  /// Middleware quality of service settings for the service.
  rmw_qos_profile_t qos;
  /// If true, messages published from within the same node are ignored.
  bool ignore_local_publications;
  /// Custom allocator for the service, used for incidental allocations.
  /* For default behavior (malloc/free), see: rcl_get_default_allocator() */
  rcl_allocator_t allocator;
} rcl_service_options_t;

// TODO Redo comments

/// Return a rcl_service_t struct with members set to NULL.
/* Should be called to get a null rcl_service_t before passing to
 * rcl_initalize_service().
 * It's also possible to use calloc() instead of this if the rcl_service_t
 * is being allocated on the heap.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_service_t
rcl_get_zero_initialized_service(void);

/// Initialize a ROS service.
/* After calling this function on a rcl_service_t, it can be used to take
 * messages of the given type to the given topic using rcl_take().
 *
 * The given rcl_node_t must be valid and the resulting rcl_service_t is
 * only valid as long as the given rcl_node_t remains valid.
 *
 * The rosidl_message_type_support_t is obtained on a per .msg type basis.
 * When the user defines a ROS message, code is generated which provides the
 * required rosidl_message_type_support_t object.
 * This object can be obtained using a language appropriate mechanism.
 * \TODO(wjwwood) probably should talk about this once and link to it instead
 * For C this macro can be used (using std_msgs/String as an example):
 *
 *    #include <rosidl_generator_c/message_type_support.h>
 *    #include <std_msgs/msgs/string.h>
 *    rosidl_message_type_support_t * string_ts =
 *      ROSIDL_GET_MESSAGE_TYPE_SUPPORT(std_msgs, String);
 *
 * For C++ a template function is used:
 *
 *    #include <rosidl_generator_cpp/message_type_support.hpp>
 *    #include <std_msgs/msgs/string.hpp>
 *    rosidl_message_type_support_t * string_ts =
 *      rosidl_generator_cpp::get_message_type_support_handle<std_msgs::msg::String>();
 *
 * The rosidl_message_type_support_t object contains message type specific
 * information used to publish messages.
 *
 * \TODO(wjwwood) update this once we've come up with an official scheme.
 * The topic name must be a non-empty string which follows the topic naming
 * format.
 *
 * The options struct allows the user to set the quality of service settings as
 * well as a custom allocator which is used when (de)initializing the
 * service to allocate space for incidental things, e.g. the topic
 * name string.
 *
 * Expected usage (for C messages):
 *
 *    #include <rcl/rcl.h>
 *    #include <rosidl_generator_c/message_type_support.h>
 *    #include <std_msgs/msgs/string.h>
 *
 *    rcl_node_t node = rcl_get_zero_initialized_node();
 *    rcl_node_options_t node_ops = rcl_node_get_default_options();
 *    rcl_ret_t ret = rcl_node_init(&node, "node_name", &node_ops);
 *    // ... error handling
 *    rosidl_message_type_support_t * ts = ROSIDL_GET_MESSAGE_TYPE_SUPPORT(std_msgs, String);
 *    rcl_service_t service = rcl_get_zero_initialized_service();
 *    rcl_service_options_t service_ops = rcl_service_get_default_options();
 *    ret = rcl_service_init(&service, &node, ts, "chatter", &service_ops);
 *    // ... error handling, and when finished deinitialization
 *    ret = rcl_service_fini(&service, &node);
 *    // ... error handling for rcl_service_fini()
 *    ret = rcl_node_fini(&node);
 *    // ... error handling for rcl_deinitialize_node()
 *
 * This function is not thread-safe.
 *
 * \param[out] service preallocated service structure
 * \param[in] node valid rcl node handle
 * \param[in] type_support type support object for the topic's type
 * \param[in] topic_name the name of the topic
 * \param[in] options service options, including quality of service settings
 * \return RCL_RET_OK if service was initialized successfully, or
 *         RCL_RET_INVALID_ARGUMENT if any arugments are invalid, or
 *         RCL_RET_BAD_ALLOC if allocating memory failed, or
 *         RCL_RET_ERROR if an unspecified error occurs.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_service_init(
  rcl_service_t * service,
  const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rcl_service_options_t * options);

/// Deinitialize a rcl_service_t.
/* After calling, the node will no longer be subscribed on this topic
 * (assuming this is the only service on this topic in this node).
 *
 * After calling, calls to rcl_wait and rcl_take will fail when using this
 * service.
 * Additioanlly rcl_wait will be interrupted if currently blocking.
 * However, the given node handle is still valid.
 *
 * This function is not thread-safe.
 *
 * \param[inout] service handle to the service to be deinitialized
 * \param[in] node handle to the node used to create the service
 * \return RCL_RET_OK if service was deinitialized successfully, or
 *         RCL_RET_INVALID_ARGUMENT if any arugments are invalid, or
 *         RCL_RET_ERROR if an unspecified error occurs.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_service_fini(rcl_service_t * service, rcl_node_t * node);

/// Return the default service options in a rcl_service_options_t.
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_service_options_t
rcl_service_get_default_options(void);

/// Take a ROS message from a topic using a rcl service.
/* It is the job of the caller to ensure that the type of the ros_message
 * argument and the type associate with the service, via the type
 * support, match.
 * Passing a different type to rcl_take produces undefined behavior and cannot
 * be checked by this function and therefore no deliberate error will occur.
 *
 * TODO(wjwwood) blocking of take?
 * TODO(wjwwood) pre-, during-, and post-conditions for message ownership?
 * TODO(wjwwood) is rcl_take thread-safe?
 * TODO(wjwwood) Should there be an rcl_message_info_t?
 *
 * The ros_message pointer should point to an already allocated ROS message
 * struct of the correct type, into which the taken ROS message will be copied
 * if one is available.
 * If taken is false after calling, then the ROS message will be unmodified.
 *
 * If allocation is required when taking the message, e.g. if space needs to
 * be allocated for a dynamically sized array in the target message, then the
 * allocator given in the service options is used.
 *
 * The taken pointer should point to an already allocated boolean in which the
 * result can be stored.
 *
 * The rmw message_info struct contains meta information about this particular
 * message instance, like what the GUID of the publisher which published it
 * originally or whether or not the message received from within the same
 * process.
 * The message_info argument should be an already allocated rmw_message_info_t
 * structure.
 * Passing NULL for message_info will result in the argument being ignored.
 *
 * \param[in] service the handle to the service from which to take
 * \param[inout] ros_message type-erased ptr to a allocated ROS message
 * \param[out] taken pointer to a bool, if set to false a message was not taken
 * \param[out] message_info rmw struct which contains meta-data for the message
 * \return RCL_RET_OK if the message was published, or
 *         RCL_RET_INVALID_ARGUMENT if any arugments are invalid, or
 *         RCL_RET_SERVICE_INVALID if the service is invalid, or
 *         RCL_RET_BAD_ALLOC if allocating memory failed, or
 *         RCL_RET_ERROR if an unspecified error occurs.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_handle_request(
  const rcl_service_t * service,
  void * ros_request,
  void * ros_response);


// TODO comment
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_send_response(
  const rcl_service_t * service,
  void * ros_response,
  rmw_req_id_t req_id);

/// Get the topic name for the service.
/* This function returns the service's internal topic name string.
 * This function can fail, and therefore return NULL, if the:
 *   - service is NULL
 *   - service is invalid (never called init, called fini, or invalid)
 *
 * The returned string is only valid as long as the service is valid.
 * The value of the string may change if the topic name changes, and therefore
 * copying the string is recommended if this is a concern.
 *
 * This function is not thread-safe, and copying the result is not thread-safe.
 *
 * \param[in] service the pointer to the service
 * \return name string if successful, otherwise NULL
 */
RCL_PUBLIC
RCL_WARN_UNUSED
const char *
rcl_service_get_topic_name(const rcl_service_t * service);

/// Return the rcl service options.
/* This function returns the service's internal options struct.
 * This function can fail, and therefore return NULL, if the:
 *   - service is NULL
 *   - service is invalid (never called init, called fini, or invalid)
 *
 * The returned struct is only valid as long as the service is valid.
 * The values in the struct may change if the service's options change,
 * and therefore copying the struct is recommended if this is a concern.
 *
 * This function is not thread-safe, and copying the result is not thread-safe.
 *
 * \param[in] service pointer to the service
 * \return options struct if successful, otherwise NULL
 */
RCL_PUBLIC
RCL_WARN_UNUSED
const rcl_service_options_t *
rcl_service_get_options(const rcl_service_t * service);

/// Return the rmw service handle.
/* The handle returned is a pointer to the internally held rmw handle.
 * This function can fail, and therefore return NULL, if the:
 *   - service is NULL
 *   - service is invalid (never called init, called fini, or invalid)
 *
 * The returned handle is made invalid if the service is finalized or if
 * rcl_shutdown() is called.
 * The returned handle is not guaranteed to be valid for the life time of the
 * service as it may be finalized and recreated itself.
 * Therefore it is recommended to get the handle from the service using
 * this function each time it is needed and avoid use of the handle
 * concurrently with functions that might change it.
 *
 * This function is not thread-safe.
 *
 * \param[in] service pointer to the rcl service
 * \return rmw service handle if successful, otherwise NULL
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rmw_service_t *
rcl_service_get_rmw_handle(const rcl_service_t * service);

#if __cplusplus
}
#endif

#endif  // RCL__SERVICE_H_
