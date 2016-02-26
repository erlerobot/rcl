// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCL__CLIENT_H_
#define RCL__CLIENT_H_

#if __cplusplus
extern "C"
{
#endif

#include "rosidl_generator_c/service_type_support.h"

#include "rcl/macros.h"
#include "rcl/node.h"
#include "rcl/visibility_control.h"

/// Internal rcl client implementation struct.
struct rcl_client_impl_t;

/// Handle for a rcl client.
typedef struct rcl_client_t
{
  struct rcl_client_impl_t * impl;
} rcl_client_t;

/// Options available for a rcl client.
typedef struct rcl_client_options_t
{
  /// Middleware quality of service settings for the client.
  rmw_qos_profile_t qos;
  /// Custom allocator for the client, used for incidental allocations.
  /* For default behavior (malloc/free), use: rcl_get_default_allocator() */
  rcl_allocator_t allocator;
} rcl_client_options_t;

/// Return a rcl_client_t struct with members set to NULL.
/* Should be called to get a null rcl_client_t before passing to
 * rcl_initalize_client().
 * It's also possible to use calloc() instead of this if the rcl_client is
 * being allocated on the heap.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_client_t
rcl_get_zero_initialized_client(void);


// TODO Rewrite this comment for clients
/// Initialize a rcl client.
/* After calling this function on a rcl_client_t, it can be used to publish
 * messages of the given type to the given topic using rcl_publish().
 *
 * The given rcl_node_t must be valid and the resulting rcl_client_t is only
 * valid as long as the given rcl_node_t remains valid.
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
 * well as a custom allocator which is used when initializing/finalizing the
 * client to allocate space for incidentals, e.g. the topic name string.
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
 *    rcl_client_t client = rcl_get_zero_initialized_client();
 *    rcl_client_options_t client_ops = rcl_client_get_default_options();
 *    ret = rcl_client_init(&client, &node, ts, "chatter", &client_ops);
 *    // ... error handling, and on shutdown do finalization:
 *    ret = rcl_client_fini(&client, &node);
 *    // ... error handling for rcl_client_fini()
 *    ret = rcl_node_fini(&node);
 *    // ... error handling for rcl_deinitialize_node()
 *
 * This function is not thread-safe.
 *
 * \param[inout] client preallocated client structure
 * \param[in] node valid rcl node handle
 * \param[in] type_support type support object for the topic's type
 * \param[in] service_name the name of the topic to publish on
 * \param[in] options client options, including quality of service settings
 * \return RCL_RET_OK if the client was initialized successfully, or
 *         RCL_RET_NODE_INVALID if the node is invalid, or
 *         RCL_RET_ALREADY_INIT if the client is already initialized, or
 *         RCL_RET_INVALID_ARGUMENT if any arguments are invalid, or
 *         RCL_RET_BAD_ALLOC if allocating memory fails, or
 *         RCL_RET_ERROR if an unspecified error occurs.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_client_init(
  rcl_client_t * client,
  const rcl_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rcl_client_options_t * options);

// TODO Redo this comment for clients
/// Finalize a rcl_client_t.
/* After calling, the node will no longer be advertising that it is publishing
 * on this topic (assuming this is the only client on this topic).
 *
 * After calling, calls to rcl_publish will fail when using this client.
 * However, the given node handle is still valid.
 *
 * This function is not thread-safe.
 *
 * \param[inout] client handle to the client to be finalized
 * \param[in] node handle to the node used to create the client
 * \return RCL_RET_OK if client was finalized successfully, or
 *         RCL_RET_INVALID_ARGUMENT if any arguments are invalid, or
 *         RCL_RET_ERROR if an unspecified error occurs.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_client_fini(rcl_client_t * client, rcl_node_t * node);

/// Return the default client options in a rcl_client_options_t.
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_client_options_t
rcl_client_get_default_options(void);

// TODO Redo  this comment
/// Send a ROS request using a client.
/* It is the job of the caller to ensure that the type of the ros_message
 * parameter and the type associate with the client (via the type support)
 * match.
 * Passing a different type to publish produces undefined behavior and cannot
 * be checked by this function and therefore no deliberate error will occur.
 *
 * TODO(wjwwood): update after researching what the blocking behavior is
 *                or optionally link to a document that describes blocking
 *                behavior is more detail.
 * Calling rcl_publish is a potentially blocking call.
 * When called rcl_publish will immediately do any publishing related work,
 * including, but not limited to, converting the message into a different type,
 * serializing the message, collecting publish statistics, etc.
 * The last thing it will do is call the underlying middleware's publish
 * function which may or may not block based on the quality of service settings
 * given via the client options in rcl_client_init().
 * For example, if the reliability is set to reliable, then a publish may block
 * until space in the publish queue is available, but if the reliability is set
 * to best effort then it should not block.
 *
 * The ROS message given by the ros_message void pointer is always owned by the
 * calling code, but should remain constant during publish.
 *
 * This function is thread safe so long as access to both the client and the
 * ros_message is synchronized.
 * That means that calling rcl_publish from multiple threads is allowed, but
 * calling rcl_publish at the same time as non-thread safe client functions
 * is not, e.g. calling rcl_publish and rcl_client_fini concurrently
 * is not allowed.
 * Before calling rcl_publish the message can change and after calling
 * rcl_publish the message can change, but it cannot be changed during the
 * publish call.
 * The same ros_message, however, can be passed to multiple calls of
 * rcl_publish simultaneously, even if the clients differ.
 * The ros_message is unmodified by rcl_publish.
 *
 * \param[in] client handle to the client which will do the publishing
 * \param[in] ros_message type-erased pointer to the ROS message
 * \return RCL_RET_OK if the message was published successfully, or
 *         RCL_RET_INVALID_ARGUMENT if any arguments are invalid, or
 *         RCL_RET_CLIENT_INVALID if the client is invalid, or
 *         RCL_RET_ERROR if an unspecified error occurs.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_send_request(const rcl_client_t * client, const void * ros_request);


// TODO Comment
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_take_response(const rcl_client_t * client, void * request_header, void * ros_response);

// TODO Redo comment
/// Get the topic name for the client.
/* This function returns the client's internal topic name string.
 * This function can fail, and therefore return NULL, if the:
 *   - client is NULL
 *   - client is invalid (never called init, called fini, or invalid node)
 *
 * The returned string is only valid as long as the rcl_client_t is valid.
 * The value of the string may change if the topic name changes, and therefore
 * copying the string is recommended if this is a concern.
 *
 * This function is not thread-safe, and copying the result is not thread-safe.
 *
 * \param[in] client pointer to the client
 * \return name string if successful, otherwise NULL
 */
RCL_PUBLIC
RCL_WARN_UNUSED
const char *
rcl_client_get_service_name(const rcl_client_t * client);

/// Return the rcl client options.
/* This function returns the client's internal options struct.
 * This function can fail, and therefore return NULL, if the:
 *   - client is NULL
 *   - client is invalid (never called init, called fini, or invalid node)
 *
 * The returned struct is only valid as long as the rcl_client_t is valid.
 * The values in the struct may change if the options of the client change,
 * and therefore copying the struct is recommended if this is a concern.
 *
 * This function is not thread-safe, and copying the result is not thread-safe.
 *
 * \param[in] client pointer to the client
 * \return options struct if successful, otherwise NULL
 */
RCL_PUBLIC
RCL_WARN_UNUSED
const rcl_client_options_t *
rcl_client_get_options(const rcl_client_t * client);

/// Return the rmw client handle.
/* The handle returned is a pointer to the internally held rmw handle.
 * This function can fail, and therefore return NULL, if the:
 *   - client is NULL
 *   - client is invalid (never called init, called fini, or invalid node)
 *
 * The returned handle is made invalid if the client is finalized or if
 * rcl_shutdown() is called.
 * The returned handle is not guaranteed to be valid for the life time of the
 * client as it may be finalized and recreated itself.
 * Therefore it is recommended to get the handle from the client using
 * this function each time it is needed and avoid use of the handle
 * concurrently with functions that might change it.
 *
 * \param[in] client pointer to the rcl client
 * \return rmw client handle if successful, otherwise NULL
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rmw_client_t *
rcl_client_get_rmw_handle(const rcl_client_t * client);

#if __cplusplus
}
#endif

#endif  // RCL__CLIENT_H_
