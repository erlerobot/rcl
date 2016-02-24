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

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include "rcl/service.h"

#include "rcl/rcl.h"

#include "sensor_msgs/set_camera_info.h"

#include "../memory_tools/memory_tools.hpp"
#include "../scope_exit.hpp"
#include "rcl/error_handling.h"

class TestServiceFixture : public ::testing::Test
{
public:
  rcl_node_t * node_ptr;
  void SetUp()
  {
    stop_memory_checking();
    rcl_ret_t ret;
    ret = rcl_init(0, nullptr, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    this->node_ptr = new rcl_node_t;
    *this->node_ptr = rcl_get_zero_initialized_node();
    const char * name = "node_name";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(this->node_ptr, name, &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    set_on_unexpected_malloc_callback([]() {ASSERT_FALSE(true) << "UNEXPECTED MALLOC";});
    set_on_unexpected_realloc_callback([]() {ASSERT_FALSE(true) << "UNEXPECTED REALLOC";});
    set_on_unexpected_free_callback([]() {ASSERT_FALSE(true) << "UNEXPECTED FREE";});
    start_memory_checking();
  }

  void TearDown()
  {
    assert_no_malloc_end();
    assert_no_realloc_end();
    assert_no_free_end();
    stop_memory_checking();
    set_on_unexpected_malloc_callback(nullptr);
    set_on_unexpected_realloc_callback(nullptr);
    set_on_unexpected_free_callback(nullptr);
    rcl_ret_t ret = rcl_node_fini(this->node_ptr);
    delete this->node_ptr;
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    ret = rcl_shutdown();
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  }
};

// TODO ??? Will this work?
void
wait_for_service_to_be_ready(
  rcl_service_t * service,
  size_t max_tries,
  int64_t period_ms,
  bool & success)
{
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t ret = rcl_wait_set_init(&wait_set, 1, 0, 0, rcl_get_default_allocator());
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  auto wait_set_exit = make_scope_exit([&wait_set]() {
    stop_memory_checking();
    rcl_ret_t ret = rcl_wait_set_fini(&wait_set);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  });
  size_t iteration = 0;
  do {
    ++iteration;
    ret = rcl_wait_set_clear_services(&wait_set);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    ret = rcl_wait_set_add_service(&wait_set, service);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    ret = rcl_wait(&wait_set, RCL_MS_TO_NS(period_ms));
    if (ret == RCL_RET_TIMEOUT) {
      continue;
    }
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    for (size_t i = 0; i < wait_set.size_of_services; ++i) {
      if (wait_set.services[i] && wait_set.services[i] == service) {
        success = true;
        return;
      }
    }
  } while (iteration < max_tries);
  success = false;
}

/* Basic nominal test of a service.
 */
TEST_F(TestServiceFixture, test_service_nominal) {
  stop_memory_checking();
  rcl_ret_t ret;
  rcl_client_t client = rcl_get_zero_initialized_client();
  const rosidl_message_type_support_t * ts = ROSIDL_GET_TYPE_SUPPORT(sensor_msgs, srv, SetCameraInfo);
  // TODO(wjwwood): Change this back to just chatter when this OpenSplice problem is resolved:
  //  ========================================================================================
  //  Report      : WARNING
  //  Date        : Wed Feb 10 18:17:03 PST 2016
  //  Description : Create Topic "chatter" failed: typename <std_msgs::msg::dds_::Int64_>
  //                differs exiting definition <std_msgs::msg::dds_::String_>.
  //  Node        : farl
  //  Process     : test_service__rmw_opensplice_cpp <23524>
  //  Thread      : main thread 7fff7342d000
  //  Internals   : V6.4.140407OSS///v_topicNew/v_topic.c/448/21/1455157023.781423000
  const char * topic = "add_two_ints";
  rcl_client_options_t client_options = rcl_client_get_default_options();
  ret = rcl_client_init(&client, this->node_ptr, ts, topic, &client_options);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  auto client_exit = make_scope_exit([&client, this]() {
    stop_memory_checking();
    rcl_ret_t ret = rcl_client_fini(&client, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  });
  rcl_service_t service = rcl_get_zero_initialized_service();
  rcl_service_options_t service_options = rcl_service_get_default_options();
  ret = rcl_service_init(&service, this->node_ptr, ts, topic, &service_options);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  auto service_exit = make_scope_exit([&service, this]() {
    stop_memory_checking();
    rcl_ret_t ret = rcl_service_fini(&service, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  });
  // TODO(wjwwood): add logic to wait for the connection to be established
  //                probably using the count_services busy wait mechanism
  //                until then we will sleep for a short period of time
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  sensor_msgs__srv__SetCameraInfo__request client_request;
  sensor_msgs__srv__SetCameraInfo__request__init(&client_request);
  msg.data = 42;
  ret = rcl_send_request(&client, &client_request, 0);
  sensor_msgs__srv__SetCameraInfo__request__fini(&client_request);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();

  bool success;
  wait_for_service_to_be_ready(&service, 10, 100, success);
  ASSERT_TRUE(success);
  // consider putting this in a function for an independent scope
  {
    sensor_msgs__srv__SetCameraInfo__response service_response;
    sensor_msgs__srv__SetCameraInfo__response__init(&service_response);
    auto msg_exit = make_scope_exit([&msg]() {
      stop_memory_checking();
      sensor_msgs__srv__SetCameraInfo__response__fini(&service_response);
    });
    bool taken = false;

    sensor_msgs__srv__SetCameraInfo__request service_request;
    sensor_msgs__srv__SetCameraInfo__request__init(&service_request);
    ret = rcl_handle_request(&service, &msg, service_request, service_response);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();

    // TODO expectations
    ASSERT_EQ(42, msg.data);
    ret = rcl_send_response(&service, &msg, service_response);
  }

  // TODO wait for client
  sensor_msgs__srv__SetCameraInfo__response client_response;
  sensor_msgs__srv__SetCameraInfo__response__init(&client_response);

  rcl_handle_response(client, &client_response);
  // TODO expectations
}
