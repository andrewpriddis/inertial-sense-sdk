// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from inertial_sense_ros2:srv/RefLLAUpdate.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "inertial_sense_ros2/srv/detail/ref_lla_update__functions.h"
#include "inertial_sense_ros2/srv/detail/ref_lla_update__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace inertial_sense_ros2
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void RefLLAUpdate_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) inertial_sense_ros2::srv::RefLLAUpdate_Request(_init);
}

void RefLLAUpdate_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<inertial_sense_ros2::srv::RefLLAUpdate_Request *>(message_memory);
  typed_message->~RefLLAUpdate_Request();
}

size_t size_function__RefLLAUpdate_Request__lla(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__RefLLAUpdate_Request__lla(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__RefLLAUpdate_Request__lla(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__RefLLAUpdate_Request__lla(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__RefLLAUpdate_Request__lla(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__RefLLAUpdate_Request__lla(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__RefLLAUpdate_Request__lla(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RefLLAUpdate_Request_message_member_array[1] = {
  {
    "lla",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::srv::RefLLAUpdate_Request, lla),  // bytes offset in struct
    nullptr,  // default value
    size_function__RefLLAUpdate_Request__lla,  // size() function pointer
    get_const_function__RefLLAUpdate_Request__lla,  // get_const(index) function pointer
    get_function__RefLLAUpdate_Request__lla,  // get(index) function pointer
    fetch_function__RefLLAUpdate_Request__lla,  // fetch(index, &value) function pointer
    assign_function__RefLLAUpdate_Request__lla,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RefLLAUpdate_Request_message_members = {
  "inertial_sense_ros2::srv",  // message namespace
  "RefLLAUpdate_Request",  // message name
  1,  // number of fields
  sizeof(inertial_sense_ros2::srv::RefLLAUpdate_Request),
  false,  // has_any_key_member_
  RefLLAUpdate_Request_message_member_array,  // message members
  RefLLAUpdate_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  RefLLAUpdate_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RefLLAUpdate_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RefLLAUpdate_Request_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__srv__RefLLAUpdate_Request__get_type_hash,
  &inertial_sense_ros2__srv__RefLLAUpdate_Request__get_type_description,
  &inertial_sense_ros2__srv__RefLLAUpdate_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace inertial_sense_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate_Request>()
{
  return &::inertial_sense_ros2::srv::rosidl_typesupport_introspection_cpp::RefLLAUpdate_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, srv, RefLLAUpdate_Request)() {
  return &::inertial_sense_ros2::srv::rosidl_typesupport_introspection_cpp::RefLLAUpdate_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/ref_lla_update__functions.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/ref_lla_update__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace inertial_sense_ros2
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void RefLLAUpdate_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) inertial_sense_ros2::srv::RefLLAUpdate_Response(_init);
}

void RefLLAUpdate_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<inertial_sense_ros2::srv::RefLLAUpdate_Response *>(message_memory);
  typed_message->~RefLLAUpdate_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RefLLAUpdate_Response_message_member_array[2] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::srv::RefLLAUpdate_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "message",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::srv::RefLLAUpdate_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RefLLAUpdate_Response_message_members = {
  "inertial_sense_ros2::srv",  // message namespace
  "RefLLAUpdate_Response",  // message name
  2,  // number of fields
  sizeof(inertial_sense_ros2::srv::RefLLAUpdate_Response),
  false,  // has_any_key_member_
  RefLLAUpdate_Response_message_member_array,  // message members
  RefLLAUpdate_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  RefLLAUpdate_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RefLLAUpdate_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RefLLAUpdate_Response_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__srv__RefLLAUpdate_Response__get_type_hash,
  &inertial_sense_ros2__srv__RefLLAUpdate_Response__get_type_description,
  &inertial_sense_ros2__srv__RefLLAUpdate_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace inertial_sense_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate_Response>()
{
  return &::inertial_sense_ros2::srv::rosidl_typesupport_introspection_cpp::RefLLAUpdate_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, srv, RefLLAUpdate_Response)() {
  return &::inertial_sense_ros2::srv::rosidl_typesupport_introspection_cpp::RefLLAUpdate_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/ref_lla_update__functions.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/ref_lla_update__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace inertial_sense_ros2
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void RefLLAUpdate_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) inertial_sense_ros2::srv::RefLLAUpdate_Event(_init);
}

void RefLLAUpdate_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<inertial_sense_ros2::srv::RefLLAUpdate_Event *>(message_memory);
  typed_message->~RefLLAUpdate_Event();
}

size_t size_function__RefLLAUpdate_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<inertial_sense_ros2::srv::RefLLAUpdate_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RefLLAUpdate_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<inertial_sense_ros2::srv::RefLLAUpdate_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__RefLLAUpdate_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<inertial_sense_ros2::srv::RefLLAUpdate_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__RefLLAUpdate_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const inertial_sense_ros2::srv::RefLLAUpdate_Request *>(
    get_const_function__RefLLAUpdate_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<inertial_sense_ros2::srv::RefLLAUpdate_Request *>(untyped_value);
  value = item;
}

void assign_function__RefLLAUpdate_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<inertial_sense_ros2::srv::RefLLAUpdate_Request *>(
    get_function__RefLLAUpdate_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const inertial_sense_ros2::srv::RefLLAUpdate_Request *>(untyped_value);
  item = value;
}

void resize_function__RefLLAUpdate_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<inertial_sense_ros2::srv::RefLLAUpdate_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RefLLAUpdate_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<inertial_sense_ros2::srv::RefLLAUpdate_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RefLLAUpdate_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<inertial_sense_ros2::srv::RefLLAUpdate_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__RefLLAUpdate_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<inertial_sense_ros2::srv::RefLLAUpdate_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__RefLLAUpdate_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const inertial_sense_ros2::srv::RefLLAUpdate_Response *>(
    get_const_function__RefLLAUpdate_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<inertial_sense_ros2::srv::RefLLAUpdate_Response *>(untyped_value);
  value = item;
}

void assign_function__RefLLAUpdate_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<inertial_sense_ros2::srv::RefLLAUpdate_Response *>(
    get_function__RefLLAUpdate_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const inertial_sense_ros2::srv::RefLLAUpdate_Response *>(untyped_value);
  item = value;
}

void resize_function__RefLLAUpdate_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<inertial_sense_ros2::srv::RefLLAUpdate_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RefLLAUpdate_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::srv::RefLLAUpdate_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(inertial_sense_ros2::srv::RefLLAUpdate_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__RefLLAUpdate_Event__request,  // size() function pointer
    get_const_function__RefLLAUpdate_Event__request,  // get_const(index) function pointer
    get_function__RefLLAUpdate_Event__request,  // get(index) function pointer
    fetch_function__RefLLAUpdate_Event__request,  // fetch(index, &value) function pointer
    assign_function__RefLLAUpdate_Event__request,  // assign(index, value) function pointer
    resize_function__RefLLAUpdate_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(inertial_sense_ros2::srv::RefLLAUpdate_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__RefLLAUpdate_Event__response,  // size() function pointer
    get_const_function__RefLLAUpdate_Event__response,  // get_const(index) function pointer
    get_function__RefLLAUpdate_Event__response,  // get(index) function pointer
    fetch_function__RefLLAUpdate_Event__response,  // fetch(index, &value) function pointer
    assign_function__RefLLAUpdate_Event__response,  // assign(index, value) function pointer
    resize_function__RefLLAUpdate_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RefLLAUpdate_Event_message_members = {
  "inertial_sense_ros2::srv",  // message namespace
  "RefLLAUpdate_Event",  // message name
  3,  // number of fields
  sizeof(inertial_sense_ros2::srv::RefLLAUpdate_Event),
  false,  // has_any_key_member_
  RefLLAUpdate_Event_message_member_array,  // message members
  RefLLAUpdate_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  RefLLAUpdate_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RefLLAUpdate_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RefLLAUpdate_Event_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__srv__RefLLAUpdate_Event__get_type_hash,
  &inertial_sense_ros2__srv__RefLLAUpdate_Event__get_type_description,
  &inertial_sense_ros2__srv__RefLLAUpdate_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace inertial_sense_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate_Event>()
{
  return &::inertial_sense_ros2::srv::rosidl_typesupport_introspection_cpp::RefLLAUpdate_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, srv, RefLLAUpdate_Event)() {
  return &::inertial_sense_ros2::srv::rosidl_typesupport_introspection_cpp::RefLLAUpdate_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/ref_lla_update__functions.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/ref_lla_update__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace inertial_sense_ros2
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers RefLLAUpdate_service_members = {
  "inertial_sense_ros2::srv",  // service namespace
  "RefLLAUpdate",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t RefLLAUpdate_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RefLLAUpdate_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<inertial_sense_ros2::srv::RefLLAUpdate>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<inertial_sense_ros2::srv::RefLLAUpdate>,
  &inertial_sense_ros2__srv__RefLLAUpdate__get_type_hash,
  &inertial_sense_ros2__srv__RefLLAUpdate__get_type_description,
  &inertial_sense_ros2__srv__RefLLAUpdate__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace inertial_sense_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::inertial_sense_ros2::srv::rosidl_typesupport_introspection_cpp::RefLLAUpdate_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::inertial_sense_ros2::srv::RefLLAUpdate_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::inertial_sense_ros2::srv::RefLLAUpdate_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::inertial_sense_ros2::srv::RefLLAUpdate_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, srv, RefLLAUpdate)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<inertial_sense_ros2::srv::RefLLAUpdate>();
}

#ifdef __cplusplus
}
#endif