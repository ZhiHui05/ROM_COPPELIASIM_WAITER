#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "pure_pursuit_pkg__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pure_pursuit_pkg__msg__WayPointPath() -> *const std::ffi::c_void;
}

#[link(name = "pure_pursuit_pkg__rosidl_generator_c")]
extern "C" {
    fn pure_pursuit_pkg__msg__WayPointPath__init(msg: *mut WayPointPath) -> bool;
    fn pure_pursuit_pkg__msg__WayPointPath__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<WayPointPath>, size: usize) -> bool;
    fn pure_pursuit_pkg__msg__WayPointPath__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<WayPointPath>);
    fn pure_pursuit_pkg__msg__WayPointPath__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<WayPointPath>, out_seq: *mut rosidl_runtime_rs::Sequence<WayPointPath>) -> bool;
}

// Corresponds to pure_pursuit_pkg__msg__WayPointPath
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WayPointPath {

    // This member is not documented.
    #[allow(missing_docs)]
    pub closed_path: std_msgs::msg::rmw::Bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub points: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,

}



impl Default for WayPointPath {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pure_pursuit_pkg__msg__WayPointPath__init(&mut msg as *mut _) {
        panic!("Call to pure_pursuit_pkg__msg__WayPointPath__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for WayPointPath {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pure_pursuit_pkg__msg__WayPointPath__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pure_pursuit_pkg__msg__WayPointPath__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pure_pursuit_pkg__msg__WayPointPath__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for WayPointPath {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for WayPointPath where Self: Sized {
  const TYPE_NAME: &'static str = "pure_pursuit_pkg/msg/WayPointPath";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pure_pursuit_pkg__msg__WayPointPath() }
  }
}


