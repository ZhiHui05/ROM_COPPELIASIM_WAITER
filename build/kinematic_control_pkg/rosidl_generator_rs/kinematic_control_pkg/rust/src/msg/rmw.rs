#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "kinematic_control_pkg__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__kinematic_control_pkg__msg__Reference() -> *const std::ffi::c_void;
}

#[link(name = "kinematic_control_pkg__rosidl_generator_c")]
extern "C" {
    fn kinematic_control_pkg__msg__Reference__init(msg: *mut Reference) -> bool;
    fn kinematic_control_pkg__msg__Reference__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Reference>, size: usize) -> bool;
    fn kinematic_control_pkg__msg__Reference__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Reference>);
    fn kinematic_control_pkg__msg__Reference__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Reference>, out_seq: *mut rosidl_runtime_rs::Sequence<Reference>) -> bool;
}

// Corresponds to kinematic_control_pkg__msg__Reference
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Reference {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::rmw::Twist,

}



impl Default for Reference {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !kinematic_control_pkg__msg__Reference__init(&mut msg as *mut _) {
        panic!("Call to kinematic_control_pkg__msg__Reference__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Reference {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { kinematic_control_pkg__msg__Reference__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { kinematic_control_pkg__msg__Reference__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { kinematic_control_pkg__msg__Reference__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Reference {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Reference where Self: Sized {
  const TYPE_NAME: &'static str = "kinematic_control_pkg/msg/Reference";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__kinematic_control_pkg__msg__Reference() }
  }
}


