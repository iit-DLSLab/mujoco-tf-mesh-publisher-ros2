#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "dls2_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dls2_interface__msg__BlindState() -> *const std::ffi::c_void;
}

#[link(name = "dls2_interface__rosidl_generator_c")]
extern "C" {
    fn dls2_interface__msg__BlindState__init(msg: *mut BlindState) -> bool;
    fn dls2_interface__msg__BlindState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BlindState>, size: usize) -> bool;
    fn dls2_interface__msg__BlindState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BlindState>);
    fn dls2_interface__msg__BlindState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BlindState>, out_seq: *mut rosidl_runtime_rs::Sequence<BlindState>) -> bool;
}

// Corresponds to dls2_interface__msg__BlindState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Header

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BlindState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sequence_id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_name: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_position: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_velocity: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_acceleration: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_effort: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_temperature: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feet_contact: rosidl_runtime_rs::Sequence<bool>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_feet_positions: rosidl_runtime_rs::Sequence<f64>,

}



impl Default for BlindState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dls2_interface__msg__BlindState__init(&mut msg as *mut _) {
        panic!("Call to dls2_interface__msg__BlindState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BlindState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dls2_interface__msg__BlindState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dls2_interface__msg__BlindState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dls2_interface__msg__BlindState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BlindState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BlindState where Self: Sized {
  const TYPE_NAME: &'static str = "dls2_interface/msg/BlindState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dls2_interface__msg__BlindState() }
  }
}


