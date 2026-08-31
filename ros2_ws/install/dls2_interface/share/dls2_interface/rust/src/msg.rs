#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to dls2_interface__msg__BlindState
/// Header

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BlindState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sequence_id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_name: Vec<std::string::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_position: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_velocity: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_acceleration: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_effort: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joints_temperature: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feet_contact: Vec<bool>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_feet_positions: Vec<f64>,

}



impl Default for BlindState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::BlindState::default())
  }
}

impl rosidl_runtime_rs::Message for BlindState {
  type RmwMsg = super::msg::rmw::BlindState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        frame_id: msg.frame_id.as_str().into(),
        sequence_id: msg.sequence_id,
        timestamp: msg.timestamp,
        robot_name: msg.robot_name.as_str().into(),
        joints_name: msg.joints_name
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        joints_position: msg.joints_position.into(),
        joints_velocity: msg.joints_velocity.into(),
        joints_acceleration: msg.joints_acceleration.into(),
        joints_effort: msg.joints_effort.into(),
        joints_temperature: msg.joints_temperature.into(),
        feet_contact: msg.feet_contact.into(),
        current_feet_positions: msg.current_feet_positions.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        frame_id: msg.frame_id.as_str().into(),
      sequence_id: msg.sequence_id,
      timestamp: msg.timestamp,
        robot_name: msg.robot_name.as_str().into(),
        joints_name: msg.joints_name
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        joints_position: msg.joints_position.as_slice().into(),
        joints_velocity: msg.joints_velocity.as_slice().into(),
        joints_acceleration: msg.joints_acceleration.as_slice().into(),
        joints_effort: msg.joints_effort.as_slice().into(),
        joints_temperature: msg.joints_temperature.as_slice().into(),
        feet_contact: msg.feet_contact.as_slice().into(),
        current_feet_positions: msg.current_feet_positions.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      frame_id: msg.frame_id.to_string(),
      sequence_id: msg.sequence_id,
      timestamp: msg.timestamp,
      robot_name: msg.robot_name.to_string(),
      joints_name: msg.joints_name
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      joints_position: msg.joints_position
          .into_iter()
          .collect(),
      joints_velocity: msg.joints_velocity
          .into_iter()
          .collect(),
      joints_acceleration: msg.joints_acceleration
          .into_iter()
          .collect(),
      joints_effort: msg.joints_effort
          .into_iter()
          .collect(),
      joints_temperature: msg.joints_temperature
          .into_iter()
          .collect(),
      feet_contact: msg.feet_contact
          .into_iter()
          .collect(),
      current_feet_positions: msg.current_feet_positions
          .into_iter()
          .collect(),
    }
  }
}


