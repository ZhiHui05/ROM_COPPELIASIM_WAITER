#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to pure_pursuit_pkg__msg__WayPointPath

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WayPointPath {

    // This member is not documented.
    #[allow(missing_docs)]
    pub closed_path: std_msgs::msg::Bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub points: Vec<geometry_msgs::msg::Point>,

}



impl Default for WayPointPath {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::WayPointPath::default())
  }
}

impl rosidl_runtime_rs::Message for WayPointPath {
  type RmwMsg = super::msg::rmw::WayPointPath;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        closed_path: std_msgs::msg::Bool::into_rmw_message(std::borrow::Cow::Owned(msg.closed_path)).into_owned(),
        points: msg.points
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        closed_path: std_msgs::msg::Bool::into_rmw_message(std::borrow::Cow::Borrowed(&msg.closed_path)).into_owned(),
        points: msg.points
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      closed_path: std_msgs::msg::Bool::from_rmw_message(msg.closed_path),
      points: msg.points
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
    }
  }
}


