//! Application-wide static configuration values.

// The total number of cameras in the system.
pub const NUM_CAMERAS: usize = 5;

// The names of the directories to be created on the SD card for each camera.
pub const CAMERA_PREFIXES: [&str; NUM_CAMERAS] = ["CAM1", "CAM2", "CAM3", "CAM4", "CAM5"];

// The pulse width in microseconds for the camera trigger signal.
pub const TRIGGER_PULSE_WIDTH_US: u32 = 100;

pub const FILENAME_SUFFIX: &str = "JPG";
// The prefix for image filenames.
pub const FILENAME_PREFIX: &str = "IMG_";