#![allow(dead_code)]
// #![allow(unused_imports)]

use gdnative::prelude::*;

mod math;
mod motion_model;
mod simulation;
use simulation::{DemoController, Odometry};

// Function that registers all exposed classes to Godot
fn init_lib(handle: InitHandle) {
    handle.add_class::<DemoController>();
    handle.add_class::<Odometry>();
}

godot_init!(init_lib);
