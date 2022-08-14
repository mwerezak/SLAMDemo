#![allow(dead_code)]
// #![allow(unused_imports)]

use gdnative::prelude::*;

mod math;
mod motion_model;
mod state_estimation;
mod simulation;
mod api_helpers;

mod demos;

use simulation::{Odometry, GPS};
use demos::pf_localization::LocalizationFilter;
use demos::gauss_2d::Gauss2D;

// Function that registers all exposed classes to Godot
fn init_lib(handle: InitHandle) {
    handle.add_class::<Odometry>();
    handle.add_class::<GPS>();

    handle.add_class::<LocalizationFilter>();
    handle.add_class::<Gauss2D>();
}

godot_init!(init_lib);
