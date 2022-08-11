use gdnative::prelude::*;

mod hello;
use hello::HelloWorld;

// Function that registers all exposed classes to Godot
fn init_lib(handle: InitHandle) {
    handle.add_class::<HelloWorld>();
}

godot_init!(init_lib);
