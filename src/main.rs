mod app;
#[allow(unused)]
mod gl;
mod runtime;
mod texture_atlas;

use euclid::{
    default::{Rect, Transform2D},
    point2, size2, vec2,
};
use zerocopy::AsBytes;

fn main() {
    runtime::run("My game", (800, 600), |gl_context: &mut gl::Context| {
        let mut app = app::Application::new(gl_context).unwrap();
        move |dt: f32, gl_context: &mut gl::Context| {
            app.update(dt).unwrap();

            app.render(gl_context).unwrap();
        }
    })
}

#[repr(C)]
#[derive(AsBytes)]
struct Vertex {
    position: [f32; 2],
    uv: [f32; 2],
}
