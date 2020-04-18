mod app;
#[allow(unused)]
mod gl;
mod input;
mod platform;
mod texture_atlas;

use zerocopy::AsBytes;

use input::InputEvent;

fn main() {
    platform::run("My game", (800, 600), |gl_context: &mut gl::Context| {
        let mut app = app::Application::new(gl_context).unwrap();
        move |dt: f32, input: &[InputEvent], gl_context: &mut gl::Context| {
            app.update(input, dt).unwrap();

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
