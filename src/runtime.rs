use crate::gl;

#[cfg(not(target_arch = "wasm32"))]
pub fn run<F: Fn(&mut gl::Context) -> U, U: FnMut(f32, &mut gl::Context) + 'static>(
    title: &str,
    size: (u32, u32),
    f: F,
) {
    use glutin::{
        event,
        event::WindowEvent,
        event_loop::{ControlFlow, EventLoop},
    };
    use std::time::Instant;

    env_logger::init();
    let event_loop = EventLoop::new();
    let mut wb = glutin::window::WindowBuilder::new();
    wb = wb
        .with_title(title)
        .with_inner_size(glutin::dpi::LogicalSize::new(size.0, size.1))
        .with_resizable(false);
    let windowed_context = unsafe {
        glutin::ContextBuilder::new()
            .with_gl(glutin::GlRequest::Specific(glutin::Api::OpenGlEs, (2, 0)))
            .build_windowed(wb, &event_loop)
            .unwrap()
            .make_current()
            .unwrap()
    };

    let mut gl_context =
        gl::Context::from_glow_context(glow::Context::from_loader_function(|addr| {
            windowed_context.get_proc_address(addr)
        }));

    let mut update_fn = f(&mut gl_context);

    let mut last_time = Instant::now();
    event_loop.run(move |event, _, control_flow| {
        *control_flow = ControlFlow::Poll;
        match event {
            event::Event::MainEventsCleared => windowed_context.window().request_redraw(),
            event::Event::WindowEvent {
                event: WindowEvent::Resized(size),
                ..
            } => {
                log::info!("Resize to {:?}", size);
            }
            event::Event::WindowEvent { event, .. } => match event {
                WindowEvent::KeyboardInput {
                    input:
                        event::KeyboardInput {
                            virtual_keycode: Some(event::VirtualKeyCode::Escape),
                            state: event::ElementState::Pressed,
                            ..
                        },
                    ..
                }
                | WindowEvent::CloseRequested => {
                    *control_flow = ControlFlow::Exit;
                }
                _ => {}
            },
            event::Event::RedrawRequested(_) => {
                let now = Instant::now();
                let dt = (now - last_time).as_micros() as f32 / 1_000_000.;
                last_time = now;
                update_fn(dt, &mut gl_context);
                windowed_context.swap_buffers().unwrap();
                gl_context.maintain();
            }
            _ => {}
        }
    });
}

#[cfg(target_arch = "wasm32")]
pub fn run<F: Fn(&mut gl::Context) -> U, U: FnMut(&mut gl::Context) + 'static>(
    title: &str,
    size: (u32, u32),
    f: F,
) {
    use std::{cell::RefCell, rc::Rc};
    use wasm_bindgen::{closure::Closure, JsCast};

    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    console_log::init_with_level(log::Level::Info).unwrap();

    let document = web_sys::window()
        .and_then(|win| win.document())
        .expect("Cannot get document");
    document.set_title(title);

    let canvas = document
        .create_element("canvas")
        .expect("Cannot create canvas")
        .dyn_into::<web_sys::HtmlCanvasElement>()
        .expect("Cannot get canvas element");
    document
        .body()
        .expect("Cannot get document body")
        .append_child(&canvas)
        .expect("Cannot insert canvas into document body");
    canvas
        .set_attribute("width", &format!("{}", size.0))
        .expect("cannot set width");
    canvas
        .set_attribute("height", &format!("{}", size.1))
        .expect("cannot set height");

    let webgl1_context = canvas
        .get_context("webgl")
        .expect("1")
        .expect("2")
        .dyn_into::<web_sys::WebGlRenderingContext>()
        .expect("3");

    let glow_context = glow::Context::from_webgl1_context(webgl1_context);
    let mut gl_context = gl::Context::from_glow_context(glow_context);

    let mut update_fn = f(&mut gl_context);

    let f: Rc<RefCell<Option<Closure<dyn FnMut(f64)>>>> = Rc::new(RefCell::new(None));
    let g = Rc::clone(&f);
    wasm_bindgen_futures::spawn_local(async move {
        *g.borrow_mut() = Some(Closure::wrap(Box::new(move |_: f64| {
            update_fn(&mut gl_context);

            web_sys::window()
                .expect("no global window")
                .request_animation_frame(f.borrow().as_ref().unwrap().as_ref().unchecked_ref())
                .expect("could not request animation frame");
        }) as Box<dyn FnMut(f64)>));

        web_sys::window()
            .expect("no global window")
            .request_animation_frame(g.borrow().as_ref().unwrap().as_ref().unchecked_ref())
            .expect("could not request animation frame");
    })
}
