use anyhow::Error;
use euclid::{
    default::{Point2D, Rect, Size2D, Transform2D},
    point2, size2, vec2,
};
use zerocopy::AsBytes;

use crate::{gl, texture_atlas::TextureAtlas};

const TEXTURE_ATLAS_SIZE: Size2D<u32> = Size2D {
    width: 1024,
    height: 1024,
    _unit: std::marker::PhantomData::<euclid::UnknownUnit>,
};

pub struct Application {
    assets: Assets,

    texture_atlas: TextureAtlas,
    texture: gl::Texture,

    program: gl::Program,
    borders_buffer: gl::VertexBuffer,
}

impl Application {
    pub fn new(gl_context: &mut gl::Context) -> Result<Self, Error> {
        let mut texture_atlas =
            TextureAtlas::new((TEXTURE_ATLAS_SIZE.width, TEXTURE_ATLAS_SIZE.height));
        let mut texture = gl_context.create_texture(
            gl::TextureFormat::RGBAFloat,
            TEXTURE_ATLAS_SIZE.width,
            TEXTURE_ATLAS_SIZE.height,
        )?;

        let borders_image = image::load_from_memory(include_bytes!("../assets/borders.png"))
            .unwrap()
            .to_rgba();
        let borders = texture_atlas
            .add_texture((borders_image.width(), borders_image.height()))
            .unwrap();
        log::info!(
            "w: {}, h: {}",
            borders_image.width(),
            borders_image.height()
        );
        texture.write(
            borders[0],
            borders[1],
            borders[2] - borders[0],
            borders[3] - borders[1],
            &borders_image.into_raw(),
        );

        let assets = Assets { borders };

        let vertex_shader = gl_context
            .create_shader(gl::ShaderType::Vertex, include_str!("shaders/shader.vert"))
            .unwrap();
        let fragment_shader = gl_context
            .create_shader(
                gl::ShaderType::Fragment,
                include_str!("shaders/shader.frag"),
            )
            .unwrap();

        let mut program = gl_context
            .create_program(&gl::ProgramDescriptor {
                vertex_shader: &vertex_shader,
                fragment_shader: &fragment_shader,
                uniforms: &[
                    gl::UniformEntry {
                        name: "u_transform",
                        ty: gl::UniformType::Mat3,
                    },
                    gl::UniformEntry {
                        name: "u_texture",
                        ty: gl::UniformType::Texture,
                    },
                ],
                vertex_format: gl::VertexFormat {
                    stride: std::mem::size_of::<Vertex>(),
                    attributes: &[
                        gl::VertexAttribute {
                            name: "a_pos",
                            ty: gl::VertexAttributeType::Float,
                            size: 2,
                            offset: 0,
                        },
                        gl::VertexAttribute {
                            name: "a_uv",
                            ty: gl::VertexAttributeType::Float,
                            size: 2,
                            offset: 2 * 4,
                        },
                    ],
                },
            })
            .unwrap();

        let transform = Transform2D::create_scale(1.0 / 800.0, 1.0 / 600.0)
            .post_scale(3., 3.)
            .post_scale(2., 2.)
            .post_translate(vec2(-1.0, -1.0));
        program
            .set_uniform(
                0,
                gl::Uniform::Mat3([
                    [transform.m11, transform.m12, 0.0],
                    [transform.m21, transform.m22, 0.0],
                    [transform.m31, transform.m32, 1.0],
                ]),
            )
            .unwrap();
        program
            .set_uniform(1, gl::Uniform::Texture(&texture))
            .unwrap();

        let mut border_vertices = [Vertex::zero(); 6];
        render_quad(point2(0.0, 0.0), assets.borders, &mut border_vertices);
        let mut borders_buffer = gl_context.create_vertex_buffer()?;
        borders_buffer.write(&border_vertices);

        Ok(Self {
            texture_atlas,
            texture,
            assets,

            program,
            borders_buffer,
        })
    }

    pub fn update(&mut self, dt: f32) -> Result<(), Error> {
        Ok(())
    }

    pub fn render(&mut self, gl_context: &mut gl::Context) -> Result<(), Error> {
        gl_context.clear([91. / 255., 164. / 255., 244. / 255., 1.0]);

        self.program.render_vertices(&self.borders_buffer)?;

        Ok(())
    }
}

struct Assets {
    borders: [u32; 4],
}
#[repr(C)]
#[derive(Clone, Copy, Debug, AsBytes)]
struct Vertex {
    position: [f32; 2],
    uv: [f32; 2],
}

impl Vertex {
    pub fn zero() -> Self {
        Self {
            position: [0.0, 0.0],
            uv: [0.0, 0.0],
        }
    }
}

fn render_quad(position: Point2D<f32>, tex_coords: [u32; 4], out: &mut [Vertex]) {
    let size = size2(
        (tex_coords[2] - tex_coords[0]) as f32,
        (tex_coords[3] - tex_coords[1]) as f32,
    );
    let vertex_rect = Rect::new(position, size);

    let uv_pos = point2(
        tex_coords[0] as f32 / TEXTURE_ATLAS_SIZE.width as f32,
        tex_coords[1] as f32 / TEXTURE_ATLAS_SIZE.height as f32,
    );
    let uv_size = size2(
        (tex_coords[2] - tex_coords[0]) as f32 / TEXTURE_ATLAS_SIZE.width as f32,
        (tex_coords[3] - tex_coords[1]) as f32 / TEXTURE_ATLAS_SIZE.height as f32,
    );
    let uv_rect = Rect::new(uv_pos, uv_size);

    out[0..6].copy_from_slice(&[
        Vertex {
            position: vertex_rect.min().to_array(),
            uv: [uv_rect.min_x(), uv_rect.max_y()],
        },
        Vertex {
            position: [vertex_rect.max_x(), vertex_rect.min_y()],
            uv: [uv_rect.max_x(), uv_rect.max_y()],
        },
        Vertex {
            position: [vertex_rect.min_x(), vertex_rect.max_y()],
            uv: [uv_rect.min_x(), uv_rect.min_y()],
        },
        Vertex {
            position: [vertex_rect.max_x(), vertex_rect.min_y()],
            uv: [uv_rect.max_x(), uv_rect.max_y()],
        },
        Vertex {
            position: vertex_rect.max().to_array(),
            uv: [uv_rect.max_x(), uv_rect.min_y()],
        },
        Vertex {
            position: [vertex_rect.min_x(), vertex_rect.max_y()],
            uv: [uv_rect.min_x(), uv_rect.min_y()],
        },
    ]);
}
