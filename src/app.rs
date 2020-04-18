use anyhow::Error;
use euclid::{
    default::{Point2D, Rect, Size2D, Transform2D, Vector2D},
    point2, size2, vec2, Angle,
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
    entities_buffer: gl::VertexBuffer,
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

        let assets = Assets {
            borders: load_image(
                include_bytes!("../assets/borders.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            frog_body: load_image(
                include_bytes!("../assets/frog_body.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            frog_leg_upper_left: load_image(
                include_bytes!("../assets/frog_leg_upper_left.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            frog_leg_lower_left: load_image(
                include_bytes!("../assets/frog_leg_lower_left.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            frog_leg_upper_right: load_image(
                include_bytes!("../assets/frog_leg_upper_right.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            frog_leg_lower_right: load_image(
                include_bytes!("../assets/frog_leg_lower_right.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            frog_foot: load_image(
                include_bytes!("../assets/frog_foot.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            lily_pad: load_image(
                include_bytes!("../assets/lily_pad.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
        };

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

        let transform = Transform2D::create_scale(1. / 800., 1. / 600.)
            .post_scale(3., 3.)
            .post_scale(2., 2.)
            .post_translate(vec2(-1., -1.));
        program
            .set_uniform(
                0,
                gl::Uniform::Mat3([
                    [transform.m11, transform.m12, 0.],
                    [transform.m21, transform.m22, 0.],
                    [transform.m31, transform.m32, 1.],
                ]),
            )
            .unwrap();
        program
            .set_uniform(1, gl::Uniform::Texture(&texture))
            .unwrap();

        let mut border_vertices = Vec::new();
        render_quad(point2(0., 0.), assets.borders, &mut border_vertices);
        let mut borders_buffer = gl_context.create_vertex_buffer()?;
        borders_buffer.write(&border_vertices);

        Ok(Self {
            texture_atlas,
            texture,
            assets,

            program,
            borders_buffer,
            entities_buffer: gl_context.create_vertex_buffer()?,
        })
    }

    pub fn update(&mut self, dt: f32) -> Result<(), Error> {
        Ok(())
    }

    pub fn render(&mut self, gl_context: &mut gl::Context) -> Result<(), Error> {
        gl_context.clear([91. / 255., 164. / 255., 244. / 255., 1.]);

        let mut frog = Frog::new(&self.assets);

        let mut entities = Vec::new();

        frog.render(&mut entities);

        self.program.render_vertices(&self.borders_buffer)?;

        self.entities_buffer.write(&entities);
        self.program.render_vertices(&self.entities_buffer)?;

        Ok(())
    }
}

type TextureRect = [u32; 4];

struct Assets {
    borders: TextureRect,
    frog_body: TextureRect,
    frog_leg_upper_left: TextureRect,
    frog_leg_lower_left: TextureRect,
    frog_leg_upper_right: TextureRect,
    frog_leg_lower_right: TextureRect,
    frog_foot: TextureRect,
    lily_pad: TextureRect,
}

struct Frog {
    position: Point2D<f32>,

    lily_pad: Sprite,
    body: (Point2D<f32>, Angle<f32>, Sprite),
    leg_upper_left: (Point2D<f32>, Angle<f32>, Sprite),
    leg_lower_left: (Point2D<f32>, Angle<f32>, Sprite),
    foot_left: (Point2D<f32>, Angle<f32>, Sprite),
    leg_upper_right: (Point2D<f32>, Angle<f32>, Sprite),
    leg_lower_right: (Point2D<f32>, Angle<f32>, Sprite),
    foot_right: (Point2D<f32>, Angle<f32>, Sprite),
}

impl Frog {
    pub fn new(assets: &Assets) -> Self {
        let foot = Sprite::new(assets.frog_foot, point2(3., 2.));

        let body_anchor = point2(14.0, 14.0);
        let body = (
            body_anchor,
            Angle::degrees(0.),
            Sprite::new(assets.frog_body, point2(9., 10.)),
        );

        let left_upper_anchor = point2(5., 2.);
        let leg_upper_left = (
            left_upper_anchor,
            Angle::degrees(-120.),
            Sprite::new(assets.frog_leg_upper_left, point2(4., 2.)),
        );
        let left_lower_anchor = point2(2., 9.);
        let leg_lower_left = (
            left_lower_anchor,
            Angle::degrees(120.),
            Sprite::new(assets.frog_leg_lower_left, point2(3., 10.)),
        );
        let left_foot_anchor = point2(2., 1.);
        let foot_left = (left_foot_anchor, Angle::degrees(180.), foot.clone());

        let right_upper_anchor = point2(13., 2.);
        let leg_upper_right = (
            right_upper_anchor,
            Angle::degrees(0.),
            Sprite::new(assets.frog_leg_upper_right, point2(2., 2.)),
        );
        let right_lower_anchor = point2(4., 9.);
        let leg_lower_right = (
            right_lower_anchor,
            Angle::degrees(0.),
            Sprite::new(assets.frog_leg_lower_right, point2(2., 10.)),
        );
        let right_foot_anchor = point2(4., 1.);
        let foot_right = (right_foot_anchor, Angle::degrees(0.), foot);

        Self {
            position: point2(100., 50.),

            lily_pad: Sprite::new(assets.lily_pad, point2(14., 14.)),
            body,
            leg_upper_left,
            leg_lower_left,
            foot_left,
            leg_upper_right,
            leg_lower_right,
            foot_right,
        }
    }

    pub fn render(&mut self, out: &mut Vec<Vertex>) {
        self.lily_pad
            .set_transform(Transform2D::create_rotation(Angle::degrees(45.)));

        let anchor_sprite =
            |sprite: &mut Sprite, anchor: Point2D<f32>, angle: Angle<f32>, parent: &Sprite| {
                sprite.set_transform(
                    Transform2D::create_rotation(angle)
                        .post_translate(anchor.to_vector())
                        .post_transform(parent.transform()),
                );
            };

        // the body bone's connected to the.. lily pad bone!
        anchor_sprite(&mut self.body.2, self.body.0, self.body.1, &self.lily_pad);

        // the upper leg bone's connected to the.. body bone!
        anchor_sprite(
            &mut self.leg_upper_left.2,
            self.leg_upper_left.0,
            self.leg_upper_left.1,
            &self.body.2,
        );
        // the lower leg bone's connected to the.. upper leg bone bone!
        anchor_sprite(
            &mut self.leg_lower_left.2,
            self.leg_lower_left.0,
            self.leg_lower_left.1,
            &self.leg_upper_left.2,
        );
        // the foot bone's connected to the.. lower leg bone bone!
        anchor_sprite(
            &mut self.foot_left.2,
            self.foot_left.0,
            self.foot_left.1,
            &self.leg_lower_left.2,
        );

        // the upper leg bone's connected to the.. body bone!
        anchor_sprite(
            &mut self.leg_upper_right.2,
            self.leg_upper_right.0,
            self.leg_upper_right.1,
            &self.body.2,
        );
        // the lower leg bone's connected to the.. upper leg bone bone!
        anchor_sprite(
            &mut self.leg_lower_right.2,
            self.leg_lower_right.0,
            self.leg_lower_right.1,
            &self.leg_upper_right.2,
        );
        // the foot bone's connected to the.. lower leg bone bone!
        anchor_sprite(
            &mut self.foot_right.2,
            self.foot_right.0,
            self.foot_right.1,
            &self.leg_lower_right.2,
        );

        render_sprite(&self.lily_pad, self.position, out);
        render_sprite(&self.body.2, self.position, out);
        render_sprite(&self.foot_left.2, self.position, out);
        render_sprite(&self.foot_right.2, self.position, out);
        render_sprite(&self.leg_lower_left.2, self.position, out);
        render_sprite(&self.leg_lower_right.2, self.position, out);
        render_sprite(&self.leg_upper_left.2, self.position, out);
        render_sprite(&self.leg_upper_right.2, self.position, out);
    }
}

#[derive(Clone)]
struct Sprite {
    image: TextureRect,
    origin: Point2D<f32>,
    transform: Transform2D<f32>,
}

impl Sprite {
    fn new(image: TextureRect, origin: Point2D<f32>) -> Self {
        Self {
            image,
            origin,
            transform: Transform2D::create_translation(-origin.x, -origin.y),
        }
    }

    fn set_transform(&mut self, t: Transform2D<f32>) {
        self.transform =
            Transform2D::create_translation(-self.origin.x, -self.origin.y).post_transform(&t);
    }
}

impl Sprite {
    fn transform(&self) -> &Transform2D<f32> {
        &self.transform
    }
}

struct GameState {}

#[repr(C)]
#[derive(Clone, Copy, Debug, AsBytes)]
struct Vertex {
    position: [f32; 2],
    uv: [f32; 2],
}

fn load_image(
    image_bytes: &[u8],
    texture_atlas: &mut TextureAtlas,
    texture: &mut gl::Texture,
) -> Result<TextureRect, Error> {
    let image = image::load_from_memory(image_bytes).unwrap().to_rgba();
    let texture_coords = texture_atlas
        .add_texture((image.width(), image.height()))
        .unwrap();
    texture.write(
        texture_coords[0],
        texture_coords[1],
        texture_coords[2] - texture_coords[0],
        texture_coords[3] - texture_coords[1],
        &image.into_raw(),
    );
    Ok(texture_coords)
}

fn render_sprite(sprite: &Sprite, position: Point2D<f32>, out: &mut Vec<Vertex>) {
    let size = size2(
        (sprite.image[2] - sprite.image[0]) as f32,
        (sprite.image[3] - sprite.image[1]) as f32,
    );
    let vertex_rect = Rect::new(point2(0., 0.), size);

    let uv_pos = point2(
        sprite.image[0] as f32 / TEXTURE_ATLAS_SIZE.width as f32,
        sprite.image[1] as f32 / TEXTURE_ATLAS_SIZE.height as f32,
    );
    let uv_size = size2(
        (sprite.image[2] - sprite.image[0]) as f32 / TEXTURE_ATLAS_SIZE.width as f32,
        (sprite.image[3] - sprite.image[1]) as f32 / TEXTURE_ATLAS_SIZE.height as f32,
    );
    let uv_rect = Rect::new(uv_pos, uv_size);

    let transform = |p: Point2D<f32>| -> [f32; 2] {
        (position + sprite.transform().transform_point(p).to_vector()).to_array()
    };
    out.extend_from_slice(&[
        Vertex {
            position: transform(vertex_rect.min()),
            uv: [uv_rect.min_x(), uv_rect.max_y()],
        },
        Vertex {
            position: transform(point2(vertex_rect.max_x(), vertex_rect.min_y())),
            uv: [uv_rect.max_x(), uv_rect.max_y()],
        },
        Vertex {
            position: transform(point2(vertex_rect.min_x(), vertex_rect.max_y())),
            uv: [uv_rect.min_x(), uv_rect.min_y()],
        },
        Vertex {
            position: transform(point2(vertex_rect.max_x(), vertex_rect.min_y())),
            uv: [uv_rect.max_x(), uv_rect.max_y()],
        },
        Vertex {
            position: transform(vertex_rect.max()),
            uv: [uv_rect.max_x(), uv_rect.min_y()],
        },
        Vertex {
            position: transform(point2(vertex_rect.min_x(), vertex_rect.max_y())),
            uv: [uv_rect.min_x(), uv_rect.min_y()],
        },
    ]);
}

fn render_quad(position: Point2D<f32>, tex_coords: TextureRect, out: &mut Vec<Vertex>) {
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

    out.extend_from_slice(&[
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
