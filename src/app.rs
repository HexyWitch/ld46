use anyhow::Error;
use euclid::{
    default::{Point2D, Rect, Size2D, Transform2D, Vector2D},
    point2, size2, vec2, Angle,
};
use rand::{rngs::SmallRng, Rng, SeedableRng};
use zerocopy::AsBytes;

use crate::{
    gl,
    input::{InputEvent, Key},
    texture_atlas::TextureAtlas,
};

const TEXTURE_ATLAS_SIZE: Size2D<u32> = Size2D {
    width: 1024,
    height: 1024,
    _unit: std::marker::PhantomData::<euclid::UnknownUnit>,
};

pub struct Application {
    assets: Assets,

    program: gl::Program,
    borders_buffer: gl::VertexBuffer,
    entities_buffer: gl::VertexBuffer,

    game_state: GameState,

    last_update: f32,
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
            fly: load_image(
                include_bytes!("../assets/fly.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            fly_shadow: load_image(
                include_bytes!("../assets/fly_shadow.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            tongue_segment: load_image(
                include_bytes!("../assets/tongue_segment.png"),
                &mut texture_atlas,
                &mut texture,
            )?,
            tongue_end: load_image(
                include_bytes!("../assets/tongue_end.png"),
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

        let game_state = GameState::new(&assets);

        Ok(Self {
            assets,

            program,
            borders_buffer,
            entities_buffer: gl_context.create_vertex_buffer()?,

            game_state,

            last_update: 0.0,
        })
    }

    pub fn update(&mut self, input: &[InputEvent], dt: f32) -> Result<(), Error> {
        self.last_update += dt;

        for event in input {
            match event {
                InputEvent::KeyDown(Key::A) => self.game_state.frog.kick(-1),
                InputEvent::KeyDown(Key::D) => self.game_state.frog.kick(1),
                _ => {}
            }
        }

        // the jankiest fixed timestep loop you'll ever see
        if self.last_update > 1. / 60. {
            let dt = 1. / 60.;

            self.game_state.frog.update(dt);

            if let Some(ref mut eaten_fly) = self.game_state.eaten_fly {
                if self.game_state.frog.is_eating() && self.game_state.frog.tongue_withdrawing() {
                    eaten_fly.set_eaten();
                    eaten_fly.position = self.game_state.frog.tongue_position().unwrap();
                }

                if !self.game_state.frog.is_eating() {
                    self.game_state.eaten_fly = None;
                }
            }

            let mut eat_fly = None;
            for (i, fly) in self.game_state.flies.iter_mut().enumerate() {
                fly.update(dt, &mut self.game_state.rng);

                if self.game_state.eaten_fly.is_none() && !self.game_state.frog.is_eating() {
                    if (fly.position - self.game_state.frog.position).length() < 30. {
                        let frog_dir = Transform2D::create_rotation(self.game_state.frog.angle)
                            .transform_vector(vec2(0., 1.))
                            .normalize();
                        let to_fly_dir = (fly.position - self.game_state.frog.position).normalize();
                        if frog_dir.dot(to_fly_dir) > 0.71 {
                            self.game_state.frog.start_eating(fly.position);
                            eat_fly = Some(i);
                        }
                    }
                }
            }
            if let Some(eat_fly) = eat_fly {
                self.game_state.eaten_fly = Some(self.game_state.flies.swap_remove(eat_fly));
            }

            self.last_update -= 1. / 60.;
        }
        Ok(())
    }

    pub fn render(&mut self, gl_context: &mut gl::Context) -> Result<(), Error> {
        gl_context.clear([91. / 255., 164. / 255., 244. / 255., 1.]);

        let mut entities = Vec::new();

        if let Some(ref eaten_fly) = self.game_state.eaten_fly {
            eaten_fly.render_shadow(&mut entities);
        }
        for fly in self.game_state.flies.iter() {
            fly.render_shadow(&mut entities);
        }

        if self.game_state.frog.tongue_withdrawing() {
            self.game_state
                .eaten_fly
                .as_ref()
                .unwrap()
                .render_fly(&mut entities);
        }

        self.game_state.frog.render(&mut entities);

        if !self.game_state.frog.tongue_withdrawing() {
            if let Some(ref eaten_fly) = self.game_state.eaten_fly {
                eaten_fly.render_fly(&mut entities);
            }
        }

        for fly in self.game_state.flies.iter() {
            fly.render_fly(&mut entities);
        }

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
    fly: TextureRect,
    fly_shadow: TextureRect,
    tongue_segment: TextureRect,
    tongue_end: TextureRect,
}

struct TongueState {
    timer: f32,
    target_position: Point2D<f32>,
}

impl TongueState {
    pub fn new(target_position: Point2D<f32>) -> Self {
        TongueState {
            timer: 0.0,
            target_position,
        }
    }
}

struct Frog {
    position: Point2D<f32>,
    velocity: Vector2D<f32>,
    angle: Angle<f32>,
    angular_velocity: f32,

    kick_left_duration: f32,
    kick_right_duration: f32,

    tongue_state: Option<TongueState>,

    lily_pad: Sprite,
    body: (Point2D<f32>, Angle<f32>, Sprite),
    leg_upper_left: (Point2D<f32>, Angle<f32>, Sprite),
    leg_lower_left: (Point2D<f32>, Angle<f32>, Sprite),
    foot_left: (Point2D<f32>, Angle<f32>, Sprite),
    leg_upper_right: (Point2D<f32>, Angle<f32>, Sprite),
    leg_lower_right: (Point2D<f32>, Angle<f32>, Sprite),
    foot_right: (Point2D<f32>, Angle<f32>, Sprite),

    tongue_anchor: Point2D<f32>,
    tongue_segment: Sprite,
    tongue_end: Sprite,
}

impl Frog {
    pub fn new(assets: &Assets, position: Point2D<f32>) -> Self {
        let foot = Sprite::new(assets.frog_foot, 1, point2(3., 2.));

        let body_anchor = point2(14.0, 14.0);
        let body = (
            body_anchor,
            Angle::degrees(0.),
            Sprite::new(assets.frog_body, 2, point2(9., 10.)),
        );

        let left_upper_anchor = point2(5., 2.);
        let leg_upper_left = (
            left_upper_anchor,
            Angle::degrees(-120.),
            Sprite::new(assets.frog_leg_upper_left, 1, point2(4., 2.)),
        );
        let left_lower_anchor = point2(2., 9.);
        let leg_lower_left = (
            left_lower_anchor,
            Angle::degrees(120.),
            Sprite::new(assets.frog_leg_lower_left, 1, point2(3., 10.)),
        );
        let left_foot_anchor = point2(2., 1.);
        let foot_left = (left_foot_anchor, Angle::degrees(180.), foot.clone());

        let right_upper_anchor = point2(13., 2.);
        let leg_upper_right = (
            right_upper_anchor,
            Angle::degrees(0.),
            Sprite::new(assets.frog_leg_upper_right, 1, point2(2., 2.)),
        );
        let right_lower_anchor = point2(4., 9.);
        let leg_lower_right = (
            right_lower_anchor,
            Angle::degrees(0.),
            Sprite::new(assets.frog_leg_lower_right, 1, point2(2., 10.)),
        );
        let right_foot_anchor = point2(4., 1.);
        let foot_right = (right_foot_anchor, Angle::degrees(0.), foot);

        Self {
            position,
            velocity: vec2(0., 0.),
            angle: Angle::degrees(0.),
            angular_velocity: 0.,

            kick_left_duration: 0.0,
            kick_right_duration: 0.0,

            tongue_state: None,

            lily_pad: Sprite::new(assets.lily_pad, 1, point2(14., 14.)),
            body,
            leg_upper_left,
            leg_lower_left,
            foot_left,
            leg_upper_right,
            leg_lower_right,
            foot_right,

            tongue_anchor: point2(9.0, 15.0),
            tongue_segment: Sprite::new(assets.tongue_segment, 1, point2(0.0, 1.5)),
            tongue_end: Sprite::new(assets.tongue_end, 1, point2(2.5, 2.5)),
        }
    }

    pub fn is_eating(&self) -> bool {
        self.tongue_state.is_some()
    }

    pub fn start_eating(&mut self, target_position: Point2D<f32>) {
        self.tongue_state = Some(TongueState::new(target_position));
    }

    pub fn tongue_withdrawing(&self) -> bool {
        if let Some(ref tongue_state) = self.tongue_state {
            tongue_state.timer > 0.1
        } else {
            false
        }
    }

    pub fn tongue_position(&self) -> Option<Point2D<f32>> {
        if let Some(ref tongue_state) = self.tongue_state {
            let r = if tongue_state.timer < 0.1 {
                tongue_state.timer / 0.1
            } else {
                (0.2 - tongue_state.timer) / 0.1
            };
            let anchor_pos = self.position
                + self
                    .body
                    .2
                    .transform()
                    .transform_point(self.tongue_anchor)
                    .to_vector();
            Some(anchor_pos + (tongue_state.target_position - anchor_pos) * r)
        } else {
            None
        }
    }

    pub fn kick(&mut self, direction: i32) {
        let direction = if direction > 0 { 1 } else { -1 };
        let duration = if direction < 0 {
            &mut self.kick_right_duration
        } else {
            &mut self.kick_left_duration
        };

        let pre_ang_vel = self.angular_velocity;
        if *duration <= 0.0 {
            if self.angular_velocity * (direction as f32) < 0.0 {
                self.angular_velocity = 0.0;
            }
            self.angular_velocity +=
                FROG_KICK_ANGULAR_MOMENTUM / FROG_MOMENT_OF_INERTIA * direction as f32;
            self.velocity += Transform2D::create_rotation(self.angle)
                .transform_vector(vec2(0.0, 1.0))
                * (FROG_KICK_LINEAR_MOMENTUM / FROG_MASS);
        }

        // One frog's loss of angular momentum is the same frog's gain in linear momentum
        let ang_vel_loss = (self.angular_velocity - pre_ang_vel) * -pre_ang_vel.signum();
        let angular_momentum_loss = ang_vel_loss.max(0.0) * FROG_MOMENT_OF_INERTIA;
        self.velocity += Transform2D::create_rotation(self.angle).transform_vector(vec2(0.0, 1.0))
            * ((angular_momentum_loss) / FROG_MASS);

        *duration = 0.25
    }

    pub fn update(&mut self, dt: f32) {
        let (upper_a, lower_a, foot_a) = if self.kick_left_duration > 0.0 {
            (
                Angle::degrees(-120.),
                Angle::degrees(120.),
                Angle::degrees(180.),
            )
        } else {
            (Angle::degrees(0.), Angle::degrees(0.), Angle::degrees(0.))
        };
        self.leg_upper_left.1 = upper_a;
        self.leg_lower_left.1 = lower_a;
        self.foot_left.1 = foot_a;
        self.kick_left_duration -= dt;

        let (upper_a, lower_a, foot_a) = if self.kick_right_duration > 0.0 {
            (
                Angle::degrees(120.),
                Angle::degrees(-120.),
                Angle::degrees(180.),
            )
        } else {
            (Angle::degrees(0.), Angle::degrees(0.), Angle::degrees(0.))
        };
        self.leg_upper_right.1 = upper_a;
        self.leg_lower_right.1 = lower_a;
        self.foot_right.1 = foot_a;
        self.kick_right_duration -= dt;

        self.position += self.velocity * dt;
        self.angle.radians += self.angular_velocity * dt;

        self.lily_pad
            .set_transform(Transform2D::create_rotation(self.angle));

        self.velocity *= 0.99;
        self.angular_velocity *= 0.995;

        if let Some(ref mut tongue_state) = self.tongue_state {
            tongue_state.timer += dt;
            if tongue_state.timer > 0.2 {
                self.tongue_state = None;
            }
        }

        if self.position.x < LEVEL_BOUNDS[0] {
            self.position.x = LEVEL_BOUNDS[0];
            self.velocity.x = self.velocity.x.abs() * 0.5;
        }
        if self.position.x > LEVEL_BOUNDS[2] {
            self.position.x = LEVEL_BOUNDS[2];
            self.velocity.x = -self.velocity.x.abs() * 0.5;
        }
        if self.position.y < LEVEL_BOUNDS[1] {
            self.position.y = LEVEL_BOUNDS[1];
            self.velocity.y = self.velocity.y.abs() * 0.5;
        }
        if self.position.y > LEVEL_BOUNDS[3] {
            self.position.y = LEVEL_BOUNDS[3];
            self.velocity.y = -self.velocity.y.abs() * 0.5;
        }
    }

    pub fn render(&mut self, out: &mut Vec<Vertex>) {
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

        render_sprite(&self.lily_pad, 0, self.position, out);

        if self.is_eating() {
            let anchor_pos = self.position
                + self
                    .body
                    .2
                    .transform()
                    .transform_point(self.tongue_anchor)
                    .to_vector();
            let tongue_position = self.tongue_position().unwrap();
            render_line(
                &mut self.tongue_segment,
                &mut self.tongue_end,
                anchor_pos,
                tongue_position,
                out,
            );
        }

        if self.tongue_state.is_some() {
            render_sprite(&self.body.2, 1, self.position, out);
        } else {
            render_sprite(&self.body.2, 0, self.position, out);
        }
        render_sprite(&self.foot_left.2, 0, self.position, out);
        render_sprite(&self.foot_right.2, 0, self.position, out);
        render_sprite(&self.leg_lower_left.2, 0, self.position, out);
        render_sprite(&self.leg_lower_right.2, 0, self.position, out);
        render_sprite(&self.leg_upper_left.2, 0, self.position, out);
        render_sprite(&self.leg_upper_right.2, 0, self.position, out);
    }
}

struct Fly {
    position: Point2D<f32>,
    target_position: Option<Point2D<f32>>,
    idle_timer: f32,
    eaten: bool,

    sprite: Sprite,
    shadow: Sprite,

    animation_timer: f32,
}

impl Fly {
    pub fn new(assets: &Assets, position: Point2D<f32>, rng: &mut SmallRng) -> Self {
        Self {
            position,
            target_position: None,
            idle_timer: if rng.gen_range(0., 1.) < 0.5 {
                rng.gen_range(0.0, 3.0)
            } else {
                0.0
            },
            eaten: false,

            sprite: Sprite::new(assets.fly, 2, point2(3.5, 2.5)),
            shadow: Sprite::new(assets.fly_shadow, 1, point2(2.0, 2.0)),

            animation_timer: rng.gen_range(0.0, 0.2),
        }
    }

    pub fn set_eaten(&mut self) {
        self.eaten = true;
    }

    pub fn update(&mut self, dt: f32, rng: &mut SmallRng) {
        if !self.eaten {
            self.animation_timer = (self.animation_timer + dt) % 0.1;

            if self.target_position.is_none() && self.idle_timer <= 0.0 {
                self.target_position = Some(point2(
                    rng.gen_range(LEVEL_BOUNDS[0], LEVEL_BOUNDS[2]),
                    rng.gen_range(LEVEL_BOUNDS[1], LEVEL_BOUNDS[3]),
                ));
            } else {
                self.idle_timer -= dt;
            }

            if let Some(target_position) = self.target_position {
                let to_target = (target_position - self.position).normalize();
                let new_position = self.position + to_target * 20. * dt;
                if (target_position - new_position).dot(to_target) < 0.0 {
                    self.target_position = None;
                    self.idle_timer = rng.gen_range(0.0, 3.0);
                }
                self.position = new_position;
            }
        }
    }

    pub fn render_shadow(&self, out: &mut Vec<Vertex>) {
        if !self.eaten {
            render_sprite(&self.shadow, 0, self.position + vec2(0.0, -10.0), out);
        }
    }

    pub fn render_fly(&self, out: &mut Vec<Vertex>) {
        render_sprite(
            &self.sprite,
            if self.animation_timer < 0.05 { 0 } else { 1 },
            self.position,
            out,
        );
    }
}

const FROG_KICK_ANGULAR_MOMENTUM: f32 = 12.;
const FROG_KICK_LINEAR_MOMENTUM: f32 = 6.;

const FROG_MOMENT_OF_INERTIA: f32 = 5.0;
const FROG_MASS: f32 = 1.0;

#[derive(Clone)]
struct Sprite {
    frames: Vec<TextureRect>,
    frame_count: u32,
    origin: Point2D<f32>,
    transform: Transform2D<f32>,
}

impl Sprite {
    fn new(image: TextureRect, frame_count: u32, origin: Point2D<f32>) -> Self {
        let width = image[2] - image[0];
        let frame_width = width / frame_count;
        let frames = (0..frame_count)
            .map(|i| {
                [
                    image[0] + i * frame_width,
                    image[1],
                    image[0] + (i + 1) * frame_width,
                    image[3],
                ]
            })
            .collect();
        Self {
            frames,
            frame_count,
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

struct GameState {
    rng: SmallRng,

    frog: Frog,
    flies: Vec<Fly>,
    eaten_fly: Option<Fly>,
}

impl GameState {
    pub fn new(assets: &Assets) -> Self {
        let mut rng = SmallRng::seed_from_u64(0);
        let mut flies = Vec::new();
        for _ in 0..5 {
            let pos = point2(rng.gen_range(20., 244.), rng.gen_range(20., 180.));
            flies.push(Fly::new(assets, pos, &mut rng));
        }
        Self {
            rng,
            frog: Frog::new(assets, point2(133., 50.)),
            flies,
            eaten_fly: None,
        }
    }
}

const LEVEL_BOUNDS: [f32; 4] = [20., 20., 250., 180.];

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

fn render_line(
    segment: &mut Sprite,
    end: &mut Sprite,
    start_point: Point2D<f32>,
    end_point: Point2D<f32>,
    out: &mut Vec<Vertex>,
) {
    let angle = (end_point - start_point).angle_from_x_axis();

    segment.set_transform(
        Transform2D::create_scale(
            (end_point - start_point).length()
                / (segment.frames[0][2] as f32 - segment.frames[0][0] as f32),
            1.0,
        )
        .post_rotate(-angle),
    );
    end.set_transform(Transform2D::create_rotation(-angle));
    render_sprite(segment, 0, start_point, out);
    render_sprite(end, 0, end_point, out);
}

fn render_sprite(sprite: &Sprite, frame: usize, position: Point2D<f32>, out: &mut Vec<Vertex>) {
    let size = size2(
        (sprite.frames[frame][2] - sprite.frames[frame][0]) as f32,
        (sprite.frames[frame][3] - sprite.frames[frame][1]) as f32,
    );
    let vertex_rect = Rect::new(point2(0., 0.), size);

    let uv_pos = point2(
        sprite.frames[frame][0] as f32 / TEXTURE_ATLAS_SIZE.width as f32,
        sprite.frames[frame][1] as f32 / TEXTURE_ATLAS_SIZE.height as f32,
    );
    let uv_size = size2(
        (sprite.frames[frame][2] - sprite.frames[frame][0]) as f32
            / TEXTURE_ATLAS_SIZE.width as f32,
        (sprite.frames[frame][3] - sprite.frames[frame][1]) as f32
            / TEXTURE_ATLAS_SIZE.height as f32,
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
