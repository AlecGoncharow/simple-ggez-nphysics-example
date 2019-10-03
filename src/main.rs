use ggez::event::{self, EventHandler, MouseButton};
use ggez::graphics;
use ggez::mint;
use ggez::nalgebra as na;
use ggez::{Context, ContextBuilder, GameResult};

use na::base::Unit;
use na::Vector2;
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::math::Velocity;
use nphysics2d::object::{
    BodyPartHandle, BodyStatus, ColliderDesc, DefaultBodySet, DefaultColliderSet, RigidBody,
    RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};

use ggez::graphics::DrawMode;
use rand::Rng;

const WIDTH_LOCAL: f32 = 800.0;
const HEIGHT_LOCAL: f32 = 600.0;

fn main() {
    // Make a Context and an EventLoop.
    let (mut ctx, mut event_loop) = ContextBuilder::new("game_name", "author_name")
        .build()
        .unwrap();

    // Create an instance of your event handler.
    // Usually, you should provide it with the Context object
    // so it can load resources like images during setup.
    let mut my_game = MyGame::new(&mut ctx);

    // Run!
    match event::run(&mut ctx, &mut event_loop, &mut my_game) {
        Ok(_) => println!("Exited cleanly."),
        Err(e) => println!("Error occured: {}", e),
    }
}

struct MyGame {
    // hmm
    physics: Physics,
    prev_pt: Vector2<f32>,
    rad: f32,
    mouse_down: bool,
}

// trusting this: https://github.com/rustsim/nphysics/blob/master/src_testbed/testbed.rs#L114
struct Physics {
    mechanical_world: DefaultMechanicalWorld<f32>,
    geometrical_world: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    forces: DefaultForceGeneratorSet<f32>,
    constraints: DefaultJointConstraintSet<f32>,
}

impl Physics {
    fn add_ball(&mut self, x: f32, y: f32, velocity: Vector2<f32>, radius: f32) {

        let rigid_body = RigidBodyDesc::new()
            .translation(Vector2::new(x, y))
            .mass(std::f32::consts::PI * radius.powf(2.0))
            //.angular_inertia(0.1)
            //.angular_damping(5.0)
            .velocity(Velocity::linear(velocity.x, velocity.y))
            .build();

        let rb_handle = self.bodies.insert(rigid_body);

        let shape = ShapeHandle::new(Ball::new(radius));

        let collider = ColliderDesc::new(shape)
            .ccd_enabled(false)
            //.density(5.0)
            .build(BodyPartHandle(rb_handle, 0));

        let _collider_handle = self.colliders.insert(collider);
    }

    fn add_wall(&mut self, x: f32, y: f32, width: f32, height: f32) {
        let rigid_body = RigidBodyDesc::new()
            .translation(Vector2::new(x, y))
            .status(BodyStatus::Static)
            .build();

        let rb_handle = self.bodies.insert(rigid_body);

        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(width / 2.0, height / 2.0)));

        let collider = ColliderDesc::new(shape)
            .ccd_enabled(false)
            //.density(1.0)
            .build(BodyPartHandle(rb_handle, 0));

        let _collider_handle = self.colliders.insert(collider);
    }

    fn step(&mut self) {
        for contact in self.geometrical_world.contact_events() {
            // this process adapted from https://ncollide.org/collision_detection_pipeline/
            // with a few changes in the API since then, logic for new velocities is the same
            use ncollide2d::pipeline::narrow_phase::ContactEvent;
            match contact {
                ContactEvent::Started(h1, h2) => {
                    if let Some((
                        _col_handle1,
                        collider1,
                        _col_handle2,
                        collider2,
                        _algorithm,
                        manifold,
                    )) = self
                        .geometrical_world
                        .contact_pair(&self.colliders, *h1, *h2, false)
                    {
                        let deep = manifold.deepest_contact().unwrap();
                        let normal: Unit<Vector2<f32>> = deep.contact.normal;
                        // if either is a box, use normal, else requires some extra maths
                        if collider1.shape_handle().is::<Cuboid<f32>>()
                            || collider2.shape_handle().is::<Cuboid<f32>>()
                        {
                            // totally sane, totally normal
                            {
                                let body1 = self
                                    .bodies
                                    .get_mut(collider1.body())
                                    .unwrap()
                                    .downcast_mut::<RigidBody<f32>>()
                                    .unwrap();

                                let vel1 = body1.velocity().linear.clone();
                                let new_v1 = vel1 - (2.0 * vel1.dot(&normal) * normal.into_inner());
                                body1.set_velocity(Velocity::linear(new_v1.x, new_v1.y));
                            }

                            // round 2
                            {
                                let body2 = self
                                    .bodies
                                    .get_mut(collider2.body())
                                    .unwrap()
                                    .downcast_mut::<RigidBody<f32>>()
                                    .unwrap();

                                let vel2 = body2.velocity().linear.clone();
                                let new_v2 = vel2 - (2.0 * vel2.dot(&normal) * normal.into_inner());
                                body2.set_velocity(Velocity::linear(new_v2.x, new_v2.y))
                            }
                        } else {
                            // adapted from https://stackoverflow.com/a/345863/11015039
                            let body1 = self
                                .bodies
                                .get(collider1.body())
                                .unwrap()
                                .downcast_ref::<RigidBody<f32>>()
                                .unwrap();

                            let body2 = self
                                .bodies
                                .get(collider2.body())
                                .unwrap()
                                .downcast_ref::<RigidBody<f32>>()
                                .unwrap();

                            let mut v1 = body1.velocity().linear.clone();
                            let mut v2 = body2.velocity().linear.clone();
                            let pos1 = body1.position().translation.vector;
                            let pos2 = body2.position().translation.vector;
                            let mass1: f32 = body1.augmented_mass().linear;
                            let mass2: f32 = body2.augmented_mass().linear;

                            let mut collision = pos1 - pos2;
                            let distance = collision.magnitude();

                            collision = collision / distance;

                            let v1ci = v1.dot(&collision);
                            let v2ci = v2.dot(&collision);

                            let v1cf = (v1ci * (mass1 - mass2) + (2.0 * mass2 * v2ci))/(mass1 + mass2);
                            let v2cf = (v2ci * (mass2 - mass1) + (2.0 * mass1 * v1ci))/(mass1 + mass2);

                            v1 += (v1cf - v1ci) * collision;
                            v2 += (v2cf - v2ci) * collision;
                            println!("{:?}, {:?}", v1, v2);

                            {
                                let body1 = self
                                    .bodies
                                    .get_mut(collider1.body())
                                    .unwrap()
                                    .downcast_mut::<RigidBody<f32>>()
                                    .unwrap();
                                body1.set_velocity(Velocity::linear(v1.x, v1.y));
                            }

                            {
                                let body2 = self
                                    .bodies
                                    .get_mut(collider2.body())
                                    .unwrap()
                                    .downcast_mut::<RigidBody<f32>>()
                                    .unwrap();
                                body2.set_velocity(Velocity::linear(v2.x, v2.y))
                            }
                        }
                    }
                }
                ContactEvent::Stopped(_, _) => {}
            }
        }

        self.mechanical_world.step(
            &mut self.geometrical_world,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.constraints,
            &mut self.forces,
        );
    }
}

impl MyGame {
    pub fn new(_ctx: &mut Context) -> MyGame {
        let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, 0.0));
        let geometrical_world = DefaultGeometricalWorld::new();

        let bodies = DefaultBodySet::new();
        let colliders = DefaultColliderSet::new();
        let constraints = DefaultJointConstraintSet::new();
        let forces = DefaultForceGeneratorSet::new();

        let mut physics = Physics {
            mechanical_world,
            geometrical_world,
            bodies,
            colliders,
            forces,
            constraints,
        };

        physics.add_wall(200.0, 200.0, 50.0, 50.0);
        physics.add_wall(500.0, 300.0, 50.0, 200.0);
        physics.add_wall(300.0, 400.0, 50.0, 100.0);
        physics.add_wall(600.0, 100.0, 100.0, 50.0);
        physics.add_wall(100.0, 500.0, 100.0, 50.0);

        // bottom
        physics.add_wall(WIDTH_LOCAL / 2.0, 10.0, WIDTH_LOCAL, 20.0);
        // top
        physics.add_wall(WIDTH_LOCAL / 2.0, HEIGHT_LOCAL - 10.0, WIDTH_LOCAL, 20.0);
        // left
        physics.add_wall(10.0, HEIGHT_LOCAL / 2.0, 20.0, HEIGHT_LOCAL);
        // right
        physics.add_wall(WIDTH_LOCAL - 10.0, HEIGHT_LOCAL / 2.0, 20.0, HEIGHT_LOCAL);

        // Load/create resources here: images, fonts, sounds, etc.
        MyGame {
            physics,
            prev_pt: Vector2::new(0.0, 0.0),
            mouse_down: false,
            rad: 0.0
        }
    }
}

impl EventHandler for MyGame {
    fn update(&mut self, _ctx: &mut Context) -> GameResult<()> {
        // Update code here...
        self.physics.step();
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx, graphics::BLACK);

        // draw starting pos of ball if still dragging velocity
        if self.mouse_down {
            let vec = mint::Vector2 {
                x: self.prev_pt.x,
                y: HEIGHT_LOCAL - self.prev_pt.y,
            };

            println!("{:?}", self.rad);
            let b = graphics::Mesh::new_circle(
                ctx,
                DrawMode::fill(),
                mint::Point2 { x: 0.0, y: 0.0 },
                self.rad,
                1.0,
                graphics::Color::new(0.0, 0.0, 1.0, 1.0),
            )?;
            graphics::draw(ctx, &b, (vec,))?;
        }

        // draw things from nphysics sim
        for (handle, body) in self.physics.bodies.iter() {
            let down = body.downcast_ref::<RigidBody<f32>>().unwrap();
            let pos = down.position().translation.vector;
            //println!("{:?}", pos);
            let coll = self.physics.colliders.get(handle).unwrap();
            let is_ball = match coll.shape().downcast_ref::<Ball<f32>>() {
                None => false,
                _ => true,
            };

            let mut vec = mint::Vector2 {
                x: pos.x,
                y: HEIGHT_LOCAL - pos.y,
            };
            if is_ball {
                let shp = coll.shape_handle().as_shape::<Ball<f32>>().unwrap();

                let b = graphics::Mesh::new_circle(
                    ctx,
                    DrawMode::fill(),
                    mint::Point2 { x: 0.0, y: 0.0 },
                    shp.radius(),
                    1.0,
                    graphics::WHITE,
                )?;
                graphics::draw(ctx, &b, (vec,))?;
            } else {
                let box_coll = coll.shape().downcast_ref::<Cuboid<f32>>().unwrap();
                let half_ext = box_coll.half_extents();

                let x = vec.x - half_ext.x;
                let y = vec.y - half_ext.y;
                vec.x = x;
                vec.y = y;
                use ggez::graphics::Rect;
                let b = graphics::Mesh::new_rectangle(
                    ctx,
                    DrawMode::fill(),
                    Rect {
                        x: 0.0,
                        y: 0.0,
                        w: half_ext.x * 2.0,
                        h: half_ext.y * 2.0,
                    },
                    graphics::WHITE,
                )?;
                graphics::draw(ctx, &b, (vec,))?;
            }
            //println!("{:?}", down);
        }

        graphics::present(ctx)
    }

    fn mouse_button_down_event(&mut self, _ctx: &mut Context, button: MouseButton, x: f32, y: f32) {
        self.mouse_down = true;
        let mut rng = rand::thread_rng();
        let scale: f32 = rng.gen();

        self.rad = 5.0 + (15.0 * scale);
        println!("Mouse button pressed: {:?}, x: {}, y: {}", button, x, y);
        self.prev_pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
    }

    fn mouse_button_up_event(&mut self, _ctx: &mut Context, button: MouseButton, x: f32, y: f32) {
        self.mouse_down = false;
        println!("Mouse button released: {:?}, x: {}, y: {}", button, x, y);
        let pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
        let mut vel = pt - self.prev_pt;
        vel *= 2.0;
        self.physics.add_ball(self.prev_pt.x, self.prev_pt.y, vel, self.rad);
    }
}
