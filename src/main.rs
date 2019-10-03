use ggez::event::{self, EventHandler, MouseButton};
use ggez::graphics;
use ggez::mint;
use ggez::nalgebra as na;
use ggez::{Context, ContextBuilder, GameResult};

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
    fn add_ball(&mut self, x: f32, y: f32, velocity: Vector2<f32>) {
        let rigid_body = RigidBodyDesc::new()
            .translation(Vector2::new(x, y))
            .mass(0.1)
            .velocity(Velocity::linear(velocity.x, velocity.y))
            .build();

        let rb_handle = self.bodies.insert(rigid_body);

        let shape = ShapeHandle::new(Ball::new(10.0));

        let collider = ColliderDesc::new(shape)
            .ccd_enabled(false)
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
            .build(BodyPartHandle(rb_handle, 0));

        let _collider_handle = self.colliders.insert(collider);
    }

    fn step(&mut self) {
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

            let b = graphics::Mesh::new_circle(
                ctx,
                DrawMode::fill(),
                mint::Point2 { x: 0.0, y: 0.0 },
                10.0,
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
                let b = graphics::Mesh::new_circle(
                    ctx,
                    DrawMode::fill(),
                    mint::Point2 { x: 0.0, y: 0.0 },
                    10.0,
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
        println!("Mouse button pressed: {:?}, x: {}, y: {}", button, x, y);
        self.prev_pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
    }

    fn mouse_button_up_event(&mut self, _ctx: &mut Context, button: MouseButton, x: f32, y: f32) {
        self.mouse_down = false;
        println!("Mouse button released: {:?}, x: {}, y: {}", button, x, y);
        let pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
        let mut vel = pt - self.prev_pt;
        vel *= 2.0;
        self.physics.add_ball(self.prev_pt.x, self.prev_pt.y, vel);
    }
}
