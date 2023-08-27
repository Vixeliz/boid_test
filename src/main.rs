use ggegui::{egui, Gui};
use ggez::glam;
use ggez::graphics::{
    Camera3d, Canvas3d, DrawParam3d, InstanceArray3d, Mesh3d, Mesh3dBuilder, Shader, ShaderBuilder,
};
use rand::Rng;
use std::{env, path};

use ggez::input::keyboard::KeyCode;
use ggez::{
    event,
    glam::*,
    graphics::{self, Color},
    Context, GameResult,
};

struct BoidState {
    max_speed: f32,
    view_distance: f32,
    min_distance: f32,
    box_size: f32,
    avoidance: f32,
    centering: f32,
    matching: f32,
}

impl Default for BoidState {
    fn default() -> Self {
        Self {
            max_speed: 500.0,
            view_distance: 10.0,
            min_distance: 5.0,
            box_size: 100.0,
            avoidance: 0.5,
            centering: 0.075,
            matching: 0.2,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Boid {
    pos: Vec3,
    vel: Vec3,
    col: Color,
}

impl Boid {
    fn update(&mut self, dt: f32, boids: &[Boid], boid_state: &BoidState) {
        self.center(boids, boid_state);
        self.avoid(boids, boid_state);
        self.matching(boids, boid_state);

        // Update position based on velocity
        self.pos += self.vel * dt;

        // Clamp position and revert velocity

        let result = self.pos.cmpgt(Vec3::splat(boid_state.box_size));
        if result.x {
            self.vel.x *= -1.0;
        }
        if result.y {
            self.vel.y *= -1.0;
        }
        if result.z {
            self.vel.z *= -1.0;
        }

        let result = self.pos.cmplt(Vec3::ZERO);
        if result.x {
            self.vel.x *= -1.0;
        }
        if result.y {
            self.vel.y *= -1.0;
        }
        if result.z {
            self.vel.z *= -1.0;
        }

        self.pos.clamp(Vec3::ZERO, Vec3::splat(boid_state.box_size));

        // Clamp velocity
        self.vel.clamp(
            Vec3::splat(-boid_state.max_speed),
            Vec3::splat(boid_state.max_speed),
        );
    }

    fn center(&mut self, boids: &[Boid], boid_state: &BoidState) {
        // We get the average center of nearby boids we can see
        let mut center = Vec3::ZERO;
        let mut num_neighbors = 0;
        for other in boids {
            if self.pos.distance(other.pos) < boid_state.view_distance {
                center += other.pos;
                num_neighbors += 1;
            }
        }
        if num_neighbors > 0 {
            center /= Vec3::splat(num_neighbors as f32);

            self.vel += (center - self.pos) * boid_state.centering;
        }
    }

    fn avoid(&mut self, boids: &[Boid], boid_state: &BoidState) {
        // We add some velocity based off of the boids close to us to avoid said boids
        let mut move_vec = Vec3::default();
        for other in boids {
            let dist = self.pos.distance(other.pos);
            if dist < boid_state.min_distance && dist > 0.0 {
                move_vec += self.pos - other.pos;
            }
        }
        self.vel += move_vec * boid_state.avoidance;

        // Avoid walls
        let mut move_vec = Vec3::default();
        // X
        let dist = (boid_state.box_size - self.pos.x).abs();
        if dist < boid_state.min_distance && dist > 0.0 {
            move_vec.x += self.pos.x - boid_state.box_size;
        }
        // Y
        let dist = (boid_state.box_size - self.pos.y).abs();
        if dist < boid_state.min_distance && dist > 0.0 {
            move_vec.y += self.pos.y - boid_state.box_size;
        }
        // Z
        let dist = (boid_state.box_size - self.pos.z).abs();
        if dist < boid_state.min_distance && dist > 0.0 {
            move_vec.z += self.pos.z - boid_state.box_size;
        }

        // X
        let dist = (0.0 - self.pos.x).abs();
        if dist < boid_state.min_distance && dist > 0.0 {
            move_vec.x += self.pos.x;
        }
        // Y
        let dist = (0.0 - self.pos.y).abs();
        if dist < boid_state.min_distance && dist > 0.0 {
            move_vec.y += self.pos.y;
        }
        // Z
        let dist = (0.0 - self.pos.z).abs();
        if dist < boid_state.min_distance && dist > 0.0 {
            move_vec.z += self.pos.z;
        }
        self.vel += move_vec * 4.0;
    }

    fn matching(&mut self, boids: &[Boid], boid_state: &BoidState) {
        let mut avg_vel = Vec3::default();
        let mut num_neighbors = 0;
        for other in boids {
            if self.pos.distance(other.pos) < boid_state.view_distance {
                avg_vel += other.vel;
                num_neighbors += 1;
            }
        }
        if num_neighbors > 0 {
            avg_vel /= Vec3::splat(num_neighbors as f32);

            self.vel += (avg_vel - self.vel) * boid_state.matching;
        }
    }
}

impl Default for Boid {
    fn default() -> Self {
        let mut rng = rand::thread_rng();
        let boid_state = BoidState::default();
        Boid {
            pos: Vec3::new(
                rng.gen_range(0.0..boid_state.box_size),
                rng.gen_range(0.0..boid_state.box_size),
                rng.gen_range(0.0..boid_state.box_size),
            ),
            vel: Vec3::new(
                rng.gen_range(0.0..10.0),
                rng.gen_range(0.0..10.0),
                rng.gen_range(0.0..10.0),
            ),
            col: Color::new(
                rng.gen_range(0.0..1.0),
                rng.gen_range(0.0..1.0),
                rng.gen_range(0.0..1.0),
                1.0,
            ),
        }
    }
}

struct MainState {
    camera: Camera3d,
    instances: InstanceArray3d,
    boids: Vec<Boid>,
    shader: Shader,
    fancy_shader: Shader,
    boid_state: BoidState,
    gui: Gui,
    cube: Mesh3d,
}

impl MainState {
    fn new(ctx: &mut Context) -> GameResult<Self> {
        let mut camera = Camera3d::default();
        camera.transform.yaw = 90.0;
        let cube = Mesh3dBuilder::new().cube(Vec3::splat(1.0)).build(ctx);
        let pyramid = Mesh3dBuilder::new()
            .pyramid(Vec2::splat(1.0), 2.0, false)
            .build(ctx);

        let mut instances = graphics::InstanceArray3d::new(ctx, None, pyramid);
        instances.resize(ctx, 100);

        let boid_state = BoidState::default();
        let mut boids = Vec::new();

        for _ in 0..100 {
            boids.push(Boid::default());
        }

        Ok(MainState {
            camera,
            instances,
            boids,
            shader: ShaderBuilder::from_path("/instance_unordered3d.wgsl").build(ctx)?,
            fancy_shader: ShaderBuilder::from_path("/fancy.wgsl").build(ctx)?,
            boid_state,
            gui: Gui::new(ctx),
            cube,
        })
    }
}

impl event::EventHandler for MainState {
    fn resize_event(&mut self, _: &mut Context, width: f32, height: f32) -> GameResult {
        self.camera.projection.resize(width as u32, height as u32);
        self.camera.projection.zfar = 10000.0;
        self.camera.projection.znear = 0.1;
        Ok(())
    }
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        // Update boids:
        for i in 0..self.boids.len() {
            let mut boid = self.boids[i];
            boid.update(
                ctx.time.delta().as_secs_f32(),
                self.boids.as_slice(),
                &self.boid_state,
            );
            self.boids[i] = boid;
        }

        // GUI
        let gui_ctx = self.gui.ctx();

        egui::Window::new("UI").show(&gui_ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Box Size: ");
                ui.add(egui::Slider::new(
                    &mut self.boid_state.box_size,
                    1.0..=1000.0,
                ));
            });
            ui.horizontal(|ui| {
                ui.label("Max Speed: ");
                ui.add(egui::Slider::new(
                    &mut self.boid_state.max_speed,
                    1.0..=100.0,
                ));
            });
            ui.horizontal(|ui| {
                ui.label("Avoidance: ");
                ui.add(egui::Slider::new(&mut self.boid_state.avoidance, 0.0..=2.0));
            });
            ui.horizontal(|ui| {
                ui.label("Centering: ");
                ui.add(egui::Slider::new(&mut self.boid_state.centering, 0.0..=2.0));
            });
            ui.horizontal(|ui| {
                ui.label("Matching Velocity: ");
                ui.add(egui::Slider::new(&mut self.boid_state.matching, 0.0..=2.0));
            });
            ui.horizontal(|ui| {
                ui.label("Min Distance: ");
                ui.add(egui::Slider::new(
                    &mut self.boid_state.min_distance,
                    0.0..=2.0,
                ));
            });
            ui.horizontal(|ui| {
                ui.label("View Distance: ");
                ui.add(egui::Slider::new(
                    &mut self.boid_state.view_distance,
                    0.0..=100.0,
                ));
            });

            if ui.button("reset").clicked() {
                let mut boids = Vec::new();

                for _ in 0..100 {
                    boids.push(Boid::default());
                }
                self.boids = boids;
            }
            if ui.button("quit").clicked() {
                ctx.request_quit();
            }
        });
        self.gui.update(ctx);

        // Input
        let k_ctx = &ctx.keyboard.clone();
        let (yaw_sin, yaw_cos) = self.camera.transform.yaw.sin_cos();
        let forward = Vec3::new(yaw_cos, 0.0, yaw_sin).normalize();
        let right = Vec3::new(-yaw_sin, 0.0, yaw_cos).normalize();

        if k_ctx.is_key_pressed(KeyCode::Space) {
            self.camera.transform.position.y += 1.0;
        }
        if k_ctx.is_key_pressed(KeyCode::C) {
            self.camera.transform.position.y -= 1.0;
        }
        if k_ctx.is_key_pressed(KeyCode::W) {
            self.camera.transform = self.camera.transform.translate(forward);
        }
        if k_ctx.is_key_pressed(KeyCode::S) {
            self.camera.transform = self.camera.transform.translate(-forward);
        }
        if k_ctx.is_key_pressed(KeyCode::D) {
            self.camera.transform = self.camera.transform.translate(right);
        }
        if k_ctx.is_key_pressed(KeyCode::A) {
            self.camera.transform = self.camera.transform.translate(-right);
        }
        if k_ctx.is_key_pressed(KeyCode::Right) {
            self.camera.transform.yaw += 1.0_f32.to_radians();
        }
        if k_ctx.is_key_pressed(KeyCode::Left) {
            self.camera.transform.yaw -= 1.0_f32.to_radians();
        }
        if k_ctx.is_key_pressed(KeyCode::Up) {
            self.camera.transform.pitch += 1.0_f32.to_radians();
        }
        if k_ctx.is_key_pressed(KeyCode::Down) {
            self.camera.transform.pitch -= 1.0_f32.to_radians();
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas3d = Canvas3d::from_frame(ctx, Color::new(0.25, 0.25, 0.25, 1.0));
        canvas3d.set_projection(self.camera.to_matrix());
        canvas3d.set_shader(&self.fancy_shader);
        // Inverted box
        canvas3d.draw(
            &self.cube,
            DrawParam3d::default()
                .scale(Vec3::splat(-self.boid_state.box_size))
                .position(Vec3::splat(self.boid_state.box_size / 2.0)),
        );
        canvas3d.set_shader(&self.shader);

        // Set rotation, position, and color for boids
        self.instances.set((0..100).map(|i| {
            let direction = self.boids[i].pos + (self.boids[i].vel * 10.0);
            let up = Vec3::Y;
            let back = -direction.try_normalize().unwrap_or(Vec3::NEG_Z);
            let right = up
                .cross(back)
                .try_normalize()
                .unwrap_or_else(|| up.any_orthonormal_vector());
            let up = back.cross(right);
            graphics::DrawParam3d::default()
                .position(self.boids[i].pos / Vec3::splat(2.0))
                .color(self.boids[i].col)
                .rotation(
                    Quat::from_mat3(&Mat3::from_cols(right, up, back))
                        * Quat::from_euler(EulerRot::XYZ, -90.0_f32.to_radians(), 0.0, 0.0),
                )
        }));

        // Params that affect all boids
        let param = graphics::DrawParam3d::default()
            .color(Color::new(1.0, 1.0, 1.0, 1.0))
            .scale(Vec3::splat(2.0));

        canvas3d.draw(&self.instances, param);

        canvas3d.finish(ctx)?;
        let mut canvas = graphics::Canvas::from_frame(ctx, None);

        let dest_point1 = Vec2::new(10.0, 210.0);
        canvas.draw(
            &graphics::Text::new("You can mix 3d and 2d drawing;"),
            dest_point1,
        );
        canvas.draw(
            &self.gui,
            graphics::DrawParam::default().dest(glam::Vec2::ZERO),
        );
        canvas.finish(ctx)?;

        Ok(())
    }
}

pub fn main() -> GameResult {
    let resource_dir = if let Ok(manifest_dir) = env::var("CARGO_MANIFEST_DIR") {
        let mut path = path::PathBuf::from(manifest_dir);
        path.push("resources");
        path
    } else {
        path::PathBuf::from("./resources")
    };

    let cb = ggez::ContextBuilder::new("3dshapes", "ggez")
        .window_mode(ggez::conf::WindowMode::default().resizable(true))
        .add_resource_path(resource_dir);

    let (mut ctx, events_loop) = cb.build()?;
    let state = MainState::new(&mut ctx)?;
    event::run(ctx, events_loop, state)
}
