use dimple::aero::BallModel;
use dimple::environment::{Atmosphere, Environment, WindProfile};
use dimple::ground::{BallState, GroundModel, Surface};
use dimple::trajectory::{ShotInput, simulate_shot_with_atmosphere, simulate_shot_with_wind};
use glam::DVec3;

#[test]
fn legacy_entry_point_keeps_identical_results() {
    let env = Environment::SEA_LEVEL;
    let atmosphere = Atmosphere::legacy(env.air_density());
    for wind in [DVec3::ZERO, DVec3::new(-5.0, 0.0, 3.0)] {
        let input = ShotInput::driver();
        let old = simulate_shot_with_wind(&input, &BallModel::TOUR, &env, &Surface::FAIRWAY, wind);
        let new = simulate_shot_with_atmosphere(
            &input,
            &BallModel::TOUR,
            &atmosphere,
            &Surface::FAIRWAY,
            WindProfile::Uniform(wind),
        );
        assert_eq!(old.flight.carry_m, new.flight.carry_m);
        assert_eq!(old.final_pos, new.final_pos);
    }
}

#[test]
fn viscosity_changes_flight_at_constant_density() {
    let shot = ShotInput::seven_iron();
    let simulate = |t| {
        simulate_shot_with_atmosphere(
            &shot,
            &BallModel::TOUR,
            &Atmosphere::from_density_and_temperature(1.2, t).unwrap(),
            &Surface::FAIRWAY,
            WindProfile::Uniform(DVec3::ZERO),
        )
    };
    assert!((simulate(0.0).flight.carry_m - simulate(35.0).flight.carry_m).abs() > 0.001);
}

#[test]
fn bounce_hops_sample_the_same_profile() {
    let atmosphere = Atmosphere::from_density_and_temperature(1.2, 20.0).unwrap();
    let shot = simulate_shot_with_atmosphere(
        &ShotInput::driver(),
        &BallModel::TOUR,
        &atmosphere,
        &Surface::FAIRWAY,
        WindProfile::Uniform(DVec3::ZERO),
    );
    let p = shot.flight.points.last().unwrap();
    let landing = BallState {
        time: p.time,
        pos: p.pos,
        vel: p.vel,
        spin: shot.flight.landing_spin,
    };
    let wind10 = DVec3::new(0.0, 0.0, 5.0);
    let uniform = Surface::FAIRWAY.settle_with_atmosphere(
        &landing,
        &BallModel::TOUR,
        &atmosphere,
        WindProfile::Uniform(wind10),
    );
    let profile = Surface::FAIRWAY.settle_with_atmosphere(
        &landing,
        &BallModel::TOUR,
        &atmosphere,
        WindProfile::GarminPowerLaw {
            wind10,
            ground_height_m: 0.0,
        },
    );
    assert!(profile.contacts.len() > 1);
    assert!((uniform.rest.pos.z - profile.rest.pos.z).abs() > 0.001);
}
