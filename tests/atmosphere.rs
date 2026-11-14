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

/// Wind on the bounce hops must not keep the ball bouncing. A fast landing
/// in a strong wind used to settle into a train of hops that the wind fed
/// back as fast as the impacts drained it, so the ball hopped on to the
/// impact limit, hundreds of metres past the landing. (A wind this strong at
/// the turf may still blow the ball a long way along the ground.)
#[test]
fn strong_wind_does_not_keep_the_ball_bouncing() {
    use dimple::ground::Termination;
    let atmosphere = Atmosphere::from_density_and_temperature(1.2, 20.0).unwrap();
    for (speed_mph, launch_deg, spin, wind) in [
        (67.1, 8.0, 1500.0, DVec3::new(20.0, 0.0, 0.0)),
        (67.1, 8.0, 1500.0, DVec3::new(14.0, 0.0, 14.0)),
        (100.7, 18.0, 10000.0, DVec3::new(-20.0, 0.0, 0.0)),
        (157.0, 12.0, 2500.0, DVec3::new(20.0, 0.0, 0.0)),
    ] {
        let shot = ShotInput {
            ball_speed_mph: speed_mph,
            launch_angle_deg: launch_deg,
            launch_azimuth_deg: 0.0,
            backspin_rpm: spin,
            sidespin_rpm: 0.0,
        };
        for profile in [
            WindProfile::Uniform(wind),
            WindProfile::GarminPowerLaw {
                wind10: wind,
                ground_height_m: 0.0,
            },
        ] {
            let r = simulate_shot_with_atmosphere(
                &shot,
                &BallModel::TOUR,
                &atmosphere,
                &Surface::FAIRWAY,
                profile,
            );
            let hops = r.ground.contacts.len();
            assert_eq!(
                r.ground.termination,
                Termination::Rest,
                "{profile:?} {shot:?}"
            );
            assert!(hops < 60, "{profile:?} {shot:?}: {hops} impacts");
        }
    }
}
