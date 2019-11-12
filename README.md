# NPhysics Amethyst backend
[![Build Status]](https://travis-ci.org/AndreaCatania/amethyst_nphysics) [![License]](https://github.com/AndreaCatania/amethyst_nphysics/blob/master/LICENSE) [![Line of code][loc]](https://github.com/AndreaCatania/amethyst_nphysics/pulse)

[Build Status]: https://travis-ci.org/AndreaCatania/amethyst_nphysics.svg?branch=master
[License]: https://img.shields.io/badge/License-MIT-green.svg
[loc]: https://tokei.rs/b1/github/andreacatania/amethyst_nphysics?category=code

This crate is the [NPhysics] integration of the [amethyst_physics] interface.

To use this backend you have to specify the class `NPhysicsBackend` in the `PhysicsBundle` as shown below.

```rust
use amethyst_physics::PhysicsBundle;
use amethyst::amethyst_nphysics::NPhysicsBackend;

let game_data = GameDataBuilder::default()
    .with_bundle(PhysicsBundle::<f32, NPhysicsBackend>::new()).unwrap()
```

You can use this through [amethyst_physics].

[NPhysics]: https://nphysics.org/
[amethyst_physics]: https://github.com/AndreaCatania/amethyst_physics
