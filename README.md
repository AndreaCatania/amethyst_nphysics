# NPhysics Amethyst backend
![Build Status] ![License] ![Line of code][loc]

[Build Status]: https://travis-ci.com/AndreaCatania/amethyst_nphysics.svg?branch=master
[License]: https://img.shields.io/badge/License-MIT-green.svg
[loc]: https://tokei.rs/b1/github/andreacatania/amethyst_nphysics?category=code

This crate is the [NPhysics] integration of the [Phythyst] interface.

To use this backend you have to specify the class `NPhysicsBackend` in the `PhysicsBundle` as shown below.

```rust
use amethyst::phythyst::PhysicsBundle;
use amethyst::amethyst_nphysics::NPhysicsBackend;

let game_data = GameDataBuilder::default()
    .with_bundle(PhysicsBundle::<f32, NPhysicsBackend>::new()).unwrap()
```

You can use this trought [Phythyst].

[NPhysics]: https://nphysics.org/
[Phythyst]: https://github.com/AndreaCatania/amethyst_phythyst