# Sphere contacts with static scenery

`ISolidSweeper::GetWorldStaticCollisionShape()` is the dedicated read-only shape
used by the static-world sweep and its audit. Generic `GetCollisionShape()` and
`MakeSphere()` retain dynamic-pair semantics. Capturers must inspect the dedicated
shape through `const ISolidSweeper&`, including the actual float query radius;
they must not infer the numerical CCD margin from static radius minus physical
radius when a separate skin is present.

A sphere defaults to zero static contact offset and its existing static friction.
Game subclasses may override `GetWorldStaticContactOffsetCm()` and
`GetWorldStaticImpactFriction()` without inflating dynamic shapes or changing
other sphere users. Projection and flat support must use the same support radius
as the static sweep. Physical penetration diagnostics retain physical-radius
meaning, subtracting the separate skin from an inflated projection witness.

For a skinned sphere, response uses a radial physical-radius arm. The authored
surface point and penetration witness remain unchanged for support and auditing.
The numerical query margin and the geometry skin are independent quantities.
