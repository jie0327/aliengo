# MID-360 visual model

`mid360.dae` is the third-party low-poly MID-360 model from
[mathaimp/livox_gz_sim](https://github.com/mathaimp/livox_gz_sim/tree/3f06f1ac4d3f6854f7275ca9aa097484791a1abd),
commit `3f06f1ac4d3f6854f7275ca9aa097484791a1abd`, under the accompanying
[MIT license](LICENSE.livox_gz_sim). SHA-256:
`7323b886c2be32ae9b9cbc1f8dd44664218415f2407f8fc253c88bce319ccfc8`.
The upstream author describes the model as made from dimensions in the Livox
user manual. It is not Livox's official precise CAD model.

The mesh uses meters, +Z up, and its origin is the center of the bottom face;
the connector is on the -X side. The
point-cloud origin is 0.047 m above that face, so `robot.xacro` offsets the
visual by -0.047 m relative to `mid360_link`. The collision box and inertia
are approximations; the mount pose must still be measured on the robot.
