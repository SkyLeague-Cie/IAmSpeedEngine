# BoxStandingStill

Identifier: `IAMSPEED.PHYS.BOX_STANDING_STILL.V1`.
CompositionLevel: Molecular. Foundation: yes.
Dependencies: ContactDetection, NormalResponse, Support; Friction where the
declared external load requires a tangential reaction.

This is **maintenance of an already valid static equilibrium**, not acquisition
after falling. Zero velocity at an arbitrary position is not sufficient: an
airborne box must fall. The current first cell is a non-penetrating full face
on a finite horizontal plane, with a supported COM and gravity normal to it.

For an unchanged world and admissible external load, require all linear and
angular velocity components to remain zero, zero penetration, and unchanged
position/orientation throughout the declared interval, including integration
and canonical frame boundaries. Contact reactions balance force and torque;
freeze, sleep, scripted pose/velocity resets and quantization hiding motion
cannot establish the proof. Sleep may optimize the already proven state.

Support or load invalidation is processed before the next affected integration
segment. Impulses, torques, separating motion, holes, finite-domain loss and
incompatible additional contacts revoke the old equilibrium. A changed load
that is still supportable requires a new feasible reaction, not automatic motion.

All six box faces are equivalent to the generic law: there is no preferred
host-game face. Exact edge/vertex equilibria are distinct unstable cells, not
full-face certificates and not a reason to invent a rotation.

Current evidence and limits: `TryBuildBoxRestingSupport` and
segment reaction pass 36 x 9000 unquantized component frames. The historical
automation names remain `RestingEquilibrium.BoxSupportCertificate` and
`RestingEquilibrium.BoxUnquantizedIntegration`. They implement the normal-load
subset of this contract, not BoxEquilibrium acquisition. The36-cell canonical
quantization campaign and native host rest checks also pass. Exact support is
enabled by default on strict SurfaceAnalytic; other backends are unchanged.

Canonical-state qualification must include non-grid-aligned support planes
and boxes with non-quantizable local rotations. Independently rounding actor
position/Euler orientation can destroy a previously valid face contact.
Unquantized integration evidence cannot substitute for this boundary check.

The contact-compatible component quantizer preserves a certified supported
face, including material-admissible sliding. During exact planar
impact acquisition, a shape may also retain the current frame's solved contact
pose and coupled linear/angular velocities: rounding either independently can
break an edge constraint. This precision policy neither projects nor settles
motion; unsupported free-flight states keep the existing quantization path.
World binding lasts from preparation through canonical publication. The
36-cell CanonicalQuantization automation also checks the ordinary path after
unbinding and rejection of separation. Future network compression must preserve
these constraints; current off-grid storage is not a network-format guarantee.
