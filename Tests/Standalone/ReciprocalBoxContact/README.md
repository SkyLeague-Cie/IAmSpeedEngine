# Reciprocal box contact: standalone transform test

This includes the actual production `ReciprocalBoxContact.h`, `SHitResult.h`
and `ContactFeature.h`. `BoxSubBody.cpp` uses the same helper for the reciprocal
fake callback in the overlap and ordinary impulse branches.

`CoreMinimal.h` here is a storage/operation shim, not an Unreal runtime. It
substitutes vector storage/unary minus, raw weak-pointer storage, primitive
class declarations and FHitResult fields. This test does not establish Unreal
type/ABI/lifetime correctness or compile the BoxSubBody translation unit.

From an x64 MSVC Developer Command Prompt, set TEST to this directory and IAS
to the IAmSpeed repository root. Create a fresh private output directory, enter
it, then compile and run (never use an Engine output directory):

```bat
cl /nologo /EHsc /std:c++17 /W4 /WX /O2 /I"%TEST%" /I"%IAS%/Source" "%TEST%/ContactTransformTests.cpp" /Fe:contact_transform.exe
contact_transform.exe
```

The 576 cases cover overlap/blocking flags, six normal orientations (including
a nonunit normal), sixteen feature-kind pairs and three feature-index samples.
Assertions cover event preservation, input immutability, witness ownership,
participant identity, opposite normal and round-trip participant exchange.

Initial private run: 576 cases, zero failures. Three separately compiled source
mutants (missing component inversion, missing normal inversion, missing witness
swap) each fail 576 cases, demonstrating sensitivity rather than accepting a
vacuous involution. These are mutation controls, not native before/after car tests.

No collision detection, OBB geometry computation, overlap solver, impulses,
angular response, fake callback dispatch, stale-hit cache integration, CCD,
real-car geometry, demolition or Gold is executed. Native fixture correction
and the full UE source candidate remain UNCOMPILED/UNRUN. The slot2 integrity
hold is independent and is not relaxed by this unit result.
