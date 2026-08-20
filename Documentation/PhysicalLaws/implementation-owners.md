# PhysicalLaw implementation owners

This ledger is many-to-many. It identifies current public implementation
owners without claiming that a law belongs to exactly one class.

| Owner | PhysicalLaw identifiers |
| --- | --- |
| `FWorldQueryService` | ContactDetection, Support |
| `USpeedWorldSubsystem` | ContactDetection, NormalResponse, Sliding, RestingEquilibrium, Rolling, Support, MultiContactEquilibrium |
| `ISpeedComponent` / `USpeedMovementComponent` | NormalResponse, Sliding, RestingEquilibrium, Support, MultiContactEquilibrium |
| `UBoxSubBody` | ContactDetection, NormalResponse, Friction, Sliding, Support, MultiContactEquilibrium |
| `USphereSubBody` | ContactDetection, NormalResponse, Friction, Sliding, RestingEquilibrium, Rolling, Support |
| `USWheelSubBody` / ray-wheel implementation | ContactDetection, Friction, Rolling, Support, Suspension |
| `ISpeedWheeledComponent` / `USpeedWheeledComponent` | Friction, Sliding, RestingEquilibrium, Rolling, Support, MultiContactEquilibrium, Suspension |
| static-world backend and Unreal legacy adapter | ContactDetection, Support |

Names in this table refer to generic IAmSpeed API or implementation types. A
host project maps its own mechanics to stable PhysicalLaw identifiers outside
the plugin.
