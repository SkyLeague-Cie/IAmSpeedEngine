#pragma once

// One exported TLS definition is needed across Unreal module/DLL boundaries.
#ifndef IAMSPEED_API
#define IAMSPEED_INPUT_PORTABLE_API
#else
#define IAMSPEED_INPUT_PORTABLE_API IAMSPEED_API
#endif

namespace Speed::Input
{
/** Synchronous presentation callbacks may observe input but never publish it.
 * This is a guard on the supported input API, not a sandbox for arbitrary C++.
 * Callbacks must not schedule deferred writes or mutate physics through other APIs.
 */
class IAMSPEED_INPUT_PORTABLE_API FPresentationInputScope final
{
public:
	FPresentationInputScope();
	~FPresentationInputScope();
	FPresentationInputScope(const FPresentationInputScope&) = delete;
	FPresentationInputScope& operator=(const FPresentationInputScope&) = delete;
	static bool IsActive();
};
}
#undef IAMSPEED_INPUT_PORTABLE_API
