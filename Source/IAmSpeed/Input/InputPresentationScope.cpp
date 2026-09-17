#include "InputPresentationScope.h"

namespace
{
thread_local unsigned PresentationInputDepth = 0;
}

Speed::Input::FPresentationInputScope::FPresentationInputScope() { ++PresentationInputDepth; }
Speed::Input::FPresentationInputScope::~FPresentationInputScope() { --PresentationInputDepth; }
bool Speed::Input::FPresentationInputScope::IsActive() { return PresentationInputDepth != 0; }
