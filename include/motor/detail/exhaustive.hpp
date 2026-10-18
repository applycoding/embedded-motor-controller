#pragma once

namespace motor {
namespace detail {

// Default arm of an enum switch. -Wswitch-enum still fails the build when a
// new enumerator is not listed. This call marks that path unreachable.
template <typename Enum>
[[noreturn]] inline void unhandled_enum(Enum value) {
    static_cast<void>(value);
#if defined(__GNUC__) || defined(__clang__)
    __builtin_unreachable();
#else
    for (;;) {
    }
#endif
}

}  // namespace detail
}  // namespace motor
