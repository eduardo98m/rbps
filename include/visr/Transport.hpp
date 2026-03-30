#pragma once
#include "visr/Snapshot.hpp"
#include "visr/Command.hpp"
#include <type_traits>
 
// ============================================================================
//  visr/Transport.hpp  (C++17 compatible — no C++20 concepts)
//
//  A "DebugTransport" is any struct that provides:
//    void push_snapshot(const FrameSnapshot&)
//    bool poll_command(Command&)
//
//  We enforce this contract with a trait + static_assert rather than a C++20
//  concept.  The macro ASSERT_DEBUG_TRANSPORT(T) can be placed anywhere you
//  want a clear compile error if a transport type is wrong.
//
// ============================================================================
 
namespace visr
{
    // -------------------------------------------------------------------------
    //  Transport trait  (detection idiom, C++17)
    // -------------------------------------------------------------------------
    namespace detail
    {
        template<typename T, typename = void>
        struct has_push_snapshot : std::false_type {};
 
        template<typename T>
        struct has_push_snapshot<T,
            std::void_t<decltype(std::declval<T>().push_snapshot(
                            std::declval<const FrameSnapshot&>()))>>
            : std::true_type {};
 
        template<typename T, typename = void>
        struct has_poll_command : std::false_type {};
 
        template<typename T>
        struct has_poll_command<T,
            std::void_t<decltype(std::declval<T>().poll_command(
                            std::declval<Command&>()))>>
            : std::true_type {};
 
    } // namespace detail
 
    template<typename T>
    struct is_debug_transport
        : std::bool_constant<
            detail::has_push_snapshot<T>::value &&
            detail::has_poll_command<T>::value>
    {};
 
    // Convenience macro — drop at the top of any function template that takes
    // a transport so the error points at the call site.
    #define ASSERT_DEBUG_TRANSPORT(T) \
        static_assert(::visr::is_debug_transport<T>::value, \
                      #T " must implement push_snapshot() and poll_command()")
 
    // -------------------------------------------------------------------------
    //  NullTransport — zero-overhead default
    // -------------------------------------------------------------------------
    struct NullTransport
    {
        void push_snapshot(const FrameSnapshot &) noexcept {}
        bool poll_command(Command &)              noexcept { return false; }
    };
 
    static_assert(is_debug_transport<NullTransport>::value,
                  "NullTransport must satisfy the transport contract");
 
} // namespace visr