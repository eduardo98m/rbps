#pragma once
#include "visr/Snapshot.hpp"
#include "visr/Command.hpp"
#include <type_traits>

/**
 * @file Transport.hpp
 * @brief Transport contract (`push_snapshot` + `poll_command`) and detection trait.
 * @ingroup visr
 *
 * Any struct that provides
 * - `void push_snapshot(const FrameSnapshot&)` — physics → render direction.
 * - `bool poll_command(Command&)`              — render → physics direction.
 *
 * is a valid transport. Enforced with a C++17 detection-idiom trait
 * (`is_debug_transport<T>`) instead of a C++20 concept. The
 * `ASSERT_DEBUG_TRANSPORT(T)` macro lets call-sites surface the contract
 * mismatch as a clear compile error.
 */

namespace visr
{
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
 
    /**
     * @brief Compile-time predicate: does `T` satisfy the `DebugTransport` contract?
     * @ingroup visr
     */
    template<typename T>
    struct is_debug_transport
        : std::bool_constant<
            detail::has_push_snapshot<T>::value &&
            detail::has_poll_command<T>::value>
    {};

    /**
     * @def ASSERT_DEBUG_TRANSPORT
     * @brief Static-assert that `T` is a valid transport.
     *
     * Drop at the top of any function template that takes a transport so
     * the contract violation reports at the call site instead of deep
     * inside the implementation.
     *
     * @ingroup visr
     */
    #define ASSERT_DEBUG_TRANSPORT(T) \
        static_assert(::visr::is_debug_transport<T>::value, \
                      #T " must implement push_snapshot() and poll_command()")

    /**
     * @brief Zero-overhead transport that drops everything.
     *
     * Default template parameter for `DebugChannel`. Useful when the
     * visualizer code is compiled in but no physical channel is needed
     * (e.g. headless tests).
     *
     * @ingroup visr
     */
    struct NullTransport
    {
        /** @brief Drops the snapshot. */
        void push_snapshot(const FrameSnapshot &) noexcept {}
        /** @brief Always returns `false` — no commands ever delivered. */
        bool poll_command(Command &)              noexcept { return false; }
    };
 
    static_assert(is_debug_transport<NullTransport>::value,
                  "NullTransport must satisfy the transport contract");
 
} // namespace visr