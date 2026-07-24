#pragma once

#include <apm32/f4/adc/multi_channel_adc.hpp>
#include <apm32/f4/gpio/output_pin.hpp>

#include <emb/gpio.hpp>
#include <emb/meta/typelist.hpp>
#include <emb/meta/unroll.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace apm32::f4::adc {

// Binds a regular external channel of a multi_channel_adc to the index of
// the drive pin that stimulates it on the bench.
template<some_adc_channel Channel, std::size_t Pin>
struct verifier_binding {
  using channel = Channel;
  static constexpr std::size_t pin = Pin;
};

template<typename T>
inline constexpr bool is_verifier_binding = false;

template<typename C, std::size_t P>
inline constexpr bool is_verifier_binding<verifier_binding<C, P>> = true;

template<typename T>
concept some_verifier_binding = is_verifier_binding<T>;

// Verifies the regular sequence -> DMA buffer layout of a multi_channel_adc
// without external test equipment. N spare GPIO outputs (drive pins) are
// jumpered to the ADC inputs of the M bound channels (M >= N, channels may
// share a drive pin); every drive pin holds a known level, so each bound
// rank must read back a near-rail code matching its channel's pin. step()
// advances a one-hot pattern across the pins (with a single pin it toggles
// the level instead), so every channel is exercised at both levels and a
// rank/cell misalignment across differently-driven channels breaks the
// expected picture on some step. With a single drive pin a misalignment
// within the equally-driven group stays invisible; two or more pins restore
// full discrimination.
//
// The verifier owns no signalling and no scheduling policy: the caller
// invokes step()/check() at its own pace and renders the verdicts. After
// each step() the next settle_checks calls of check() return settling,
// covering the sequence in flight and the level switch transient.
template<some_multi_channel_adc Adc, some_verifier_binding... Bindings>
  requires(sizeof...(Bindings) > 0 && Adc::dma_enabled)
class multi_channel_adc_verifier {
public:
  using bindings = emb::typelist<Bindings...>;
  static constexpr std::size_t binding_count = sizeof...(Bindings);
  static constexpr std::size_t pin_count = std::max({Bindings::pin...}) + 1;

  static_assert(
      (... && (Bindings::channel::descriptor::type
               == channel_type::external)),
      "only external channels can be stimulated by drive pins"
  );
  static_assert(
      (... && !Bindings::channel::injected),
      "only regular channels are covered by the regular DMA buffer"
  );
  static_assert(
      (... && emb::typelist_contains_v<
                  typename Adc::channels,
                  typename Bindings::channel>),
      "every bound channel must belong to the verified ADC"
  );
  static_assert(
      []() consteval {
        std::array<bool, pin_count> used{};
        ((used[Bindings::pin] = true), ...);
        return std::ranges::all_of(used, [](bool u) { return u; });
      }(),
      "drive pin indices must be contiguous starting at 0"
  );

  enum class check_status { settling, pass, fail };

  static constexpr std::uint32_t low_code_max = 100;
  static constexpr std::uint32_t high_code_min = 3995;
private:
  // single pin: step() toggles the level; multiple pins: one-hot rotation
  static constexpr std::size_t step_period = pin_count == 1 ? 2 : pin_count;

  Adc& adc_;
  std::array<std::optional<gpio::output_pin>, pin_count> drive_pins_;
  std::uint32_t const settle_checks_;
  std::size_t step_ = 0;
  std::uint32_t skip_checks_;
  std::uint32_t pass_count_ = 0;
  std::uint32_t fail_count_ = 0;
  unsigned last_failed_rank_ = 0;
public:
  multi_channel_adc_verifier(
      Adc& adc,
      std::span<gpio::output_pin_config const, pin_count> drive_pin_configs,
      std::uint32_t settle_checks = 2
  )
      : adc_(adc),
        settle_checks_(settle_checks),
        skip_checks_(settle_checks) {
    for (auto i = 0uz; i < pin_count; ++i) {
      drive_pins_[i].emplace(drive_pin_configs[i]);
    }
    apply_pattern();
  }

  void step() {
    step_ = (step_ + 1) % step_period;
    apply_pattern();
    skip_checks_ = settle_checks_;
  }

  [[nodiscard]] check_status check() {
    if (skip_checks_ > 0) {
      --skip_checks_;
      return check_status::settling;
    }

    std::optional<unsigned> failed_rank;

    emb::unroll<binding_count>([&, this]<std::size_t B>() {
      using binding = emb::typelist_at_t<bindings, B>;
      using channel = binding::channel;
      emb::unroll<channel::ranks.size()>([&, this]<std::size_t R>() {
        constexpr auto rank = channel::ranks[R];
        std::uint32_t const code =
            *adc_.template regular_storage<rank>();
        bool const ok = forced_high(binding::pin)
                            ? code >= high_code_min
                            : code <= low_code_max;
        if (!ok && !failed_rank.has_value()) {
          failed_rank = rank;
        }
      });
    });

    if (failed_rank.has_value()) {
      ++fail_count_;
      last_failed_rank_ = *failed_rank;
      return check_status::fail;
    }
    ++pass_count_;
    return check_status::pass;
  }

  std::uint32_t pass_count() const {
    return pass_count_;
  }

  std::uint32_t fail_count() const {
    return fail_count_;
  }

  unsigned last_failed_rank() const {
    return last_failed_rank_;
  }
private:
  bool forced_high(std::size_t pin) const {
    return pin_count == 1 ? step_ == 1 : pin == step_;
  }

  void apply_pattern() {
    for (auto i = 0uz; i < pin_count; ++i) {
      drive_pins_[i]->set_level(
          forced_high(i) ? emb::gpio::level::high : emb::gpio::level::low
      );
    }
  }
};

} // namespace apm32::f4::adc
