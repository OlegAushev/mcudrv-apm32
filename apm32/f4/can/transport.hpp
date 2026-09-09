#pragma once

#include <apm32/f4/can/transceiver.hpp>

#include <emb/can.hpp>
#include <emb/can/bus.hpp>
#include <emb/container/inplace_vector.hpp>
#include <emb/delegate.hpp>

#include <cstddef>

namespace apm32::f4::can {

// This peripheral seen as emb::can::transport: the bus a protocol stack
// talks to without knowing which controller carries it.
//
// Its own class rather than a base of transceiver, for two reasons. The
// interface is virtual, and folding it in would give every transceiver a
// vtable whether or not anything asks for the abstraction. And it is built
// from two objects, not one — the transceiver moves frames, the filter
// setup decides which arrive — so there is no single peripheral for the
// interface to be a face of.
//
// Frames accepted by the hardware filters reach every subscriber in the
// order they registered. MaxSubscribers sizes that list at compile time and
// belongs to whoever assembles the stack, which is why it is a parameter
// and not a constant here; a subscriber past the bound is dropped rather
// than reported, since registering is a wiring step done once at startup.
template<some_can_instance Instance,
         transceiver_traits Traits,
         std::size_t MaxSubscribers = 1>
class transport : public emb::can::transport {
public:
  using transceiver_t = transceiver<Instance, Traits>;
  using filter_setup_t = filter_setup<Instance, Traits>;

  transport(transceiver_t& xcvr, filter_setup_t& filters)
      : xcvr_(xcvr), filters_(filters)
  {
    xcvr_.on_rx_fifo0(emb::make_delegate<&transport::dispatch_rx>(this));
  }

  transport(transport const&) = delete;
  transport& operator=(transport const&) = delete;

  bool send(emb::can::frame_t const& frame) override
  {
    return xcvr_.put(frame).has_value();
  }

  void subscribe(emb::delegate<void(emb::can::frame_t const&)> handler) override
  {
    (void)subscribers_.try_push_back(handler);
  }

  void add_filter(emb::can::format_t format,
                  emb::can::id_t id,
                  emb::can::id_t mask) override
  {
    filters_.add(filter_32_mask{.format = format, .id = id, .mask = mask},
                 rx_fifo::_0);
  }

private:
  void dispatch_rx(emb::can::frame_t const& frame)
  {
    for (auto& sub : subscribers_)
      sub(frame);
  }

  transceiver_t& xcvr_;
  filter_setup_t& filters_;
  emb::inplace_vector<emb::delegate<void(emb::can::frame_t const&)>,
                      MaxSubscribers>
      subscribers_;
};

} // namespace apm32::f4::can
