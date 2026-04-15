#pragma once

#include <apm32/f4/can/can_instances.hpp>
#include <apm32/f4/can/can_utils.hpp>

#include <emb/meta.hpp>

namespace apm32 {
namespace f4 {
namespace can {

struct transceiver_config {
  //
};

template<some_can_instance... Instances>
  requires(sizeof...(Instances) >= 1)
       && (sizeof...(Instances) <= 2)
       && (std::same_as<emb::typelist_at<emb::typelist<Instances...>, 0>, can1>)
       && emb::typelist_unique_v<emb::typelist<Instances...>>
class transceiver {
  //
};

} // namespace can
} // namespace f4
} // namespace apm32
