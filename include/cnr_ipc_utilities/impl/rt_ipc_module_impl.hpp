/**
 *
 * @file coe_utilities.h
 * @brief FIle with some utility for the management of the Can Over Ethercat protocol
 *
 */

#ifndef SRC_CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_IMPL_RT_IPC_MODULE_IMPL
#define SRC_CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_IMPL_RT_IPC_MODULE_IMPL

#include <cnr_ipc_utilities/rt_ipc_module.h>

namespace cnr
{

namespace ipc
{
/****
 *
 *
 *
 *
 ****/
template <BidirAccessMode M>
template <BidirAccessMode O, typename std::enable_if<(O == BidirAccessMode::CHANNEL_CREATOR)>::type *>
inline rt_bidir_ipc_t<M>::rt_bidir_ipc_t(const std::string &identifier,
                                         const std::size_t &rx_dim,
                                         const std::size_t &tx_dim,
                                         const double &watchdog_s)
    : identifier_(identifier),
      rx_(rx_dim, identifier_ + "_rx", watchdog_s),
      tx_(tx_dim, identifier_ + "_tx", watchdog_s)
{
}

template <BidirAccessMode M>
template <BidirAccessMode O, typename std::enable_if<(O == BidirAccessMode::CHANNEL_OPENER)>::type *>
inline rt_bidir_ipc_t<M>::rt_bidir_ipc_t(const std::string &identifier, const double &watchdog_s)
    : identifier_(identifier), rx_(identifier_ + "_rx", watchdog_s), tx_(identifier_ + "_tx", watchdog_s)
{
}

template <BidirAccessMode M>
inline bool rt_bidir_ipc_t<M>::is_hard_rt()
{
  return rx_.is_hard_rt() && tx_.is_hard_rt();
}

template <BidirAccessMode M>
inline bool rt_bidir_ipc_t<M>::set_hard_rt(std::string &what)
{
  return rx_.set_hard_rt(what) && tx_.set_hard_rt(what);
}

template <BidirAccessMode M>
inline bool rt_bidir_ipc_t<M>::set_soft_rt(std::string &what)
{
  return rx_.set_soft_rt(what) && tx_.set_soft_rt(what);
}

template <BidirAccessMode M>
inline bool rt_bidir_ipc_t<M>::is_bonded()
{
  return rx_.is_bonded() && tx_.is_bonded();
}

template <BidirAccessMode M>
inline bool rt_bidir_ipc_t<M>::bond(std::string &what)
{
  return rx_.bond(what) && tx_.bond(what);
}

template <BidirAccessMode M>
inline bool rt_bidir_ipc_t<M>::break_bond()
{
  return rx_.break_bond() && tx_.break_bond();
}

template <BidirAccessMode M>
template <BidirAccessMode O, typename std::enable_if<(O == BidirAccessMode::CHANNEL_CREATOR)>::type *>
inline ErrorCode rt_bidir_ipc_t<M>::write(const uint8_t *idata_buffer,
                                          const uint8_t *safety_buffer,
                                          const double &time,
                                          const std::size_t &n_bytes,
                                          std::string &what)
{
  return tx_.write(idata_buffer, safety_buffer, time, n_bytes, what);
}

template <BidirAccessMode M>
template <BidirAccessMode O, typename std::enable_if<(O == BidirAccessMode::CHANNEL_OPENER)>::type *>
inline ErrorCode rt_bidir_ipc_t<M>::write(const uint8_t *idata_buffer,
                                          const uint8_t *safety_buffer,
                                          const double &time,
                                          const std::size_t &n_bytes,
                                          std::string &what)
{
  return rx_.write(idata_buffer, safety_buffer, time, n_bytes, what);
}


template <BidirAccessMode M>
template <BidirAccessMode O, typename std::enable_if<(O == BidirAccessMode::CHANNEL_CREATOR)>::type *>
inline ErrorCode rt_bidir_ipc_t<M>::read(uint8_t *odata_buffer,
                                         double *odata_time_label,
                                         double *odata_time_latency,
                                         const uint8_t *safety_buffer,
                                         const std::size_t &n_bytes,
                                         std::string &what)
{
  return rx_.read(odata_buffer, odata_time_label, odata_time_latency, safety_buffer, n_bytes, what);
}


template <BidirAccessMode M>
template <BidirAccessMode O, typename std::enable_if<(O == BidirAccessMode::CHANNEL_OPENER)>::type *>
inline ErrorCode rt_bidir_ipc_t<M>::read(uint8_t *odata_buffer,
                                         double *odata_time_label,
                                         double *odata_time_latency,
                                         const uint8_t *safety_buffer,
                                         const std::size_t &n_bytes,
                                         std::string &what)
{
  return tx_.write(odata_buffer, odata_time_label, odata_time_latency, safety_buffer, n_bytes, what);
}

template <BidirAccessMode M>
inline rt_bidir_ipc_list_t<M>::~rt_bidir_ipc_list_t()
{
  printf("Destroying the Modules Shared Memory ");
  {
    for (auto e : modules_)
    {
      e.reset();
    }
  }
}

template <BidirAccessMode M>
inline void rt_bidir_ipc_list_t<M>::clear()
{
  modules_.clear();
}

template <BidirAccessMode M>
inline typename rt_bidir_ipc_list_t<M>::iterator rt_bidir_ipc_list_t<M>::begin()
{
  return modules_.begin();
}

template <BidirAccessMode M>
inline typename rt_bidir_ipc_list_t<M>::iterator rt_bidir_ipc_list_t<M>::end()
{
  return modules_.end();
}

template <BidirAccessMode M>
inline typename rt_bidir_ipc_list_t<M>::const_iterator rt_bidir_ipc_list_t<M>::begin() const
{
  return modules_.begin();
}

template <BidirAccessMode M>
inline typename rt_bidir_ipc_list_t<M>::const_iterator rt_bidir_ipc_list_t<M>::end() const
{
  return modules_.end();
}

template <BidirAccessMode M>
inline typename rt_bidir_ipc_list_t<M>::const_iterator rt_bidir_ipc_list_t<M>::cbegin() const
{
  return modules_.cbegin();
}

template <BidirAccessMode M>
inline typename rt_bidir_ipc_list_t<M>::const_iterator rt_bidir_ipc_list_t<M>::cend() const
{
  return modules_.cend();
}

template <BidirAccessMode M>
inline const typename rt_bidir_ipc_t<M>::ptr &rt_bidir_ipc_list_t<M>::operator[](const std::string &i) const
{
  auto const &it = std::find_if(
      modules_.begin(), modules_.end(), [&i](typename rt_bidir_ipc_t<M>::ptr m) { return i == (m->identifier_); });
  if (it == modules_.end())
    throw std::runtime_error(("Shared memory identifier '" + i + "' not in the mapped list").c_str());
  return *it;
}

template <BidirAccessMode M>
inline typename rt_bidir_ipc_t<M>::ptr &rt_bidir_ipc_list_t<M>::operator[](const std::string &i)
{
  auto it = std::find_if(
      modules_.begin(), modules_.end(), [&i](const typename rt_bidir_ipc_t<M>::ptr &m) { return i == m->identifier_; });
  if (it == modules_.end())
    throw std::runtime_error(("Shared memory identifier '" + i + "' not in the mapped list").c_str());
  return *it;
}

template <BidirAccessMode M>
inline bool rt_bidir_ipc_list_t<M>::insert(typename rt_bidir_ipc_t<M>::ptr module_shm)
{
  for (const typename rt_bidir_ipc_t<M>::ptr &module : modules_)
  {
    if (module->identifier_ == module_shm->identifier_)
    {
      return false;
    }
  }
  modules_.push_back(module_shm);
  return true;
}

}  // namespace ipc
}  // namespace cnr

#endif /* SRC_CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_IMPL_RT_IPC_MODULE_IMPL */
