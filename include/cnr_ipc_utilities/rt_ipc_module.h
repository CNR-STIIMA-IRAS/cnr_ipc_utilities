
#ifndef SRC_CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_RT_IPC_MODULE
#define SRC_CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_RT_IPC_MODULE

#include <cnr_ipc_utilities/rt_ipc.h>

namespace cnr
{

namespace ipc
{

/**
 * @brief 
 * 
 */
enum class BidirAccessMode : int
{
  CHANNEL_CREATOR = 0,
  CHANNEL_OPENER,
};

/**
 * @brief 
 * 
 * @tparam M 
 */
template <BidirAccessMode M>
class rt_bidir_ipc_t
{
 public:
  using rx_t = typename std::conditional<M == cnr::ipc::BidirAccessMode::CHANNEL_CREATOR,
                                         rt_ipc_creator_and_reader_t,
                                         rt_ipc_opener_and_writer_t>::type;
  using tx_t = typename std::conditional<M == cnr::ipc::BidirAccessMode::CHANNEL_CREATOR,
                                         rt_ipc_creator_and_writer_t,
                                         rt_ipc_opener_and_reader_t>::type;
  using ptr = std::shared_ptr<rt_bidir_ipc_t<M> >;

  const std::string identifier_;

  rx_t rx_;
  tx_t tx_;

  rt_bidir_ipc_t() = delete;
  rt_bidir_ipc_t(const rt_bidir_ipc_t &) = delete;
  rt_bidir_ipc_t(rt_bidir_ipc_t &&) = delete;

  template <BidirAccessMode O = M, typename std::enable_if<(O == BidirAccessMode::CHANNEL_CREATOR)>::type * = nullptr>
  rt_bidir_ipc_t(const std::string &identifier,
                 const std::size_t &rx_dim,
                 const std::size_t &tx_dim,
                 const double &watchdog_s);

  template <BidirAccessMode O = M, typename std::enable_if<(O == BidirAccessMode::CHANNEL_OPENER)>::type * = nullptr>
  rt_bidir_ipc_t(const std::string &identifier, const double &watchdog_s);

  ~rt_bidir_ipc_t() = default;

  bool is_hard_rt();
  bool set_hard_rt(std::string &what);
  bool set_soft_rt(std::string &what);

  bool is_bonded();
  bool bond(std::string &what);
  bool break_bond();

  template <BidirAccessMode O = M, typename std::enable_if<(O == BidirAccessMode::CHANNEL_CREATOR)>::type * = nullptr>
  ErrorCode write(const uint8_t *idata_buffer,
                  const uint8_t *safety_buffer,
                  const double &time,
                  const std::size_t &n_bytes,
                  std::string &what);

  template <BidirAccessMode O = M, typename std::enable_if<(O == BidirAccessMode::CHANNEL_OPENER)>::type * = nullptr>
  ErrorCode write(const uint8_t *idata_buffer,
                  const uint8_t *safety_buffer,
                  const double &time,
                  const std::size_t &n_bytes,
                  std::string &what);

  template <BidirAccessMode O = M, typename std::enable_if<(O == BidirAccessMode::CHANNEL_CREATOR)>::type * = nullptr>
  ErrorCode read(uint8_t *odata_buffer,
                 double *odata_time_label,
                 double *odata_time_latency,
                 const uint8_t *safety_buffer,
                 const std::size_t &n_bytes,
                 std::string &what);

  template <BidirAccessMode O = M, typename std::enable_if<(O == BidirAccessMode::CHANNEL_OPENER)>::type * = nullptr>
  ErrorCode read(uint8_t *odata_buffer,
                 double *odata_time_label,
                 double *odata_time_latency,
                 const uint8_t *safety_buffer,
                 const std::size_t &n_bytes,
                 std::string &what);
};

using rt_bidir_ipc_creator_t = rt_bidir_ipc_t<BidirAccessMode::CHANNEL_CREATOR>;
using rt_bidir_ipc_opener_t = rt_bidir_ipc_t<BidirAccessMode::CHANNEL_OPENER>;

template<BidirAccessMode M> 
using rt_bidir_ipc_ptr_t = typename rt_bidir_ipc_t< M >::ptr;
using rt_bidir_ipc_creator_ptr_t = rt_bidir_ipc_ptr_t<BidirAccessMode::CHANNEL_CREATOR>;
using rt_bidir_ipc_opener_ptr_t = rt_bidir_ipc_ptr_t<BidirAccessMode::CHANNEL_OPENER>;

/**
 * @brief 
 * 
 * @tparam M 
 */
template <BidirAccessMode M>
class rt_bidir_ipc_list_t
{
 public:
  using ptr = std::shared_ptr<rt_bidir_ipc_list_t<M> >;
  using List = std::vector< rt_bidir_ipc_ptr_t<M> >;
  using iterator = typename List::iterator ;
  using const_iterator = typename List::const_iterator;

  ~rt_bidir_ipc_list_t();

  void clear();
  iterator begin();
  iterator end();

  const_iterator begin() const;
  const_iterator end() const;

  const_iterator cbegin() const;
  const_iterator cend() const;

  const typename rt_bidir_ipc_t<M>::ptr &operator[](const std::string &i) const;
  typename rt_bidir_ipc_t<M>::ptr &operator[](const std::string &i);

  bool insert(typename rt_bidir_ipc_t<M>::ptr module_shm);

 private:
  List modules_;
};

using rt_master_ipc_t = rt_bidir_ipc_list_t<BidirAccessMode::CHANNEL_CREATOR>;
using rt_slave_ipc_t = rt_bidir_ipc_list_t<BidirAccessMode::CHANNEL_OPENER>;

template<BidirAccessMode M> 
using rt_bidir_ipc_list_ptr_t = typename rt_bidir_ipc_list_t< M >::ptr;
using rt_master_ipc_ptr_t = rt_bidir_ipc_list_ptr_t<BidirAccessMode::CHANNEL_CREATOR>;
using rt_slave_ipc_ptr_t = rt_bidir_ipc_list_ptr_t<BidirAccessMode::CHANNEL_OPENER>;

}  // namespace ipc
}  // namespace cnr

#include <cnr_ipc_utilities/impl/rt_ipc_module_impl.hpp>

#endif  /* SRC_CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_RT_IPC_MODULE */
