#ifndef SRC_CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_RT_IPC
#define SRC_CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_RT_IPC

#include <type_traits>

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/upgradable_lock.hpp>

#include <tuple>

namespace cnr
{
namespace ipc
{

/**
 * @class rt_ipc_t
 *
 */
enum class AccessMode : int
{
  CREATE_AND_SYNC_WRITE = 0,
  CREATE_AND_SYNC_READ,
  OPEN_AND_SYNC_WRITE,
  OPEN_AND_SYNC_READ, 
};

enum class ErrorCode : int
{
  OK = 0,
  ERROR_UNMACTHED_DATA_DIMENSION = 1,
  ERROR_UNCORRECT_CALL = 2,
  ERROR_WRITING_WATCHDOG = 3,
  ERROR_READING_WATCHDOG = 4,
  ERROR_UNMATCHED_TIME = 5,
  WARNING_NOT_BONDED = 6
};

std::string to_string(ErrorCode err);
int as_integer(ErrorCode const value);

template<AccessMode M>
class ipc_t
{
public:
  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::CREATE_AND_SYNC_WRITE) || (O==AccessMode::CREATE_AND_SYNC_READ)>::type* = nullptr>
  ipc_t(const std::size_t& dim, const std::string& name);

  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::OPEN_AND_SYNC_WRITE) || (O==AccessMode::OPEN_AND_SYNC_READ)>::type* = nullptr>
  ipc_t(const std::string& name);

  ~ipc_t();

  std::size_t dim() const;
  void get(void *shmem);
  void set(const void *shmem);

private:
  const std::string name_;
  boost::interprocess::mapped_region shared_map_;
  boost::interprocess::shared_memory_object shared_memory_;
  std::shared_ptr<boost::interprocess::named_mutex> mutex_;
};


struct rt_data_t
{
  struct Header
  {
    uint8_t bond_flag_;
    uint8_t rt_flag_;
    double time_;
  } __attribute__((packed)) header_;

  char buffer_[1024];

  explicit rt_data_t() { clear(); }
  rt_data_t(const rt_data_t & ) = delete;
  rt_data_t(rt_data_t&&) = delete;
  void clear()
  {
    header_.bond_flag_ = 0;
    header_.rt_flag_ = 0;
    header_.time_ = 0;
    std::memset(&buffer_[0], 0x0, 1024 * sizeof(char));
  }
};

std::string to_string(const rt_data_t& in);


class watchdog_t
{
public:
    watchdog_t(const double watchdog_s, const double multiplier = 1.1);
    double now();
    void tic();
    bool toc();
    double dt() const;
    double threshold() const; 
private:
    double tic_, toc_;
    const double watchdog_;
    const double multiplier_;
};


template<AccessMode M>
class rt_ipc_t
{
 public:
  using ptr = std::shared_ptr< rt_ipc_t<M> >;

  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::CREATE_AND_SYNC_WRITE)>::type* = nullptr>
  rt_ipc_t(const std::size_t& dim, const std::string &name, const double& watchdog_s);

  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::OPEN_AND_SYNC_WRITE)>::type* = nullptr>
  rt_ipc_t(const std::string& name, const double& watchdog_s);

  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::CREATE_AND_SYNC_READ)>::type* = nullptr>
  rt_ipc_t(const std::size_t& dim, const std::string &name, const double& watchdog_s);

  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::OPEN_AND_SYNC_READ)>::type* = nullptr>
  rt_ipc_t(const std::string& name, const double& watchdog_s);


  rt_ipc_t() = delete;
  rt_ipc_t(const rt_ipc_t&) = delete;
  rt_ipc_t(rt_ipc_t&&) = delete;

  ~rt_ipc_t();

  bool is_hard_rt();
  bool set_hard_rt(std::string& what);
  bool set_soft_rt(std::string& what);

  bool is_bonded();
  bool bond(std::string& what);
  bool break_bond();

  /**
   * @brief 
   * 
   * @param buffer 
   * @param time 
   * @param n_bytes 
   * @param what 
   * @return ErrorCode 
   */
  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::CREATE_AND_SYNC_WRITE) || (O==AccessMode::OPEN_AND_SYNC_WRITE)>::type*  = nullptr>
  ErrorCode write(const uint8_t *idata_buffer, const uint8_t *safety_buffer, const double& time, const std::size_t &n_bytes, std::string& what);

  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::OPEN_AND_SYNC_READ) || (O==AccessMode::CREATE_AND_SYNC_READ)>::type* = nullptr>
  ErrorCode write(const uint8_t *idata_buffer, const uint8_t *safety_buffer, const double& time, const std::size_t &n_bytes, std::string& what)
  {
    return ErrorCode::ERROR_UNCORRECT_CALL;
  }

  /**
   * @brief The function copies the content of the shared memory over the buffer (first argument)
   * 
   * NOTE:
   * - If the shared memory is not bonded, the buffer is set to zero. OK is returned.
   * - The behavior is different if the object is the ipc_t CREATOR/WRITER or CLIENT/READER
   *      * If the object is the WRITER, does not raise error if the deadline is not met. Probably, the process of the reader is already acting to overcome the watchdog error. The WRITER just add the label with the time he wrote the data. 
   * 
   * @param data_buffer the data in the shared memory
   * @param data_time_buffer Time in the shared memory, corresponding to the last shared memory write operation
   * @param data_time_latency Time Difference between the actual writing time and the previous writing time
   * @param n_bytes number of bytes to be copied. Forcing thuis information is for security matter.
   * @param what string with a description of the error/warning
   * @return ErrorCode 
   */
  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::OPEN_AND_SYNC_READ) || (O==AccessMode::CREATE_AND_SYNC_READ)>::type* = nullptr>
  ErrorCode read(uint8_t *odata_buffer, double *odata_time_label, double *odata_time_latency, const uint8_t *safety_buffer, const std::size_t &n_bytes, std::string& what);
  
  template<AccessMode O=M, typename std::enable_if<(O==AccessMode::OPEN_AND_SYNC_WRITE) || (O==AccessMode::CREATE_AND_SYNC_WRITE)>::type* = nullptr>
  ErrorCode read(uint8_t *odata_buffer, double *odata_time_label, double *odata_time_latency, const uint8_t *safety_buffer, const std::size_t &n_bytes, std::string& what)
  {
    return ErrorCode::ERROR_UNCORRECT_CALL;
  }

  std::size_t dim(bool prepend_header) const;
  std::string name() const;
  //double getWatchdog() const;
  
  void dump(rt_data_t *buffer) { return data_accessor_.get(buffer); }

 protected:
  const std::string name_;
  std::size_t dim_buffer_;

  ipc_t<M> data_accessor_;
  struct 
  {
    double update_time_;
    rt_data_t data_;
  } last_data_available_;

  watchdog_t watchdog_;

  bool bonded_prev_;
  bool is_hard_rt_prev_;

  bool set_rt(std::string& what, uint8_t hard);
};

template<AccessMode M> 
using rt_ipc_ptr_t = typename rt_ipc_t< M >::ptr;

using rt_ipc_creator_and_reader_t = rt_ipc_t<cnr::ipc::AccessMode::CREATE_AND_SYNC_READ>;
using rt_ipc_creator_and_writer_t = rt_ipc_t<cnr::ipc::AccessMode::CREATE_AND_SYNC_WRITE>;
using rt_ipc_opener_and_reader_t = rt_ipc_t<cnr::ipc::AccessMode::OPEN_AND_SYNC_READ>;
using rt_ipc_opener_and_writer_t = rt_ipc_t<cnr::ipc::AccessMode::OPEN_AND_SYNC_WRITE>;

using rt_ipc_creator_and_reader_ptr_t = rt_ipc_ptr_t<cnr::ipc::AccessMode::CREATE_AND_SYNC_READ>;
using rt_ipc_creator_and_writer_ptr_t = rt_ipc_ptr_t<cnr::ipc::AccessMode::CREATE_AND_SYNC_WRITE>;
using rt_ipc_opener_and_reader_ptr_t = rt_ipc_ptr_t<cnr::ipc::AccessMode::OPEN_AND_SYNC_READ>;
using rt_ipc_opener_and_writer_ptr_t = rt_ipc_ptr_t<cnr::ipc::AccessMode::OPEN_AND_SYNC_WRITE>;



}  // namespace ipc
}  // namespace cnr

#include <cnr_ipc_utilities/impl/rt_ipc_impl.hpp>

#endif  /* SRC_CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_RT_IPC */
