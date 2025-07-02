#ifndef CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_IMPL_RT_IPC_IMPL
#define CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_IMPL_RT_IPC_IMPL

#include <cnr_ipc_utilities/rt_ipc.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

namespace cnr
{
namespace ipc
{

inline std::string RESET() { return "\033[0m"; }
inline std::string BLACK() { return "\033[30m"; }
inline std::string RED() { return "\033[31m"; }
inline std::string GREEN() { return "\033[32m"; }
inline std::string YELLOW() { return "\033[33m"; }
inline std::string BLUE() { return "\033[34m"; }
inline std::string MAGENTA() { return "\033[35m"; }
inline std::string CYAN() { return "\033[36m"; }
inline std::string WHITE() { return "\033[37m"; }
inline std::string BOLDBLACK() { return "\033[1m\033[30m"; }
inline std::string BOLDRED() { return "\033[1m\033[31m"; }
inline std::string BOLDGREEN() { return "\033[1m\033[32m"; }
inline std::string BOLDYELLOW() { return "\033[1m\033[33m"; }
inline std::string BOLDBLUE() { return "\033[1m\033[34m"; }
inline std::string BOLDMAGENTA() { return "\033[1m\033[35m"; }
inline std::string BOLDCYAN() { return "\033[1m\033[36m"; }
inline std::string BOLDWHITE() { return "\033[1m\033[37m"; }

inline double timer_to_s(const timespec* timeA_p)
{
  return ((double)(timeA_p->tv_sec) + ((double)timeA_p->tv_nsec) / 1.e9);
}

inline double now_s()
{
  struct timespec _t;
  clock_gettime(CLOCK_MONOTONIC, &_t);
  return timer_to_s(&_t);
}

inline void* cast_and_set_time(rt_data_t* in)
{
  in->header_.time_ = now_s();
  return static_cast<void*>(in);
}

inline std::string to_string(const rt_data_t& in)
{
  std::string ret = "Bonded: " + std::to_string(in.header_.bond_flag_);
  ret += ", RT: " + std::to_string(in.header_.rt_flag_);
  ret += ", T: " + std::to_string(in.header_.time_);
  ret += ", val: " + std::string(static_cast<const char*>(in.buffer_));
  return ret;
}

void copy(rt_data_t* data,
          const uint8_t* buffer,
          const std::size_t& dim_buffer,
          const double* time = nullptr,
          const uint8_t* bond_flag = nullptr,
          const uint8_t* rt_flag = nullptr)
{
  assert(data);
  assert(buffer);
  data->header_.time_ = time ? *time : now_s();
  data->header_.bond_flag_ = bond_flag ? *bond_flag : data->header_.bond_flag_;
  data->header_.rt_flag_ = rt_flag ? *rt_flag : data->header_.rt_flag_;
  std::memset(data->buffer_, 0x0, 1024 * sizeof(char));
  std::memcpy(data->buffer_, buffer, dim_buffer);
}

void copy(rt_data_t* dest, const rt_data_t* src)
{
  assert(dest);
  assert(src);
  dest->clear();
  std::memcpy(static_cast<void*>(dest), static_cast<const void*>(src), sizeof(rt_data_t));
}

inline watchdog_t::watchdog_t(const double watchdog_s, const double multiplier)
    : tic_(-1), toc_(-1), watchdog_(watchdog_s), multiplier_(multiplier)
{
  assert(multiplier_ > 0);
}

inline void watchdog_t::tic()
{
  struct timespec _tic;
  clock_gettime(CLOCK_MONOTONIC, &_tic);
  tic_ = timer_to_s(&_tic);
}

inline bool watchdog_t::toc()
{
  toc_ = now_s();
  if (tic_ < 0)
  {
    return true;
  }

  return std::fabs(toc_ - tic_) < multiplier_ * watchdog_;
}


inline double watchdog_t::dt() const
{
  return std::fabs(toc_ - tic_);
}

inline double watchdog_t::now() { return std::max(tic_, toc_); }

inline double watchdog_t::threshold() const { return multiplier_ * watchdog_; }

template <AccessMode M>
template <
    AccessMode O,
    typename std::enable_if<(O == AccessMode::CREATE_AND_SYNC_WRITE) || (O == AccessMode::CREATE_AND_SYNC_READ)>::type*>
inline ipc_t<M>::ipc_t(const std::size_t& dim, const std::string& name) : name_(name)
{
  std::stringstream err;
  err << __PRETTY_FUNCTION__ << std::endl;
  err << "\t*** Module '" << name_ << "'" << std::endl;
  err << "\t*** Dimension in Bytes '" << dim << "'" << std::endl;
  std::size_t l = __LINE__;
  if (dim == 0)
  {
    err << "\t!!! Zero byte memory, is it correct?" << std::endl;
    throw std::runtime_error(err.str().c_str());
  }

  try
  {
    l = __LINE__;
    boost::interprocess::permissions permissions(0677);
    l = __LINE__;
    boost::interprocess::named_mutex::remove(name.c_str());
    l = __LINE__;
    boost::interprocess::shared_memory_object::remove(name.c_str());
    l = __LINE__;
    // store old
    mode_t old_umask = umask(0);

    l = __LINE__;
    shared_memory_ = boost::interprocess::shared_memory_object(
        boost::interprocess::create_only, name.c_str(), boost::interprocess::read_write, permissions);

    l = __LINE__;
    shared_memory_.truncate(dim);

    l = __LINE__;
    shared_map_ = boost::interprocess::mapped_region(shared_memory_, boost::interprocess::read_write);

    l = __LINE__;
    std::memset(shared_map_.get_address(), 0, shared_map_.get_size());

    l = __LINE__;
    mutex_.reset(new boost::interprocess::named_mutex(boost::interprocess::create_only, name_.c_str(), permissions));

    // restore old
    l = __LINE__;
    umask(old_umask);
  }
  catch (boost::interprocess::interprocess_exception& e)
  {
    err << "\t!!! Boost Error: " << e.what() << std::endl;
    err << "\t!!! Last Line Executed: " << l << std::endl;
    throw std::runtime_error(err.str().c_str());
  }
  catch (std::exception& e)
  {
    err << "\t!!! Error: " << e.what() << std::endl;
    err << "\t!!! Last Line Executed: " << l << std::endl;
    throw std::runtime_error(err.str().c_str());
  }
}

template <AccessMode M>
template <
    AccessMode O,
    typename std::enable_if<(O == AccessMode::OPEN_AND_SYNC_WRITE) || (O == AccessMode::OPEN_AND_SYNC_READ)>::type*>
inline ipc_t<M>::ipc_t(const std::string& name) : name_(name)
{
  std::stringstream err;
  err << __PRETTY_FUNCTION__ << std::endl;
  err << "\t*** Module '" << name_ << "'" << std::endl;
  std::size_t l = __LINE__;
  try
  {
    l = __LINE__;
    boost::interprocess::permissions permissions(0677);

    l = __LINE__;
    shared_memory_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only, name.c_str(), boost::interprocess::read_write);

    l = __LINE__;
    shared_map_ = boost::interprocess::mapped_region(shared_memory_, boost::interprocess::read_write);

    l = __LINE__;
    mutex_.reset(new boost::interprocess::named_mutex(boost::interprocess::open_only, name_.c_str()));
  }
  catch (boost::interprocess::interprocess_exception& e)
  {
    err << "\t!!! Boost Error: " << e.what() << std::endl;
    err << "\t!!! Last Line Executed: " << l << std::endl;
    throw std::runtime_error(err.str().c_str());
  }
  catch (std::exception& e)
  {
    err << "\t!!! Error: " << e.what() << std::endl;
    err << "\t!!! Last Line Executed: " << l << std::endl;
    throw std::runtime_error(err.str().c_str());
  }

  if (dim() == 0)
  {
    err << "\t!!! Error: "
        << "is Zero bytes, is it correct?" << std::endl;
    err << "\t!!! Last Line Executed: " << l << std::endl;
    throw std::runtime_error(err.str().c_str());
  }
}

template <AccessMode M>
inline ipc_t<M>::~ipc_t()
{
  if ((M == AccessMode::CREATE_AND_SYNC_WRITE) || (M == AccessMode::CREATE_AND_SYNC_READ))
  {
    if (!boost::interprocess::shared_memory_object::remove(name_.c_str()))
    {
      std::cerr << RED() << __PRETTY_FUNCTION__ << ": ShMem Destructor" << name_.c_str()
                << " Error in removing the shmem" << RESET() << std::endl;
    }

    if (!boost::interprocess::named_mutex::remove(name_.c_str()))
    {
      std::cerr << RED() << __PRETTY_FUNCTION__ << ": ShMem Destructor" << name_.c_str()
                << " Error in removing the mutex" << RESET() << std::endl;
    }
  }
}

template <AccessMode M>
inline std::size_t ipc_t<M>::dim() const
{
  return shared_map_.get_size();
}

template <AccessMode M>
inline void ipc_t<M>::get(void* shmem)
{
  boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
  std::memcpy(shmem, shared_map_.get_address(), shared_map_.get_size());
  lock.unlock();
}

template <AccessMode M>
inline void ipc_t<M>::set(const void* shmem)
{
  boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
  std::memcpy(shared_map_.get_address(), shmem, shared_map_.get_size());
  lock.unlock();
}

// ==========================================================================
//
//
//
//
// ==========================================================================
template <AccessMode M>
template <AccessMode O, typename std::enable_if<(O == AccessMode::CREATE_AND_SYNC_WRITE)>::type*>
inline rt_ipc_t<M>::rt_ipc_t(const std::size_t& dim,
                                                 const std::string& name,
                                                 const double& watchdog_s)
    : name_(name),
      dim_buffer_(dim),
      data_accessor_(dim + sizeof(rt_data_t::Header), name_),
      watchdog_(watchdog_s),
      bonded_prev_(false),
      is_hard_rt_prev_(false)
{
}

template <AccessMode M>
template <AccessMode O, typename std::enable_if<(O == AccessMode::OPEN_AND_SYNC_WRITE)>::type*>
inline rt_ipc_t<M>::rt_ipc_t(const std::string& name, const double& watchdog_s)
    : name_(name), data_accessor_(name_), watchdog_(watchdog_s), bonded_prev_(false), is_hard_rt_prev_(false)
{
  dim_buffer_ = data_accessor_.dim() - sizeof(rt_data_t::Header);
  if (data_accessor_.dim() < sizeof(rt_data_t::Header))
    throw std::runtime_error((__PRETTY_FUNCTION__ + ("In processing module '" + name_ +
                                                     "': the mapped shared memory dimension is wrongly low"))
                                 .c_str());
}

template <AccessMode M>
template <AccessMode O, typename std::enable_if<(O == AccessMode::CREATE_AND_SYNC_READ)>::type*>
inline rt_ipc_t<M>::rt_ipc_t(const std::size_t& dim,
                                                 const std::string& name,
                                                 const double& watchdog_s)
    : name_(name),
      dim_buffer_(dim),
      data_accessor_(dim + sizeof(rt_data_t::Header), name_),
      watchdog_(watchdog_s),
      bonded_prev_(false),
      is_hard_rt_prev_(false)
{
}

template <AccessMode M>
template <AccessMode O, typename std::enable_if<(O == AccessMode::OPEN_AND_SYNC_READ)>::type*>
inline rt_ipc_t<M>::rt_ipc_t(const std::string& name, const double& watchdog_s)
    : name_(name), data_accessor_(name_), watchdog_(watchdog_s), bonded_prev_(false), is_hard_rt_prev_(false)
{
  dim_buffer_ = data_accessor_.dim() - sizeof(rt_data_t::Header);
  if (data_accessor_.dim() < sizeof(rt_data_t::Header))
    throw std::runtime_error((__PRETTY_FUNCTION__ + ("In processing module '" + name_ +
                                                     "': the mapped shared memory dimension is wrongly low"))
                                 .c_str());
}

template <AccessMode M>
inline rt_ipc_t<M>::~rt_ipc_t()
{
  if (is_bonded())
  {
    break_bond();
  }
}

template <AccessMode M>
inline bool rt_ipc_t<M>::is_hard_rt()
{
  data_accessor_.get(&last_data_available_.data_);
  return (data_accessor_.dim() == 0) ? false : (last_data_available_.data_.header_.rt_flag_ == 1);
}

template <AccessMode M>
inline bool rt_ipc_t<M>::set_rt(std::string& warning, uint8_t hard)
{
  data_accessor_.get(&last_data_available_.data_);
  last_data_available_.update_time_ = now_s();
  if (last_data_available_.data_.header_.bond_flag_ == 0)
  {
    warning = "[" + BOLDCYAN() + name_ + RESET() + "] " + RED() + "Not bonded! It is not possible to set RT";
    return false;
  }

  if (last_data_available_.data_.header_.rt_flag_ == hard)
  {
    warning =
        "[" + BOLDCYAN() + name_ + RESET() + "] " + RED() + "Already " + std::string(hard ? "Hard" : "soft") + " RT!";
  }

  last_data_available_.data_.header_.rt_flag_ = 1;
  last_data_available_.data_.header_.time_ = now_s();
  data_accessor_.set(&(last_data_available_.data_));

  return true;
}

template <AccessMode M>
inline bool rt_ipc_t<M>::set_hard_rt(std::string& warning)
{
  return this->set_rt(warning, 1);
}

template <AccessMode M>
inline bool rt_ipc_t<M>::set_soft_rt(std::string& warning)
{
  return this->set_rt(warning, 0);
}

template <AccessMode M>
inline bool rt_ipc_t<M>::is_bonded()
{
  data_accessor_.get(&last_data_available_.data_);
  return (data_accessor_.dim() == 0) ? false : (last_data_available_.data_.header_.bond_flag_ == 1);
}

template <AccessMode M>
bool rt_ipc_t<M>::bond(std::string& warning)
{
  data_accessor_.get(&last_data_available_.data_);
  last_data_available_.update_time_ = now_s();

  if (last_data_available_.data_.header_.bond_flag_ == 1)
  {
    warning = "[" + BOLDCYAN() + name_ + RESET() + "] " + RED() + "Already Bonded!";
    return false;
  }

  last_data_available_.data_.header_.bond_flag_ = 1;
  data_accessor_.set(&last_data_available_.data_);

  return true;
}

template <AccessMode M>
inline bool rt_ipc_t<M>::break_bond()
{
  data_accessor_.get(&last_data_available_.data_);
  last_data_available_.update_time_ = now_s();

  last_data_available_.data_.header_.rt_flag_ = 0;
  last_data_available_.data_.header_.bond_flag_ = 0;

  data_accessor_.set(&last_data_available_.data_);

  return true;
}

template <AccessMode M>
template <
    AccessMode O,
    typename std::enable_if<(O == AccessMode::CREATE_AND_SYNC_WRITE) || (O == AccessMode::OPEN_AND_SYNC_WRITE)>::type*>
inline ErrorCode rt_ipc_t<M>::write(const uint8_t* idata_buffer,
                                              const uint8_t* safety_buffer,
                                              const double& idata_time_label,
                                              const std::size_t& n_bytes,
                                              std::string& warning)
{
  warning = __PRETTY_FUNCTION__ + std::string(":\n");
  if (dim_buffer_ != n_bytes)
  {
    warning += "\t\t!!!Error: " + to_string(ErrorCode::ERROR_UNMACTHED_DATA_DIMENSION) + "\n";
    warning += "\t\t!!!Expected n. bytes: " + std::to_string(dim_buffer_) + "\n";
    warning += "\t\t!!!Input n. bytes: " + std::to_string(n_bytes) + "\n";
    return ErrorCode::ERROR_UNMACTHED_DATA_DIMENSION;
  }

  if (dim_buffer_ <= 0)
  {
    return ErrorCode::OK;
  }

  const uint8_t _zero = 0;
  const uint8_t is_hard_rt_before = this->is_hard_rt();
  const uint8_t is_bonded_before = this->is_bonded();

  rt_data_t idata;
  copy(&idata, idata_buffer, dim_buffer_, &idata_time_label, &is_bonded_before, &is_hard_rt_before);

  rt_data_t sdata;
  copy(&sdata, safety_buffer, dim_buffer_, &idata_time_label, &_zero, &_zero);

  // =======================================
  if (!watchdog_.toc())
  {
    watchdog_.tic();
    if (is_hard_rt_before)
    {
      data_accessor_.set(cast_and_set_time(&sdata));
      warning += "\t\t!!!Error: " + to_string(ErrorCode::ERROR_WRITING_WATCHDOG) + "\n";
      warning += "\t\t!!!Watchdog:\t" + std::to_string(watchdog_.dt());
      return ErrorCode::ERROR_WRITING_WATCHDOG;
    }
  }

  // =======================================

  double time_misalignment = std::fabs(watchdog_.now() - idata_time_label);
  if (time_misalignment > watchdog_.threshold())
  {
    data_accessor_.set(cast_and_set_time(&sdata));
    warning += "\t\t!!!Error: " + to_string(ErrorCode::ERROR_UNMATCHED_TIME) + "\n";
    warning += "\t\t!!!Actual write time:\t" + std::to_string(watchdog_.now()) + "\n";
    warning += "\t\t!!!Label ipc time:\t" + std::to_string(idata_time_label);
    return ErrorCode::ERROR_UNMATCHED_TIME;
  }

  last_data_available_.update_time_ = now_s();
  if (is_bonded_before)
  {
    copy(&last_data_available_.data_, &idata);
    data_accessor_.set(&(last_data_available_.data_));
  }
  else
  {
    copy(&last_data_available_.data_, safety_buffer, dim_buffer_, &idata_time_label, &_zero, &_zero);
  }

  // ===============================================
  const bool is_hard_rt_after = this->is_hard_rt();
  const bool is_bonded_after = this->is_bonded();
  if (is_hard_rt_prev_ != is_hard_rt_after)
  {
    warning = "[" + BOLDCYAN() + name_ + RESET() + "] RT State Changed from '" + BOLDCYAN() +
              (is_hard_rt_prev_ ? "HARD" : "SOFT") + RESET() + "' to '" + (is_hard_rt_after ? "HARD" : "SOFT") + "'";
    is_hard_rt_prev_ = is_hard_rt_after;
  }
  if (bonded_prev_ != is_bonded_after)
  {
    warning = "[" + BOLDCYAN() + name_ + RESET() + "] Bonding State Changed from '" + BOLDCYAN() +
              (bonded_prev_ ? "BONDED" : "UNBONDED") + RESET() + "' to '" + BOLDCYAN() +
              (is_bonded_after ? "BONDED" : "UNBONDED") + RESET() + "'";
    bonded_prev_ = is_bonded_after;
  }
  // ===============================================

  warning = "";
  return ErrorCode::OK;
}

template <AccessMode M>
template <
    AccessMode O,
    typename std::enable_if<(O == AccessMode::OPEN_AND_SYNC_READ) || (O == AccessMode::CREATE_AND_SYNC_READ)>::type*>
inline ErrorCode rt_ipc_t<M>::read(uint8_t* odata_buffer,
                                             double* odata_time_label,
                                             double* odata_time_latency,
                                             const uint8_t* safety_buffer,
                                             const std::size_t& n_bytes,
                                             std::string& warning)
{
  warning = __PRETTY_FUNCTION__ + std::string(":\n");
  if (dim_buffer_ != n_bytes)
  {
    warning += "\t\t!!!Error: " + to_string(ErrorCode::ERROR_UNMACTHED_DATA_DIMENSION) + "\n";
    warning += "\t\t!!!Expected n. bytes: " + std::to_string(dim_buffer_) + "\n";
    warning += "\t\t!!!Input n. bytes: " + std::to_string(n_bytes) + "\n";
    return ErrorCode::ERROR_UNMACTHED_DATA_DIMENSION;
  }

  if (dim_buffer_ <= 0)
  {  
    return ErrorCode::OK;
  }

  // =======================================
  if(last_data_available_.data_.header_.time_ == 0.0)
  {
    last_data_available_.data_.header_.time_ = now_s();
  }

  double data_time_prev = last_data_available_.data_.header_.time_;
  
  // =======================================
  last_data_available_.update_time_ = now_s();
  data_accessor_.get(&last_data_available_.data_);

  // =======================================
  *odata_time_label = last_data_available_.data_.header_.time_;

  *odata_time_latency = (*odata_time_label - data_time_prev);

  const double time_misalignment =
      std::fabs(last_data_available_.update_time_ - last_data_available_.data_.header_.time_);
  const bool is_hard_rt = this->is_hard_rt();
  const bool is_bonded = this->is_bonded();

  // =======================================
  if (is_bonded && is_hard_rt)
  {
    if (!watchdog_.toc())
    {
      watchdog_.tic();
      if (is_hard_rt)
      {
        warning += "\t\t!!!Error: " + to_string(ErrorCode::ERROR_WRITING_WATCHDOG) + "\n";
        warning += "\t\t!!!Watchdog:\t" + std::to_string(watchdog_.dt()); 
        std::memcpy(odata_buffer, safety_buffer, dim_buffer_);
        return ErrorCode::ERROR_READING_WATCHDOG;
      }
    }

    // =======================================
    if (time_misalignment > watchdog_.threshold())
    {
      warning += "\t\t!!!Error: " + to_string(ErrorCode::ERROR_UNMATCHED_TIME) + "\n";
      warning += "\t\t!!!Actual read time:\t" + std::to_string(last_data_available_.update_time_) + "\n";
      warning += "\t\t!!!Label ipc time:\t" + std::to_string(*odata_time_label);
      std::memcpy(odata_buffer, safety_buffer, dim_buffer_);
      return ErrorCode::ERROR_UNMATCHED_TIME;
    }

    // =======================================
    if (*odata_time_latency > watchdog_.threshold())
    {
      warning += "\t\t!!!Error: " + to_string(ErrorCode::ERROR_READING_WATCHDOG) + "\n";
      warning += "\t\t!!!Time latency:\t" + std::to_string(*odata_time_latency );
      std::memcpy(odata_buffer, safety_buffer, dim_buffer_);
      return ErrorCode::ERROR_READING_WATCHDOG;
    }
  }

  // =======================================
  if(is_bonded)
  {
    std::memcpy(odata_buffer, static_cast<const void*>(last_data_available_.data_.buffer_), dim_buffer_);
  }
  else
  {
    std::memcpy(odata_buffer, static_cast<const void*>(safety_buffer), dim_buffer_);

    warning += "\t\t!!!Warning: " + to_string(ErrorCode::WARNING_NOT_BONDED);
    return ErrorCode::WARNING_NOT_BONDED;
  }

  // ===============================================
  if (is_hard_rt_prev_ != is_hard_rt)
  {
    warning = "[" + BOLDCYAN() + name_ + RESET() + "] RT State Changed from '" + BOLDCYAN() +
              (is_hard_rt_prev_ ? "HARD" : "SOFT") + RESET() + "' to '" + (is_hard_rt ? "HARD" : "SOFT") + "'";
    is_hard_rt_prev_ = is_hard_rt;
  }
  if (bonded_prev_ != is_bonded)
  {
    warning = "[" + BOLDCYAN() + name_ + RESET() + "] " + RED() + " Bonding State Changed from '" + BOLDCYAN() +
              (bonded_prev_ ? "BONDED" : "UNBONDED") + RESET() + "' to '" + BOLDCYAN() +
              (is_bonded ? "BONDED" : "UNBONDED") + RESET() + "'";
    bonded_prev_ = is_bonded;
  }
  // ===============================================

  warning = "";
  return ErrorCode::OK;
}

inline std::string to_string(ErrorCode err)
{
  std::string ret = "na";
  switch (err)
  {
    case ErrorCode::OK:
      ret = "SHARED MEMORY NONE ERROR";
      break;
    case ErrorCode::ERROR_UNMACTHED_DATA_DIMENSION:
      ret = "SHARED MEMORY UNMATHCED DATA DIMENSION";
      break;
    case ErrorCode::ERROR_UNCORRECT_CALL:
      ret = "SHARED MEMORY UNCORRECT CALL SEQUENCE";
      break;
    case ErrorCode::ERROR_WRITING_WATCHDOG:
      ret = "SHARED MEMORY WRITING WATCHDOG";
      break;
    case ErrorCode::ERROR_READING_WATCHDOG:
      ret = "SHARED MEMORY READING WATCHDOG";
      break;
    case ErrorCode::ERROR_UNMATCHED_TIME:
      ret = "SHARED MEMORY UNMATCHED TIME";
      break;
    case ErrorCode::WARNING_NOT_BONDED:
      ret = "NOT BONDED";
      break;
  }
  return ret;
}

inline int as_integer(ErrorCode const value)
{
  return static_cast<typename std::underlying_type<ErrorCode>::type>(value);
}

template <AccessMode M>
inline std::size_t rt_ipc_t<M>::dim(bool prepend_header) const
{
  return prepend_header ? dim_buffer_ + sizeof(rt_data_t::Header) : dim_buffer_;
}

}  // namespace ipc
}  // namespace cnr

#endif  /* CNR_IPC_UTILITIES_INCLUDE_CNR_IPC_UTILITIES_IMPL_SHMEM_IPC_IMPL */
