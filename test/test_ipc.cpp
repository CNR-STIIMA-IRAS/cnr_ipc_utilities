/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <map>
#include <thread>

#include <chrono>
#include <iostream>
#include <map>

#include <cnr_ipc_utilities/shmem_ipc.h>

namespace detail
{
struct unwrapper
{
  explicit unwrapper(std::exception_ptr pe) : pe_(pe) {}

  operator bool() const { return bool(pe_); }

  friend auto operator<<(std::ostream& os, unwrapper const& u) -> std::ostream&
  {
    try
    {
      std::rethrow_exception(u.pe_);
      return os << "no exception";
    }
    catch (std::runtime_error const& e)
    {
      return os << "runtime_error: " << e.what();
    }
    catch (std::logic_error const& e)
    {
      return os << "logic_error: " << e.what();
    }
    catch (std::exception const& e)
    {
      return os << "exception: " << e.what();
    }
    catch (...)
    {
      return os << "non-standard exception";
    }
  }
  std::exception_ptr pe_;
};
}  // namespace detail

auto unwrap(std::exception_ptr pe) { return detail::unwrapper(pe); }

template <class F>
::testing::AssertionResult does_not_throw(F&& f)
{
  try
  {
    f();
    return ::testing::AssertionSuccess();
  }
  catch (...)
  {
    return ::testing::AssertionFailure() << unwrap(std::current_exception());
  }
}

template <class F>
bool time_measure(std::size_t id, std::string& what, const cnr::ipc::ErrorCode& code, F&& f)
{
  what = "";
  struct timespec start, end;
  clock_gettime(CLOCK_MONOTONIC, &start);
  cnr::ipc::ErrorCode ret = f();
  clock_gettime(CLOCK_MONOTONIC, &end);
  double time_taken;
  time_taken = double(end.tv_sec - start.tv_sec) * 1e9;
  time_taken = double(time_taken + (end.tv_nsec - start.tv_nsec)) * 1e-9;
  std::cout << "============ [" << id << "] Exec Time [ms]: " << time_taken * 1000 << std::endl;
  if (ret != code)
  {
    std::cout << what << std::endl;
  }
  return code == ret;
}

template <class F>
bool time_measure_ok(std::size_t id, std::string& what, const bool& val, F&& f)
{
  what = "";
  struct timespec start, end;
  clock_gettime(CLOCK_MONOTONIC, &start);
  bool ret = f();
  clock_gettime(CLOCK_MONOTONIC, &end);
  double time_taken;
  time_taken = double(end.tv_sec - start.tv_sec) * 1e9;
  time_taken = double(time_taken + (end.tv_nsec - start.tv_nsec)) * 1e-9;
  std::cout << "============ [" << id << "] Exec Time [ms]: " << time_taken * 1000 << std::endl;
  if (ret != val)
  {
    std::cout << what << std::endl;
  }
  return val == ret;
}

const std::size_t n_bytes = 24;
const double watchdog = 0.01;

std::shared_ptr<cnr::ipc::rt_data_accessor_t<cnr::ipc::AccessMode::CREATE_AND_SYNC_WRITE> > rt_data_cw;
std::shared_ptr<cnr::ipc::rt_data_accessor_t<cnr::ipc::AccessMode::CREATE_AND_SYNC_READ> > rt_data_cr;
std::shared_ptr<cnr::ipc::rt_data_accessor_t<cnr::ipc::AccessMode::OPEN_AND_SYNC_WRITE> > rt_data_ow;
std::shared_ptr<cnr::ipc::rt_data_accessor_t<cnr::ipc::AccessMode::OPEN_AND_SYNC_READ> > rt_data_or;

TEST(TestSuite, createWriter)
{
  EXPECT_TRUE(does_not_throw(
      [&]
      {
        rt_data_cw.reset(
            new cnr::ipc::rt_data_accessor_t<cnr::ipc::AccessMode::CREATE_AND_SYNC_WRITE>(n_bytes, "AAA", watchdog));
      }));
}

TEST(TestSuite, createReader)
{
  EXPECT_TRUE(does_not_throw(
      [&]
      {
        rt_data_cr.reset(
            new cnr::ipc::rt_data_accessor_t<cnr::ipc::AccessMode::CREATE_AND_SYNC_READ>(n_bytes, "BBB", watchdog));
      }));
}

TEST(TestSuite, openWriter)
{
  EXPECT_TRUE(does_not_throw(
      [&] {
        rt_data_ow.reset(new cnr::ipc::rt_data_accessor_t<cnr::ipc::AccessMode::OPEN_AND_SYNC_WRITE>("BBB", watchdog));
      }));
}

TEST(TestSuite, openReader)
{
  EXPECT_TRUE(does_not_throw(
      [&] {
        rt_data_or.reset(new cnr::ipc::rt_data_accessor_t<cnr::ipc::AccessMode::OPEN_AND_SYNC_READ>("AAA", watchdog));
      }));
}

TEST(TestSuite, writeWriter)
{
  uint8_t idata_buffer[n_bytes] = {0};
  uint8_t safety_buffer[n_bytes] = {1};
  double time = cnr::ipc::now_s();
  std::string what;
  cnr::ipc::ErrorCode ret;
  EXPECT_TRUE(cnr::ipc::ErrorCode::OK == (ret = rt_data_cw->write(idata_buffer, safety_buffer, time, n_bytes, what)));
  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::ERROR_UNMACTHED_DATA_DIMENSION,
                           [&]() -> cnr::ipc::ErrorCode
                           { return rt_data_cw->write(idata_buffer, safety_buffer, time, 2 * n_bytes, what); }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 2000)));
  // The time that I superimpose, is different from now()
  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::ERROR_UNMATCHED_TIME,
                           [&]() -> cnr::ipc::ErrorCode
                           { return rt_data_cw->write(idata_buffer, safety_buffer, time, n_bytes, what); }));
}

TEST(TestSuite, writeOpener)
{
  uint8_t idata_buffer[n_bytes] = {0};
  uint8_t safety_buffer[n_bytes] = {1};
  double time = cnr::ipc::now_s();
  std::string what;

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode
                           { return rt_data_ow->write(idata_buffer, safety_buffer, time, n_bytes, what); }));

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::ERROR_UNMACTHED_DATA_DIMENSION,
                           [&]() -> cnr::ipc::ErrorCode
                           { return rt_data_ow->write(idata_buffer, safety_buffer, time, 2 * n_bytes, what); }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 2000)));
  // The time that I superimpose, is different from now()
  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::ERROR_UNMATCHED_TIME,
                           [&]() -> cnr::ipc::ErrorCode
                           { return rt_data_ow->write(idata_buffer, safety_buffer, time, n_bytes, what); }));
}

TEST(TestSuite, readWriteWriter)
{
  uint8_t odata_buffer[n_bytes] = {0};
  uint8_t idata_buffer[n_bytes] = {0};
  uint8_t safety_buffer[n_bytes] = {1};
  double time = cnr::ipc::now_s();
  double otime;
  double otime_latency;

  std::string what;
  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode
                           { return rt_data_cw->write(idata_buffer, safety_buffer, time, n_bytes, what); }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 1000)));
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::WARNING_NOT_BONDED,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_or->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_or->bond(what); }));

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode {
                             return rt_data_cw->write(idata_buffer, safety_buffer, cnr::ipc::now_s(), n_bytes, what);
                           }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 1000)));
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::OK,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_or->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  // only bonded, not RT, therefore no error despite the watchdog is broken
  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 1000)));  //
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::OK,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_or->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_or->set_hard_rt(what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_or->is_bonded(); }));

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode {
                             return rt_data_cw->write(idata_buffer, safety_buffer, cnr::ipc::now_s(), n_bytes, what);
                           }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 500)));
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::OK,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_or->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode {
                             return rt_data_cw->write(idata_buffer, safety_buffer, cnr::ipc::now_s(), n_bytes, what);
                           }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 2500)));
  EXPECT_FALSE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::OK,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_or->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_or->is_hard_rt(); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_or->break_bond(); }));

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode {
                             return rt_data_cw->write(idata_buffer, safety_buffer, cnr::ipc::now_s(), n_bytes, what);
                           }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 2500)));
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::WARNING_NOT_BONDED,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_or->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, false, [&]() -> bool { return rt_data_or->is_hard_rt(); }));
}


TEST(TestSuite, readWriteReader)
{
  uint8_t odata_buffer[n_bytes] = {0};
  uint8_t idata_buffer[n_bytes] = {0};
  uint8_t safety_buffer[n_bytes] = {1};
  double time = cnr::ipc::now_s();
  double otime;
  double otime_latency;

  std::string what;
  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode
                           { return rt_data_ow->write(idata_buffer, safety_buffer, time, n_bytes, what); }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 1000)));
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::WARNING_NOT_BONDED,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_cr->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_cr->bond(what); }));

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode {
                             return rt_data_ow->write(idata_buffer, safety_buffer, cnr::ipc::now_s(), n_bytes, what);
                           }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 1000)));
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::OK,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_cr->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  // only bonded, not RT, therefore no error despite the watchdog is broken
  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 1000)));  //
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::OK,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_cr->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_cr->set_hard_rt(what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_cr->is_bonded(); }));

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode {
                             return rt_data_ow->write(idata_buffer, safety_buffer, cnr::ipc::now_s(), n_bytes, what);
                           }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 500)));
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::OK,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_cr->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode {
                             return rt_data_ow->write(idata_buffer, safety_buffer, cnr::ipc::now_s(), n_bytes, what);
                           }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 2500)));
  EXPECT_FALSE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::OK,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_cr->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_cr->is_hard_rt(); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, true, [&]() -> bool { return rt_data_cr->break_bond(); }));

  EXPECT_TRUE(time_measure(__LINE__,
                           what,
                           cnr::ipc::ErrorCode::OK,
                           [&]() -> cnr::ipc::ErrorCode {
                             return rt_data_ow->write(idata_buffer, safety_buffer, cnr::ipc::now_s(), n_bytes, what);
                           }));

  std::this_thread::sleep_for(std::chrono::milliseconds(int(watchdog * 2500)));
  EXPECT_TRUE(
      time_measure(__LINE__,
                   what,
                   cnr::ipc::ErrorCode::WARNING_NOT_BONDED,
                   [&]() -> cnr::ipc::ErrorCode
                   { return rt_data_cr->read(odata_buffer, &otime, &otime_latency, safety_buffer, n_bytes, what); }));

  EXPECT_TRUE(time_measure_ok(__LINE__, what, false, [&]() -> bool { return rt_data_cr->is_hard_rt(); }));
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
