/*
 * Copyright (c) 2017-2018 Group of Computer Architecture, University of Bremen <riscv@systemc-verification.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef RISCV_GDB_NG
#define RISCV_GDB_NG

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <queue>
#include <mutex>
#include <systemc>
#include <thread>
#include <map>
#include <tuple>
#include <functional>

#include <core/common/mmu_mem_if.h>
#include <libgdb/parser2.h>

#include "debug.h"
#include "core_defs.h"
#include "platform/common/async_event.h"
#include "debug_memory.h" // DebugMemoryInterface

SC_MODULE(GDBServer) {
public:
	typedef void (GDBServer::*packet_handler)(int, gdb_command_t *);

	void haltReason(int, gdb_command_t *);
	void getRegisters(int, gdb_command_t *);
	void setThread(int, gdb_command_t *);
	void killServer(int, gdb_command_t *);
	void readMemory(int, gdb_command_t *);
	void writeMemory(int, gdb_command_t *);
	void readRegister(int, gdb_command_t *);
	void qAttached(int, gdb_command_t *);
	void qSupported(int, gdb_command_t *);
	void threadInfo(int, gdb_command_t *);
	void threadInfoEnd(int, gdb_command_t *);
	void vCont(int, gdb_command_t *);
	void vContSupported(int, gdb_command_t *);
	void removeBreakpoint(int, gdb_command_t *);
	void setBreakpoint(int, gdb_command_t *);
	void isAlive(int, gdb_command_t *);

	SC_HAS_PROCESS(GDBServer);

	GDBServer(sc_core::sc_module_name,
	          std::vector<debug_target_if*>,
	          DebugMemoryInterface*,
	          uint16_t,
	          std::vector<mmu_memory_if*> mmus = {});

	/* Used by GDBRunner to determine whether run() or run_step()
	 * should be used when receiving a run event for a debug_target_if.
	 *
	 * TODO: Pass this on a per-event basis instead. */
	bool single_run = false;

	sc_core::sc_event *get_stop_event(debug_target_if *);
	void set_run_event(debug_target_if *, sc_core::sc_event *);

private:
	typedef std::function<void(debug_target_if *)> thread_func;
	typedef std::tuple<int, gdb_packet_t *> ctx;
	typedef std::tuple<sc_core::sc_event *, sc_core::sc_event *> hart_event;

	DebugMemoryInterface *memory;
	AsyncEvent asyncEvent;
	Architecture arch;
	std::vector<debug_target_if*> harts;
	std::thread thr;
	char *prevpkt;
	std::queue<ctx> pktq;
	std::mutex mtx;
	int sockfd;

	/* operation → thread id */
	std::map<char, int> thread_ops;

	/* hart → events */
	std::map<debug_target_if *, hart_event> events;

	/* hart → mmu */
	std::map<debug_target_if *, mmu_memory_if *> mmu;

	void create_sock(uint16_t);
	std::vector<debug_target_if *> get_threads(int);
	uint64_t translate_addr(debug_target_if *, uint64_t, MemoryAccessType type);
	void exec_thread(thread_func, char = 'g');
	std::vector<debug_target_if *> run_threads(std::vector<debug_target_if *>, bool = false);
	void writeall(int, char *, size_t);
	void send_packet(int, const char *, gdb_kind_t = GDB_KIND_PACKET);
	void retransmit(int);
	void dispatch(int conn);
	void serve(void);
	void run(void);
};

#endif
