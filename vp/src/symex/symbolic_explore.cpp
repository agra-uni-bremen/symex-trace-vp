/*
 * Copyright (c) 2020,2021 Group of Computer Architecture, University of Bremen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef RISCV_ISA_EXPLORATION_H
#define RISCV_ISA_EXPLORATION_H

#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

/* Debug leaks with valgrind --leak-check=full --undef-value-errors=no
 * Also: Define valgrind here to prevent spurious Z3 memory leaks. */
#ifdef VALGRIND
#include <z3.h>
#endif

#include <clover/query_info.hpp>
// std::max_element / std::accumulate were previously only reaching this file transitively through
// the klee headers pulled in by query_info.hpp. Included explicitly so the branch-info aggregation
// below does not depend on that.
#include <algorithm>
#include <numeric>
#include <iostream>
#include <systemc>
#include <filesystem>
#include <systemc>

#include "symbolic_explore.h"
#include "symbolic_context.h"

#include "rawmode.h"

#define TESTCASE_ENV "SYMEX_TESTCASE"
#define TIMEBUDGET_ENV "SYMEX_TIMEBUDGET"
#define ERR_EXIT_ENV "SYMEX_ERREXIT"

static std::filesystem::path *testcase_path = nullptr;
static size_t errors_found = 0;
static size_t paths_found = 0;

static std::chrono::duration<double, std::milli> solver_time;

static void
dump_stats(void)
{
	auto stime = std::chrono::duration_cast<std::chrono::seconds>(solver_time);

	std::cout << std::endl << "---" << std::endl;
	std::cout << "Unique paths found: " << paths_found << std::endl;
	std::cout << "Solver Time: " << stime.count() << " seconds" << std::endl;
	// TODO: Also dump instruction branch coverage here.
	if (errors_found > 0) {
		std::cout << "Errors found: " << errors_found << std::endl;
		std::cout << "Testcase directory: " << *testcase_path << std::endl;
	}
}

static std::optional<std::string>
dump_input(std::string fn)
{
	clover::ExecutionContext &ctx = symbolic_context.ctx;
	clover::ConcreteStore store = ctx.getPrevStore();
	if (store.empty())
		return std::nullopt; // Execution does not depend on symbolic values

	assert(testcase_path);
	auto path = *testcase_path / fn;

	std::ofstream file(path);
	if (!file.is_open())
		throw std::runtime_error("failed to open " + path.string());

	clover::TestCase::toFile(store, file);
	return path;
}

static void
report_handler(const sc_core::sc_report& report, const sc_core::sc_actions& actions)
{
	auto nactions = actions;
	auto mtype = report.get_msg_type();

	if (!strcmp(mtype, "/AGRA/riscv-vp/host-error") || !testcase_path) {
		auto path = dump_input("error" + std::to_string(++errors_found));
		if (!path.has_value())
			return;

		std::cerr << "Found error, use " << *path << " to reproduce." << std::endl;
		if (getenv(ERR_EXIT_ENV)) {
			std::cerr << "Exit on first error set, terminating..." << std::endl;
			exit(EXIT_FAILURE);
		}

		nactions &= ~sc_core::SC_DISPLAY; // Prevent SystemC output
		nactions &= sc_core::SC_STOP;     // Stop SystemC simulation
	}

	// Invoke default handler, even for host-error, to ensure that
	// SC_REPORT_ERROR is handled properly (i.e. execution is stopped).
	sc_core::sc_report_handler::default_handler(report, nactions);
}

static void
sigalrm_handler(int signum)
{
	(void)signum;

	std::cout << "Time budget exceeded, terminating..." << std::endl;
	dump_stats();

	disableRawMode(STDIN_FILENO); // _Exit doesn't run atexit functions
	std::_Exit(EXIT_SUCCESS);     // Don't run deconstructors
}

static void
remove_testdir(void)
{
	assert(testcase_path != nullptr);
	if (errors_found > 0)
		return;

	// Remove test directory if no errors were found
	if (rmdir(testcase_path->c_str()) == -1)
		throw std::system_error(errno, std::generic_category());

	delete testcase_path;
	testcase_path = nullptr;
}

static void
create_testdir(void)
{
	char *dirpath;
	char tmpl[] = "/tmp/clover_testsXXXXXX";

	if (!(dirpath = mkdtemp(tmpl)))
		throw std::system_error(errno, std::generic_category());
	testcase_path = new std::filesystem::path(dirpath);

	if (std::atexit(remove_testdir))
		throw std::runtime_error("std::atexit failed");
}

static bool
setupNewValues(clover::ExecutionContext &ctx, clover::Trace &tracer)
{
	auto start = std::chrono::steady_clock::now();
	auto r = ctx.setupNewValues(tracer);
	auto end = std::chrono::steady_clock::now();

	solver_time += end - start;
	return r;
}

static int
run_test(const char *path, int argc, char **argv)
{
	std::string fp(path);
	std::ifstream file(fp);
	if (!file.is_open())
		throw std::runtime_error("failed to open " + fp);

	clover::ExecutionContext &ctx = symbolic_context.ctx;
	clover::ConcreteStore store = clover::TestCase::fromFile(fp, file);

	ctx.setupNewValues(store);
	return sc_core::sc_elab_and_sim(argc, argv);
}

static int
explore_paths(int argc, char **argv)
{
	clover::ExecutionContext &ctx = symbolic_context.ctx;
	clover::Trace &tracer = symbolic_context.trace;

	bool new_path_exists = false;
	do {
		std::cout << std::endl << "################################" << std::endl << std::endl;
		std::cout << "<symex run_id=\"" << symolic_run_id << "\">" << std::endl;
		//std::cout << std::endl << "##" << std::endl << "# "
		//	<< paths_found + 1 << "th concolic execution" << std::endl
		//	<< "##" << std::endl;

		tracer.reset();

		// Reset SystemC simulation context
		// See also: https://github.com/accellera-official/systemc/issues/8
		if (sc_core::sc_curr_simcontext) {
			sc_core::sc_report_handler::release();
			delete sc_core::sc_curr_simcontext;
		}
		sc_core::sc_curr_simcontext = NULL;

		++paths_found;
		int ret;
		if ((ret = sc_core::sc_elab_and_sim(argc, argv)))
			return ret;
		new_path_exists = setupNewValues(ctx, tracer);
		std::cout << "</symex>" << std::endl;
		++symolic_run_id;
		runs_created_by_current_run = 0;
	} while (new_path_exists);

	sc_core::sc_report_handler::release();
	delete sc_core::sc_curr_simcontext;

	return 0;
}

static void
setup_timeout(void)
{
	struct sigaction sa;

	char *timebudget = getenv(TIMEBUDGET_ENV);
	if (!timebudget)
		return;

	errno = 0;
	auto seconds = strtoul(timebudget, NULL, 10);
	if (!seconds && errno)
		throw std::system_error(errno, std::generic_category());

	sa.sa_flags = SA_RESTART;
	sa.sa_handler = sigalrm_handler;
	if (sigemptyset(&sa.sa_mask) == -1)
		throw std::system_error(errno, std::generic_category());
	if (sigaction(SIGALRM, &sa, NULL) == -1)
		throw std::system_error(errno, std::generic_category());

	int r = alarm(seconds);
	assert(r == 0);
	(void)r;
}

int
symbolic_explore(int argc, char **argv)
{
	std::cout << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
	std::cout << "<trace version=\"4.0\">" << std::endl;
	// Hide SystemC copyright message
	setenv("SYSTEMC_DISABLE_COPYRIGHT_MESSAGE", "1", 0);

	// Mempool does not seem to free all memory, disable it.
	setenv("SYSTEMC_MEMPOOL_DONT_USE", "1", 0);

	// Use current time as seed for random generator
	std::srand(std::time(nullptr));

	char *testcase = getenv(TESTCASE_ENV);
	if (testcase)
		return run_test(testcase, argc, argv);
	create_testdir();

	// Set report handler for detecting errors
	sc_core::sc_report_handler::set_handler(report_handler);

	setup_timeout();
	int ret = explore_paths(argc, argv);
	dump_stats();

#ifdef VALGRIND
	Z3_finalize_memory();
#endif

	//std::cout << "<result>" << std::endl;
	//std::cout << "</result>" << std::endl;
	//std::cout << "<test-results>" << std::endl;
	//std::cout << "</test-results>" << std::endl;

	std::cout << "<timelines>" << std::endl;
	for (uint32_t run = 0; run < paths_found; run++)
	{
		std::cout << "<run id=\"" << run << "\" >"<< std::endl;
		for (size_t i = 0; i < 30; i++)//TODO probably use a dynamic list
		{
			uint32_t b_pc = symbolic_run_links[run][i].pc;
			uint64_t b_step = symbolic_run_links[run][i].step;
			uint64_t b_hash = symbolic_run_links[run][i].hash;
			if(b_pc<=0 || b_pc > 1000000000){
				break;
			}
			std::cout << std::hex << "<branch pc=\"";
			std::cout << b_pc << std::dec;
			std::cout << "\" step=\"";
			std::cout << b_step; 
			std::cout << "\" hash=\"";
			std::cout << b_hash << "\"></branch>" << std::endl;
		}
		
		std::cout << "</run>" << std::endl;
	}
	

	std::cout << "</timelines>" << std::endl;

	std::cout << "<branch-info>" << std::endl;
	for (const auto& [branch_addr, branch_info] : info_on_branches) {
		auto& queries = branch_info.queries;

		// An address only enters the map when a query is recorded against it, so an empty entry
		// should be impossible - but emitting one would produce a branch that looks free rather
		// than one that looks wrong, so refuse instead.
		if (queries.empty()) {
			throw "branch info with no queries";
		}

		// The address comes from the map key: Branch_Info deliberately keeps no copy of it, since
		// a second copy could only ever drift from the key.
		std::cout << std::hex << "<branch addr=\"" << branch_addr << std::dec;
		std::cout << "\" num_queries=\"" << queries.size();

		// These aggregates are DERIVED from the <query> children below, which are the authoritative
		// record. They stay because they are what a human scanning the file wants and what every
		// consumer would otherwise recompute first; if the two ever disagree, the children win.
		//
		// seconds carries both a max and a total because they answer different questions - the max
		// is the one pathological query, the total is what the branch actually cost, and a branch
		// queried 200 times at 5ms outranks one queried once at 50ms only on the total. The other
		// four are complexity measures rather than costs, so only their max means anything.
		auto max_by = [&queries](auto field) {
			auto best = field(queries.front());
			for (const auto& q : queries)
				best = std::max(best, field(q));
			return best;
		};
		float seconds_total = 0.0f;
		for (const auto& q : queries)
			seconds_total += q.seconds;

		std::cout << "\" seconds_max=\"" << max_by([](const Query_Info& q) { return q.seconds; });
		std::cout << "\" seconds_total=\"" << seconds_total;
		std::cout << "\" constraints=\"" << max_by([](const Query_Info& q) { return q.constraints; });
		std::cout << "\" variables=\"" << max_by([](const Query_Info& q) { return q.variables; });
		std::cout << "\" nodes=\"" << max_by([](const Query_Info& q) { return q.nodes; });
		std::cout << "\" depth=\"" << max_by([](const Query_Info& q) { return q.depth; });
		std::cout << "\">" << std::endl;

		// One element per solver query, in the order the queries were issued (document order is the
		// only thing that records that order - there is no explicit index). Kept per-query rather
		// than aggregated away because every aggregate discards the distribution, and the
		// distribution is what separates "this branch is uniformly slow" from "this branch has one
		// pathological query among many fast ones". No max/total pair can express that difference,
		// and it cannot be recovered afterwards without re-running the whole exploration - which is
		// why it is emitted here even though nothing downstream consumes it yet.
		for (const auto& q : queries) {
			// run_id/step name the execution that FIRST reached this branch - the event the query
			// is about - not the run the query may go on to create. step restarts each run, so the
			// pair is the identity; neither half alone is.
			std::cout << "  <query run_id=\"" << q.run_id;
			std::cout << "\" step=\"" << q.step;
			std::cout << "\" seconds=\"" << q.seconds;
			std::cout << "\" constraints=\"" << q.constraints;
			std::cout << "\" variables=\"" << q.variables;
			std::cout << "\" nodes=\"" << q.nodes;
			std::cout << "\" depth=\"" << q.depth;
			std::cout << "\"></query>" << std::endl;
		}

		std::cout << "</branch>" << std::endl;
	}
	std::cout << "</branch-info>" << std::endl;

	std::cout << "</trace>" << std::endl;

	return ret;
}

#endif
