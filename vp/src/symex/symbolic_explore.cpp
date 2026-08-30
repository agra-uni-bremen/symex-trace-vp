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

#include <clover/query_info.hpp>

/* Debug leaks with valgrind --leak-check=full --undef-value-errors=no
 * Also: Define valgrind here to prevent spurious Z3 memory leaks. */
#ifdef VALGRIND
#include <z3.h>
#endif

// std::max_element / std::accumulate were previously only reaching this file transitively through
// the klee headers pulled in by query_info.hpp (included above). Included explicitly so the
// branch-info aggregation below does not depend on that.
#include <algorithm>
#include <numeric>
#include <iostream>
#include <thread>
#include <future>
#include <systemc>
#include <filesystem>
#include <systemc>

#include <clover/clover.h>
#include "symbolic_explore.h"
#include "symbolic_context.h"

#include "rawmode.h"

#define TESTCASE_ENV "SYMEX_TESTCASE"
#define TIMEBUDGET_ENV "SYMEX_TIMEBUDGET"
#define ERR_EXIT_ENV "SYMEX_ERREXIT"
#define DUMP_ENV "SYMEX_DUMPALL"
#define LOOP_TIMEOUT_ENV "SYMEX_LOOP_TIMEOUT"

static bool dump_all = false;
static std::filesystem::path *testcase_path = nullptr;
static unsigned errors_found = 0;
static unsigned long paths_found = 0;

static std::chrono::duration<double, std::milli> solver_time;

#define SEED_ENV "SYMEX_SEED"

/* When each run started simulating, stopped simulating, and finished solving for the next one, as
 * SECONDS SINCE EXPLORATION START rather than durations.
 *
 * Offsets, not durations, deliberately: explore_paths tears down and re-elaborates the whole
 * SystemC context every iteration, and that time belongs to neither simulation nor solving. With
 * offsets it shows up as the gap between one run's solver_end and the next run's sim_start - i.e.
 * as arithmetic that does not close - whereas durations would silently fold it into whichever
 * neighbour a reader happened to assume. */
struct Run_Timing {
	double sim_start;
	double sim_end;
	double solver_end;
};

static std::vector<Run_Timing> run_timings;
static std::chrono::steady_clock::time_point exploration_start;

/* Which limit was in force, so a consumer can tell a truncated exploration from one that simply ran
 * out of branches. Those two look identical in the timings and mean opposite things. */
static const char *budget_mechanism = "none";
static unsigned long budget_limit_seconds = 0;

/* Set by the SIGALRM handler and read by the exploration loop. volatile sig_atomic_t because that
 * is the only thing a signal handler may portably touch - see sigalrm_handler for why this replaced
 * a _Exit() there. */
static volatile sig_atomic_t budget_expired = 0;

/* The RNG seed, recorded whether it was chosen or taken from the clock, so an interesting
 * exploration can be reproduced after the fact rather than only when someone remembered to set the
 * variable. */
static unsigned int rng_seed = 0;

static double
seconds_since_start(std::chrono::steady_clock::time_point t)
{
	return std::chrono::duration<double>(t - exploration_start).count();
}

static void
dump_stats(void)
{
	auto stime = std::chrono::duration_cast<std::chrono::seconds>(solver_time);

	std::cout << std::endl << "---" << std::endl;
	std::cout << "Unique paths found: " << paths_found << std::endl;
	std::cout << "Solver Time: " << stime.count() << " seconds" << std::endl;
	// TODO: Also dump instruction branch coverage here.

	if (errors_found > 0)
		std::cout << "Errors found: " << errors_found << std::endl;
	if (errors_found > 0 || dump_all)
		std::cout << "Testcase directory: " << *testcase_path << std::endl;
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
		errors_found++;
		auto path = dump_input("path" + std::to_string(paths_found + 1) + "_error");
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

/* Records that the budget ran out and returns; the exploration loop notices between runs and stops
 * cleanly.
 *
 * This used to call std::_Exit(EXIT_SUCCESS) directly, which skipped the epilogue that writes
 * <timelines>, <branch-info> and </trace> - so SYMEX_TIMEBUDGET produced a truncated, unparseable
 * trace WHILE STILL EXITING 0, and the documentation's advice was simply never to use it on a trace
 * you intended to load. Setting a flag instead costs one check per iteration and makes a budgeted
 * run yield a file like any other, which is what lets a budget be an experiment rather than a way
 * of throwing the result away.
 *
 * Consequence worth knowing: the flag is only read between runs, and a run's output is buffered
 * whole (ISS::run flushes its stringstream when the run completes), so a budget that expires
 * mid-simulation stops after that run finishes rather than at the instant it fired. The cut is
 * therefore always a whole number of runs, and can overshoot the limit by one run's duration. */
static void
sigalrm_handler(int signum)
{
	(void)signum;
	budget_expired = 1;
}

static void
remove_testdir(void)
{
	assert(testcase_path != nullptr);

	// Remove test directory if no test cases were
	// generated, i.e. if the directory is empty.
	errno = 0;
	rmdir(testcase_path->c_str());
	if (errno && errno != EEXIST && errno != ENOTEMPTY)
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
setupNewValues(clover::ExecutionContext &ctx, clover::Trace &tracer, uint32_t current_run)
{
	auto start = std::chrono::steady_clock::now();
	auto r = ctx.setupNewValues(tracer, current_run);
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

	std::optional<std::chrono::steady_clock::time_point> loop_deadline;
	if (char *timeout_str = getenv(LOOP_TIMEOUT_ENV)) {
		errno = 0;
		unsigned long seconds = strtoul(timeout_str, NULL, 10);
		if (!seconds && errno)
			throw std::system_error(errno, std::generic_category());
		loop_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
		/* Recorded even though setup_timeout may also have set these: both mechanisms can be active
		 * at once, and whichever actually ends the exploration is the one the trace should name -
		 * the loop below overwrites this if it is the deadline that stops it. */
		if (budget_limit_seconds == 0) {
			budget_mechanism = "loop_timeout";
			budget_limit_seconds = seconds;
		}
	}

	exploration_start = std::chrono::steady_clock::now();

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
		/* Stamped around the simulation only, so the split between "executing the program" and
		 * "deciding what to execute next" is recorded rather than inferred. Everything earlier in
		 * this iteration - the SystemC teardown and re-elaboration above - falls into neither, and
		 * shows up as the gap from the previous run's solver_end. */
		auto sim_start = std::chrono::steady_clock::now();
		if ((ret = sc_core::sc_elab_and_sim(argc, argv)))
			return ret;
		auto sim_end = std::chrono::steady_clock::now();
		if (loop_deadline) {
			auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
				*loop_deadline - std::chrono::steady_clock::now());
			if (remaining.count() <= 0) {
				new_path_exists = false;
			} else {
				std::packaged_task<bool()> task([&]() {
					return ctx.setupNewValues(tracer, symolic_run_id);
				});
				auto fut = task.get_future();
				std::thread t(std::move(task));
				auto start = std::chrono::steady_clock::now();
				if (fut.wait_for(remaining) == std::future_status::timeout) {
					t.detach();
					new_path_exists = false;
				} else {
					solver_time += std::chrono::steady_clock::now() - start;
					t.join();
					new_path_exists = fut.get();
				}
			}
		} else {
			new_path_exists = setupNewValues(ctx, tracer, symolic_run_id);
		}

		/* Per run rather than accumulated: every aggregate discards the shape of the spend, and the
		 * shape is the question. symolic_run_id is still this run's own id here - it is incremented
		 * immediately below - so run_timings stays indexed by run. */
		run_timings.push_back(Run_Timing{
		    seconds_since_start(sim_start),
		    seconds_since_start(sim_end),
		    seconds_since_start(std::chrono::steady_clock::now()),
		});

		++symolic_run_id;
		runs_created_by_current_run = 0;
		// if (dump_all && prev_error == errors_found)
		// 	dump_input("path" + std::to_string(paths_found));
		if (loop_deadline && std::chrono::steady_clock::now() >= *loop_deadline) {
			std::cout << "Loop timeout reached, stopping exploration..." << std::endl;
			budget_mechanism = "loop_timeout";
			std::cout << "</symex>" << std::endl;
			break;
		}
		/* The SIGALRM budget, checked here rather than acted on inside the handler. See
		 * sigalrm_handler for why, and for the resulting whole-runs-only granularity. */
		if (budget_expired) {
			std::cout << "Time budget exceeded, stopping exploration..." << std::endl;
			budget_mechanism = "timebudget";
			std::cout << "</symex>" << std::endl;
			break;
		}
		std::cout << "</symex>" << std::endl;
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

	budget_mechanism = "timebudget";
	budget_limit_seconds = seconds;

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

	/* Seeded from SYMEX_SEED when set, otherwise from the clock as before.
	 *
	 * This matters more than it looks: Trace::Node::randomUnnegated picks the next branch to negate
	 * with rand(), so without a fixed seed two runs of the same binary explore different paths in a
	 * different order, and a re-run under a smaller budget is not a prefix of a larger one. With
	 * it, "what would a T-second budget have explored" becomes a claim that can be checked by
	 * re-running at T. The seed is recorded in <budget> either way, so an interesting exploration
	 * can be reproduced after the fact rather than only when someone thought to set the variable.
	 *
	 * Note what a fixed seed does NOT buy: solver wall-clock still varies run to run, so the exact
	 * cut point at a given T moves even though the path sequence does not. */
	if (char *seed_str = getenv(SEED_ENV)) {
		errno = 0;
		unsigned long parsed = strtoul(seed_str, NULL, 10);
		if (!parsed && errno)
			throw std::system_error(errno, std::generic_category());
		rng_seed = (unsigned int)parsed;
	} else {
		rng_seed = (unsigned int)std::time(nullptr);
	}
	std::srand(rng_seed);

	char *testcase = getenv(TESTCASE_ENV);
	if (testcase)
		return run_test(testcase, argc, argv);
	create_testdir();

	// Whether to dump all generate test inputs.
	dump_all = getenv(DUMP_ENV) && !testcase;

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

	/* Every branch each run could have taken the other way, in execution order. A consumer
	 * reconstructs the fork tree from these: two runs share a prefix of identical entries and the
	 * first one that differs is where the later run diverged. The pc is the branch's TARGET (it is
	 * read after the pc has advanced), which is what distinguishes the two directions of the same
	 * branch - their hashes are equal, because the hash identifies the branch, not the direction.
	 *
	 * symbolic_run_links is a vector of vectors and is indexed by run id, so both bounds are its
	 * own. An earlier version walked a fixed 0..29 range with no size check at all, which is an
	 * out-of-bounds std::vector::operator[] twice over - past the end of a run's link list, and
	 * past the end of the outer vector for any run that recorded no links. It printed whatever it
	 * found, guarded only by a pc plausibility window, so uninitialised memory reached the trace as
	 * <branch> elements: observed pc 0x4fbc8e0 with step 103211274907176 in a program whose text
	 * ends below 0x10300. Two of those got as far as a consumer's JSON, where the step did not fit
	 * a 32-bit integer and the file failed to parse at all. The window is gone with the overrun;
	 * a real branch is now never truncated by it either. */
	std::cout << "<timelines>" << std::endl;
	for (uint32_t run = 0; run < paths_found; run++)
	{
		std::cout << "<run id=\"" << run << "\" >"<< std::endl;
		if (run < symbolic_run_links.size()) {
			for (const SymbolicLink &link : symbolic_run_links[run])
			{
				std::cout << std::hex << "<branch pc=\"";
				std::cout << link.pc << std::dec;
				std::cout << "\" step=\"";
				std::cout << link.step;
				std::cout << "\" hash=\"";
				std::cout << link.hash << "\"></branch>" << std::endl;
			}
		}

		std::cout << "</run>" << std::endl;
	}
	
	std::cout << "</timelines>" << std::endl;

	/* What the exploration was allowed to spend, and where it went. One <run> per run carrying
	 * offsets from exploration start - see Run_Timing for why offsets rather than durations.
	 *
	 * exhausted is NOT derivable from the timings, which is the main reason this block exists
	 * rather than being reconstructed by a consumer: an exploration that stopped because every
	 * branch had been negated looks identical to one that was cut off, and means the opposite
	 * thing. A budget view must not imply a larger budget would have helped when there was nothing
	 * left to explore. */
	std::cout << "<budget mechanism=\"" << budget_mechanism;
	std::cout << "\" limit_seconds=\"" << budget_limit_seconds;
	std::cout << "\" exhausted=\"" << (budget_expired ? "false" : "true");
	std::cout << "\" seed=\"" << rng_seed;
	std::cout << "\" total_seconds=\"" << (run_timings.empty() ? 0.0 : run_timings.back().solver_end);
	std::cout << "\" runs=\"" << run_timings.size();
	std::cout << "\">" << std::endl;
	for (size_t run = 0; run < run_timings.size(); run++) {
		std::cout << "  <run id=\"" << run;
		std::cout << "\" sim_start=\"" << run_timings[run].sim_start;
		std::cout << "\" sim_end=\"" << run_timings[run].sim_end;
		std::cout << "\" solver_end=\"" << run_timings[run].solver_end;
		std::cout << "\"></run>" << std::endl;
	}
	std::cout << "</budget>" << std::endl;

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
			// Which gap paid for this query, and whether it bought a path. Distinct from
			// run_id/step above, which say which branch it is ABOUT - see Query_Info.
			std::cout << "\" issued_in_run=\"" << q.issued_in_run;
			std::cout << "\" sat=\"" << (q.sat ? "true" : "false");
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
