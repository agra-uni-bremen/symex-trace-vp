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

#include <stdlib.h>

#include "symbolic_context.h"

#define TIMEOUT_ENV "SYMEX_TIMEOUT"

// We need to pass the SymbolicContext which includes the solver,
// tracer, … to the sc_main method somehow. This cannot be done using
// function paramaters, for this reason a global variable is used
// instead.
SymbolicContext symbolic_context = SymbolicContext();

uint32_t symolic_run_id = 0;

SymbolicLink symbolic_run_links[20][30]; //[run_id][branch](pc,step,hash)
uint32_t runs_created_by_current_run = 0;

uint32_t symbolic_run_links_to_parent[30][2];//parent id and start pc

SymbolicContext::SymbolicContext(void)
	: solver(), trace(solver), ctx(solver)
{
	char *tm;

	if ((tm = getenv(TIMEOUT_ENV))) {
		auto timeout = klee::time::Span(tm);
		solver.setTimeout(timeout);
	}
}
