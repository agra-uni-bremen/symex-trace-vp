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

#ifndef RISCV_ISA_SYMBOLIC_CTX_H
#define RISCV_ISA_SYMBOLIC_CTX_H

#include <clover/clover.h>

class SymbolicContext {
public:
	clover::Solver solver;
	clover::Trace trace;
	clover::ExecutionContext ctx;

	SymbolicContext(void);
};

struct SymbolicLink {
	uint32_t pc;
	uint64_t step;
	uint64_t hash;
};

extern SymbolicContext symbolic_context;

extern uint32_t symolic_run_id;

extern SymbolicLink symbolic_run_links[20][30];
extern uint32_t runs_created_by_current_run;
extern uint32_t symbolic_run_links_to_parent[30][2];
#endif
