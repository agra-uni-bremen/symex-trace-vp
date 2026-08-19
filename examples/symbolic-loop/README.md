# symbolic-loop

Example application illustrating **repeated solver queries at a single branch**.

## Purpose

The other examples each explore a handful of paths, and every branch in them is
queried exactly once. That makes them useless for exercising the solver
telemetry in `<branch-info>`, because `seconds_max` and `seconds_total` come out
identical whenever `num_queries` is 1 - so a consumer that reads the wrong one
of the two still looks correct.

This example exists to break that tie. At `-O0` the loop is not unrolled, so the
condition compiles to a single branch at one address; the symbolic engine
reaches that address once per iteration under a different path condition, and
each of those is its own solver query recorded against the same address.

## Usage

Requires a RV32I cross toolchain (see `../assertion-failure/README.md`).

	$ make
	$ symex-vp main > symbolic-loop.rtrace

## Results

`NBITS` is 4, so the program admits 2^4 = 16 paths and the run finishes in well
under a second. The trace ends with a single branch carrying 15 queries:

	<branch-info>
	<branch addr="100dc" num_queries="15" seconds_max="0.00427638"
	        seconds_total="0.0560024" constraints="3" variables="4"
	        nodes="234" depth="14">
	  <query seconds="0.00427638" constraints="0" variables="4" nodes="60"  depth="14"></query>
	  <query seconds="5.5484e-05" constraints="2" variables="4" nodes="176" depth="14"></query>
	  ...one per query, in the order they were issued...
	</branch>
	</branch-info>

15 rather than 16 because the first path is executed concretely and needs no
query; the remaining 15 are each found by negating one branch condition.

The `<query>` children are the authoritative record and the attributes on
`<branch>` are derived from them. This example is also the clearest illustration
of why the children are kept: one query lands at 5.5e-05 while most sit around
4e-03, and `nodes` climbs from 60 to 234 as constraints accumulate. Both facts
are invisible in the aggregates.

**The counts are deterministic, the times are not.** `num_queries`, the run
count and the complexity figures reproduce exactly; `seconds_max` and
`seconds_total` are wall-clock measurements and vary by a factor of two or more
between runs. Anything asserting on this fixture should check `num_queries` and
the invariants `seconds_max <= seconds_total <= seconds_max * num_queries`,
never a literal duration.

Raising `NBITS` doubles the path count per bit. Do not reach for
`SYMEX_TIMEBUDGET` to bound a larger run whose trace you intend to parse: its
`SIGALRM` handler calls `_Exit()` before the epilogue writes `<timelines>`,
`<branch-info>` and `</trace>`, so the trace is truncated mid-document and the
process still exits 0.
