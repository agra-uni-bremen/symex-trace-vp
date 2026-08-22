#include <stdint.h>
#include <stddef.h>

extern void symex_error(void);
extern void make_symbolic(void *, size_t);

#define MY_ASSERT(COND) \
	((COND) ? (void)0 : symex_error())

/*
 * Solver time that buys nothing: the fixture for UNSATISFIABLE queries.
 *
 * Every other example here produces exactly one query per gap, and every one of
 * them succeeds - `symbolic-loop` is 16 runs and 15 queries, `assertion-failure`
 * 3 and 2. That is because Trace::findNewPath negates one un-negated branch,
 * gets a satisfying assignment first try, and returns. So in those traces every
 * second of solver time produced a new path, and the "time spent proving no path
 * exists" figure is structurally zero.
 *
 * That is not what real exploration looks like, and a consumer built only
 * against those fixtures would never exercise the case. Hence this one.
 *
 * Each pair below is satisfiable on its own and contradictory together, so once
 * the outer condition is in the path condition, negating the inner one asks the
 * solver for a value that cannot exist. findNewPath keeps going after an UNSAT
 * result - it marks that branch negated and picks another - so the cost is paid,
 * recorded with sat="false", and no run comes of it.
 *
 * The trailing parity test is deliberately feasible, so the trace is a MIX
 * rather than uniformly infeasible: a view that accidentally swapped the sat
 * flag would still look plausible against an all-UNSAT trace.
 */

int
main(void)
{
	uint32_t a;
	int hits = 0;

	make_symbolic(&a, sizeof(a));

	/* a > 100 && a < 50 - no such value. */
	if (a > 100) {
		if (a < 50)
			hits++;
	}

	/* a < 10 && a > 900 - likewise. */
	if (a < 10) {
		if (a > 900)
			hits++;
	}

	/* Two different exact values at once. */
	if (a == 7) {
		if (a == 9)
			hits++;
	}

	/* Reachable, unlike the three above. */
	if (a % 2 == 0)
		hits++;

	MY_ASSERT(hits < 2);

	return 0;
}
