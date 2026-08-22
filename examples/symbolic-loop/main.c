#include <stdint.h>
#include <stddef.h>

extern void symex_error(void);
extern void make_symbolic(void *, size_t);

#define MY_ASSERT(COND) \
	((COND) ? (void)0 : symex_error())

/*
 * One branch instruction, many solver queries.
 *
 * At -O0 the loop is not unrolled, so `a & (1u << i)` compiles to a single
 * branch at one address. The symbolic engine still reaches that address once
 * per iteration, each time under a different path condition, and every one of
 * those becomes its own solver query recorded against the SAME address. That
 * is what drives num_queries above 1 - and therefore the only thing that makes
 * seconds_total differ from seconds_max.
 *
 * NBITS controls the blowup: the loop admits 2^NBITS paths. Four is enough to
 * demonstrate the effect while still finishing well under a second.
 *
 * This example produces NO unsatisfiable queries - every gap's first negation
 * succeeds - so it is the wrong fixture for anything measuring solver time that
 * bought nothing. See examples/symbolic-infeasible for that.
 */
#define NBITS 4

int
main(void)
{
	uint32_t a;
	int bits = 0;

	make_symbolic(&a, sizeof(a));

	for (int i = 0; i < NBITS; i++) {
		if (a & (1u << i))
			bits++;
	}

	/* Reachable only when every bit is set - one error path out of 2^NBITS. */
	MY_ASSERT(bits != NBITS);

	return 0;
}
